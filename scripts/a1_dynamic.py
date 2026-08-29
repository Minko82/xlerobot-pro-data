#!/usr/bin/env python3
"""A1-dynamic — rail integrity while the base DRIVES and the arms WORK.

The static A1 sequence enables all seventeen actuators and commands a hold pose,
and measures almost nothing: goals are parked on the present position, so the
arms draw ~470 mA holding themselves up and the wheels draw nothing at all. That
is not the failure mode. The reported brownout happens when the base accelerates
while the arms are moving -- wheel inrush and arm gravity load landing on the
same battery at the same instant.

So this drives both at once, deliberately in phase:

    * the base runs alternating ramp/dwell/ramp legs, as B1 does, so the robot
      oscillates about its start point instead of driving across the room and
      tearing out its own tether
    * both arms slew a sinusoid locked to the leg period, so peak arm lift
      coincides with peak wheel acceleration rather than averaging against it

Four things are logged together, which is the whole point -- a sag is only
evidence if you can see what the motors were doing when it happened:

    compute rail   INA3221 on the Jetson, ~750 Hz, in a background thread
    motor rails    every servo reports Present_Voltage; the minimum per bus
    bus current    summed Present_Current, split by bus
    reset          boot_id compared across the run

Acceleration stays inside the published envelope. The firmware caps base
acceleration at 0.152 m/s^2 (Acceleration=20, 8.7 deg/s^2 per count, 0.05 m wheel
radius), so commanding more than that only means the profile is not realized --
it does not buy more current.

SAFETY. Wheels are stopped and VERIFIED in a finally, on SIGINT and on SIGTERM.
Arm torque is bled off gradually so the arms settle rather than drop. Every goal
is parked on the joint's present position before torque is enabled, because after
a power cycle Goal_Position reads 0 and a joint resting at 3487 counts would slam
307 degrees into its stop at full torque. Wheel current is watched against an
abort threshold every sample.

Usage:
    python scripts/a1_dynamic.py --out A1/dynamic_a015 --cycles 6
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from xlerobot_pro.firmware_limits import (
    ARM_ACCELERATION,
    ARM_TORQUE_LIMIT,
    TORQUE_RELEASE_SECONDS,
    WHEEL_NECK_ACCELERATION,
    WHEEL_NECK_TORQUE_LIMIT,
)

from a1_brownout import (
    CURRENT_UNIT_MA,
    RAIL_V,
    RailSampler,
    boot_id,
    bus_voltages,
    discover_layout,
)

WHEELS = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
WHEEL_RADIUS = 0.05
WHEEL_ANGLES_DEG = (240.0, 0.0, 120.0)
COUNTS_PER_DEG = 4096.0 / 360.0
RETRY = 5

#: The gravity-loaded joints. Only these move: they draw the most current, and
#: they stay in the sagittal plane, so no amount of amplitude can swing one arm
#: into the other the way shoulder_pan could.
SLEW = ("left_shoulder_lift", "left_elbow_flex",
        "right_shoulder_lift", "right_elbow_flex")

#: Per-wheel abort, raw counts (~6.5 mA each; 300 ~= 1.95 A). Three wheels at
#: this level is ~5.9 A against Bus A's 10 A fuse, which also carries the neck.
WHEEL_CURRENT_ABORT = 300

#: Refuse amplitudes that could drive a joint into a hard stop from an unknown
#: start pose. 25 deg is enough to make the arms visibly work against gravity.
MAX_ARM_AMPLITUDE_DEG = 25.0
MAX_ACCEL = 0.16     # m/s^2 -- the firmware cap is 0.152; no headroom on purpose
MAX_PEAK_VEL = 0.4   # m/s
MAX_CYCLES = 20


def degps_to_raw(degps: float) -> int:
    return max(-0x8000, min(0x7FFF, int(round(degps * 4096.0 / 360.0))))


def body_to_wheel_raw(vx: float, vy: float, omega_degps: float) -> dict[str, int]:
    angles = [math.radians(a - 90.0) for a in WHEEL_ANGLES_DEG]
    degps = [(math.cos(a) * vx + math.sin(a) * vy + 0.125 * math.radians(omega_degps))
             / WHEEL_RADIUS * 180.0 / math.pi for a in angles]
    return {n: degps_to_raw(d) for n, d in zip(WHEELS, degps)}


def stop_wheels(bus, verify: bool = True) -> bool:
    """Command zero and confirm by reading back. The one failure that matters."""
    for _ in range(4):
        for n in WHEELS:
            try:
                bus.write("Goal_Velocity", n, 0, num_retry=RETRY)
            except Exception:
                pass
        if not verify:
            return True
        time.sleep(0.15)
        try:
            if max(abs(bus.read("Present_Velocity", n, normalize=False, num_retry=RETRY))
                   for n in WHEELS) <= 5:
                return True
        except Exception:
            continue
    return False


def release_gently(buses, by_bus, seconds: float = TORQUE_RELEASE_SECONDS, steps: int = 24):
    """Bleed torque to zero so the arms settle instead of dropping."""
    try:
        for i in range(steps - 1, -1, -1):
            limit = int(ARM_TORQUE_LIMIT * i / steps)
            for lb, bus in buses:
                for n in by_bus[lb]:
                    if n in WHEELS:
                        continue
                    try:
                        bus.write("Torque_Limit", n, limit, num_retry=2)
                    except Exception:
                        pass
            time.sleep(seconds / steps)
    finally:
        for lb, bus in buses:
            for n in by_bus[lb]:
                if n in WHEELS:
                    continue
                try:
                    bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT, num_retry=2)
                except Exception:
                    pass


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--accel", type=float, default=0.15,
                   help="Base acceleration, m/s^2. Firmware caps realization at 0.152.")
    p.add_argument("--peak-vel", type=float, default=0.30, help="Peak velocity, m/s.")
    p.add_argument("--dwell", type=float, default=0.4, help="Seconds at peak velocity.")
    p.add_argument("--pause", type=float, default=1.5, help="Seconds between legs.")
    p.add_argument("--cycles", type=int, default=6, help="Legs. Direction alternates.")
    p.add_argument("--arm-amplitude", type=float, default=20.0,
                   help="Arm slew amplitude, degrees peak.")
    p.add_argument("--rate", type=float, default=20.0, help="Control/log rate, Hz.")
    p.add_argument("--battery", type=float, default=None, help="Powerbank charge, percent.")
    p.add_argument("--note", default="", help="Free text recorded with the run.")
    args = p.parse_args()

    if not (0 < args.accel <= MAX_ACCEL):
        print(f"FATAL: --accel must be in (0, {MAX_ACCEL}]."); return 1
    if not (0 < args.peak_vel <= MAX_PEAK_VEL):
        print(f"FATAL: --peak-vel must be in (0, {MAX_PEAK_VEL}]."); return 1
    if not (0 < args.cycles <= MAX_CYCLES):
        print(f"FATAL: --cycles must be in (0, {MAX_CYCLES}]."); return 1
    if not (0 <= args.arm_amplitude <= MAX_ARM_AMPLITUDE_DEG):
        print(f"FATAL: --arm-amplitude must be in [0, {MAX_ARM_AMPLITUDE_DEG}]."); return 1
    if not RAIL_V.exists():
        print(f"FATAL: {RAIL_V} missing -- INA3221 rail monitoring unavailable."); return 1

    args.out.mkdir(parents=True, exist_ok=True)

    # Probe the wiring before anything is energised. This refuses unless all 17
    # motors are present and uniquely addressed, which is what makes a null
    # result meaningful rather than an artefact of a chain that fell off.
    layout = discover_layout()
    motors = layout["motors"]
    where = {n: lb for lb, group in motors.items() for n in group}
    wheel_bus_label = where[WHEELS[0]]

    print(f"\n  base: accel {args.accel} m/s^2, peak {args.peak_vel} m/s, "
          f"{args.cycles} legs")
    print(f"  arms: +/-{args.arm_amplitude:g} deg on {', '.join(SLEW)}")
    print("\n  The base WILL move. Clear a few metres, mind the tether, and keep a")
    print("  hand near the power switch.")
    try:
        input("  ENTER to start, Ctrl-C to abort: ")
    except (EOFError, KeyboardInterrupt):
        print("\n  aborted."); return 1

    buses = [(lb, FeetechMotorsBus(port=("/dev/xle_arms" if lb == "arms" else "/dev/xle_head"),
                                   motors=motors[lb])) for lb in ("arms", "head")]
    for _, bus in buses:
        bus.connect()
    bus_of = dict(buses)
    by_bus: dict[str, list[str]] = {lb: [] for lb, _ in buses}

    stopping = {"flag": False}

    def on_signal(_s, _f):
        stopping["flag"] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    stop_evt = threading.Event()
    sampler = RailSampler(args.out / "rail.csv", stop_evt)
    rows: list[dict] = []
    boot_before = boot_id()
    aborted = ""

    sampler.start()
    time.sleep(1.0)

    try:
        # --- enable everything, wheels in velocity mode, arms goal-parked ---
        sampler.phase = "enable"
        print("\n  enabling 17 actuators...")
        for lb, group in motors.items():
            bus = bus_of[lb]
            for name in group:
                if name in WHEELS:
                    bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value, num_retry=RETRY)
                    bus.write("Torque_Limit", name, WHEEL_NECK_TORQUE_LIMIT, num_retry=RETRY)
                    bus.write("Acceleration", name, WHEEL_NECK_ACCELERATION, num_retry=RETRY)
                    bus.write("Goal_Velocity", name, 0, num_retry=RETRY)
                else:
                    bus.write("Torque_Limit", name, ARM_TORQUE_LIMIT, num_retry=RETRY)
                    bus.write("Acceleration", name, ARM_ACCELERATION, num_retry=RETRY)
                    here = bus.read("Present_Position", name, normalize=False, num_retry=RETRY)
                    bus.write("Goal_Position", name, here, normalize=False, num_retry=RETRY)
                bus.enable_torque([name], num_retry=RETRY)
                by_bus[lb].append(name)
        sampler.active = 17
        time.sleep(0.5)

        start = {}
        for n in SLEW:
            start[n] = bus_of[where[n]].read("Present_Position", n, normalize=False, num_retry=RETRY)
        print(f"  arm start counts: {start}")

        ramp_t = args.peak_vel / args.accel
        total = ramp_t + args.dwell + ramp_t
        period = 1.0 / args.rate
        t0 = time.monotonic()
        head = bus_of[wheel_bus_label]

        for leg in range(args.cycles):
            if stopping["flag"]:
                break
            sign = 1.0 if leg % 2 == 0 else -1.0
            leg_t0 = time.monotonic()
            sampler.phase = f"leg{leg}_{'fwd' if sign > 0 else 'rev'}"
            while not stopping["flag"]:
                lt = time.monotonic() - leg_t0
                if lt >= total:
                    break
                if lt < ramp_t:
                    v, phase = args.accel * lt, "ramp_up"
                elif lt < ramp_t + args.dwell:
                    v, phase = args.peak_vel, "dwell"
                else:
                    v = max(0.0, args.peak_vel - args.accel * (lt - ramp_t - args.dwell))
                    phase = "ramp_down"

                cmd = body_to_wheel_raw(sign * v, 0.0, 0.0)
                for n in WHEELS:
                    head.write("Goal_Velocity", n, cmd[n], num_retry=2)

                # Arms slew in phase with the leg, so peak lift lands on peak
                # wheel acceleration instead of averaging against it.
                delta = args.arm_amplitude * COUNTS_PER_DEG * math.sin(2.0 * math.pi * lt / total)
                for lb in ("arms", "head"):
                    goals = {n: int(round(start[n] + (delta if "shoulder_lift" in n else -delta)))
                             for n in SLEW if where[n] == lb}
                    if goals:
                        try:
                            bus_of[lb].sync_write("Goal_Position", goals, normalize=False, num_retry=2)
                        except Exception:
                            pass

                volts = bus_voltages(buses, by_bus)
                per_bus, total_ma = {}, 0.0
                for lb, bs in buses:
                    try:
                        cur = bs.sync_read("Present_Current", by_bus[lb], normalize=False)
                        amps = sum(abs(int(x)) for x in cur.values()) * CURRENT_UNIT_MA
                    except Exception:
                        continue
                    per_bus[lb] = amps
                    total_ma += amps
                try:
                    wcur = head.sync_read("Present_Current", list(WHEELS), normalize=False)
                except Exception:
                    wcur = dict.fromkeys(WHEELS, 0)
                peak_w = max(abs(int(x)) for x in wcur.values()) if wcur else 0
                if peak_w > WHEEL_CURRENT_ABORT:
                    aborted = (f"wheel current {peak_w} counts (~{peak_w*6.5/1000:.1f} A) "
                               f"exceeded {WHEEL_CURRENT_ABORT}")
                    print(f"\n  ABORT: {aborted}")
                    stopping["flag"] = True

                rows.append(dict(
                    elapsed_s=round(time.monotonic() - t0, 3), leg=leg, sign=int(sign),
                    phase=phase, timestamp=datetime.now(timezone.utc).isoformat(),
                    cmd_vx=round(sign * v, 4), arm_delta_deg=round(delta / COUNTS_PER_DEG, 2),
                    arms_ma=round(per_bus.get("arms", 0.0), 1),
                    head_ma=round(per_bus.get("head", 0.0), 1),
                    total_ma=round(total_ma, 1),
                    arms_mv=round(volts.get("arms", float("nan")), 1),
                    head_mv=round(volts.get("head", float("nan")), 1),
                    **{f"cur_{n}": int(wcur.get(n, 0)) for n in WHEELS}))
                time.sleep(max(0.0, period - ((time.monotonic() - leg_t0) - lt)))

            stop_wheels(head, verify=False)
            last = rows[-1] if rows else {}
            print(f"    leg {leg+1}/{args.cycles} ({'fwd' if sign > 0 else 'rev'})"
                  f"  peak_total {max((r['total_ma'] for r in rows), default=0):.0f} mA"
                  f"  rail_min {min((r['head_mv'] for r in rows if r['head_mv'] == r['head_mv']), default=0)/1000:.1f} V")
            end = time.monotonic() + args.pause
            while time.monotonic() < end and not stopping["flag"]:
                time.sleep(0.05)
    finally:
        ok = stop_wheels(bus_of[wheel_bus_label])
        print(f"\n  wheels stopped: {'VERIFIED' if ok else 'NOT VERIFIED -- CUT POWER'}")
        try:
            bus_of[wheel_bus_label].disable_torque(list(WHEELS), num_retry=RETRY)
        except Exception:
            pass
        print(f"  lowering arms over {TORQUE_RELEASE_SECONDS:g}s...")
        release_gently(buses, by_bus)
        stop_evt.set()
        sampler.join(timeout=5)
        for _, bus in buses:
            try:
                bus.disconnect(disable_torque=True)
            except Exception:
                pass

    boot_after = boot_id()
    if rows:
        with (args.out / "motion.csv").open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)

    def col(key, fn):
        vals = [r[key] for r in rows if isinstance(r.get(key), (int, float)) and r[key] == r[key]]
        return fn(vals) if vals else 0.0

    rail = [int(r["millivolts"]) for r in csv.DictReader((args.out / "rail.csv").open())] \
        if (args.out / "rail.csv").exists() else []
    summary = dict(
        timestamp=datetime.now(timezone.utc).isoformat(), samples=len(rows),
        accel=args.accel, peak_vel=args.peak_vel, cycles=args.cycles,
        arm_amplitude_deg=args.arm_amplitude, battery=args.battery, note=args.note,
        aborted=aborted, reset="Y" if boot_before != boot_after else "N",
        compute_v_idle_mv=rail[0] if rail else 0, compute_v_min_mv=min(rail) if rail else 0,
        arms_rail_idle_mv=col("arms_mv", max), arms_rail_min_mv=col("arms_mv", min),
        head_rail_idle_mv=col("head_mv", max), head_rail_min_mv=col("head_mv", min),
        peak_total_ma=col("total_ma", max), peak_arms_ma=col("arms_ma", max),
        peak_head_ma=col("head_ma", max))
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2))

    print(f"\n  === {args.out} ===")
    print(f"  JETSON RESET : {summary['reset']}"
          + ("   <-- BROWNOUT REPRODUCED" if summary["reset"] == "Y" else ""))
    print(f"  compute rail : idle {summary['compute_v_idle_mv']} mV  "
          f"min {summary['compute_v_min_mv']} mV  "
          f"dip {summary['compute_v_idle_mv'] - summary['compute_v_min_mv']} mV")
    for lb in ("arms", "head"):
        idle, lo = summary[f"{lb}_rail_idle_mv"], summary[f"{lb}_rail_min_mv"]
        print(f"  {lb:5s} rail   : idle {idle/1000:.1f} V  min {lo/1000:.1f} V  "
              f"sag {(idle - lo)/1000:.1f} V"
              + ("   <-- below 9 V Jetson minimum" if 0 < lo < 9000 else ""))
    print(f"  peak current : total {summary['peak_total_ma']:.0f} mA  "
          f"(arms {summary['peak_arms_ma']:.0f}, head {summary['peak_head_ma']:.0f})")
    if aborted:
        print(f"  ABORTED      : {aborted}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
