#!/usr/bin/env python3
"""C1 — drive an UNSIZED configuration until something breaks.

Table IV's "Naive Spec" column is a *computed counterfactual*: the value a builder
would get by composing datasheet numbers directly. The draft says so, honestly, and
that honesty is also the weakness — the paper claims an unsized build fails without
ever having observed one fail. One recorded failure converts that column from
arithmetic into evidence, and it is the cheapest hour on the whole list.

So this raises the envelope past its documented limits and drives both buses in
phase until the platform gives way, then says which way it gave.

    firmware_limits.py, on the margins this deliberately spends:
        Bus A (wheels/neck)  accel 20, tau 650   9.84 A against a 10 A fuse
        Bus B (arms)         accel 40, tau 450   4.90 A against a  5 A fuse

**It escalates in a ladder, not a jump.** Failing at the top of a single big step
tells you only that something broke somewhere; failing at a known rung tells you
*where* the envelope stops being survivable, which is the number Table IV wants.
One axis moves at a time for the same reason.

The wheel-acceleration ladder is the one that maps onto the paper. Table IV's
naive base acceleration is 1.00 m/s^2, and

    a = A_reg x 8.7 deg/s^2 x pi/180 x 0.05 m

puts that at A_reg = 132 against the deployed 20. That rung is the claim.

FAILURE CLASSES, all detected here, none silently retried:

    RESET     compute brownout -- kernel boot id changed mid-run
    SAG       compute rail fell more than --sag-pct below its stage-0 baseline
    LATCH     servo overload latch: output collapses, the joint drops and does
              NOT self-recover. Per A2/STATE.md one observed collapse was 1195
              counts (~105 deg). This is the dangerous one.
    THERMAL   a servo reached SERVO_TEMP_CEILING_C
    DARK      a bus stopped answering -- blown fuse or lost supply
    OVERCUR   summed bus current passed --abort-ma

SAFETY. This script exists to break something, so treat it that way.

  * NO PAYLOAD. A latch drops the arm with no warning from 1.2 m.
  * Nobody in the workspace. The base moves and the arms slew.
  * A blown fuse is a real and intended outcome. Have spares before starting.
  * Wheels are stopped and VERIFIED in a finally, on SIGINT and on SIGTERM.
    Arm torque is bled off gradually so the arms settle instead of dropping.
  * Every goal is parked on the joint's present position before torque is
    enabled: after a power cycle Goal_Position reads 0, and a joint resting at
    3487 counts would slam.
  * EPROM is never written. Max_Torque_Limit is 800, so the arm ladder stops
    there; going past it means a persistent EPROM change, which is not something
    a test script should do behind your back.

    python scripts/c1_unsized_failure.py --out A2/c1_unsized/wheel_accel
    python scripts/c1_unsized_failure.py --axis arm-torque --out A2/c1_unsized/arm_tau
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

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from xlerobot_pro.firmware_limits import (
    ARM_ACCELERATION,
    ARM_TORQUE_LIMIT,
    MAX_TORQUE_EPROM,
    SERVO_TEMP_CEILING_C,
    TORQUE_RELEASE_SECONDS,
    WHEEL_NECK_ACCELERATION,
    WHEEL_NECK_TORQUE_LIMIT,
)

from a1_brownout import (
    CURRENT_UNIT_MA,
    RAIL_V,
    RailSampler,
    boot_id,
    discover_layout,
)
from a1_dynamic import (
    SLEW,
    WHEELS,
    body_to_wheel_raw,
    stop_wheels,
)

#: Register -> m/s^2 for the base. 8.7 deg/s^2 per count, 0.05 m wheel radius.
ACCEL_PER_COUNT = 8.7 * math.pi / 180.0 * 0.05

#: Wheel-acceleration ladder. 20 is deployed; 132 is Table IV's naive 1.00 m/s^2;
#: 254 is the register ceiling. The rungs between exist so the failure has an
#: address rather than just a verdict.
WHEEL_ACCEL_LADDER = (20, 40, 70, 100, 132, 190, 254)

#: Arm-torque ladder. 450 is deployed. It stops at MAX_TORQUE_EPROM because the
#: firmware clamps Torque_Limit to the EPROM ceiling and this script will not
#: rewrite EPROM.
ARM_TORQUE_LADDER = (450, 550, 650, 725, MAX_TORQUE_EPROM)

#: Fuse rating per bus, milliamps. The arms sit behind 5 A and the wheels+neck
#: behind 10 A, so one shared abort threshold is meaningless -- 9 A would let the
#: arm bus burn its fuse without ever tripping.
BUS_FUSE_MA = {"arms": 5000.0, "head": 10000.0}

#: A joint this far from its goal while torque is enabled has latched, not lagged.
#: Well above normal gravity droop under load, well below the observed 1195.
LATCH_DEVIATION_COUNTS = 400

#: Consecutive samples the deviation must persist before it counts. One bad read
#: on a busy bus is not a latch.
LATCH_SAMPLES = 3

#: Arm slew amplitude. Bigger than a1_dynamic's cap on purpose -- gravity load is
#: what makes the arm bus draw -- but still short of any hard stop.
ARM_AMPLITUDE_DEG = 25.0

RETRY = 5


def read_rail_mv() -> int | None:
    try:
        return int(RAIL_V.read_text())
    except (OSError, ValueError):
        return None


def park_goals(bus, names) -> None:
    """Goal := present position, so enabling torque cannot slam a joint.

    Raw both ways. Reading normalize=False and writing normalized would put the
    goal somewhere else entirely, which is the exact slam this exists to prevent.
    """
    for n in names:
        try:
            here = bus.read("Present_Position", n, normalize=False, num_retry=RETRY)
            bus.write("Goal_Position", n, here, normalize=False, num_retry=RETRY)
        except Exception:
            pass


def release_gently(buses, arm_names, seconds=TORQUE_RELEASE_SECONDS, steps=24) -> None:
    """Bleed torque to zero so the arms settle instead of dropping."""
    try:
        for i in range(steps - 1, -1, -1):
            limit = int(ARM_TORQUE_LIMIT * i / steps)
            for lb, bus in buses:
                for n in arm_names.get(lb, []):
                    try:
                        bus.write("Torque_Limit", n, limit, num_retry=2)
                    except Exception:
                        pass
            time.sleep(seconds / steps)
    finally:
        for lb, bus in buses:
            for n in arm_names.get(lb, []):
                try:
                    bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT, num_retry=2)
                except Exception:
                    pass


def restore_envelope(buses, by_bus) -> None:
    """Put every register back to the documented envelope, whatever happened."""
    for lb, bus in buses:
        for n in by_bus[lb]:
            arm = n.startswith(("left_", "right_"))
            try:
                bus.write("Torque_Limit", n,
                          ARM_TORQUE_LIMIT if arm else WHEEL_NECK_TORQUE_LIMIT, num_retry=2)
                bus.write("Acceleration", n,
                          ARM_ACCELERATION if arm else WHEEL_NECK_ACCELERATION, num_retry=2)
            except Exception:
                pass


def run_stage(stage, value, args, ctx) -> dict:
    """One rung: set the register, drive both buses in phase, watch for failure.

    Returns a row describing the rung and, if the platform gave way, the class it
    gave way in. Escalation stops on the first non-empty ``failure``.
    """
    buses, by_bus, bus_of, wheel_bus, sampler, writer = (
        ctx["buses"], ctx["by_bus"], ctx["bus_of"], ctx["wheel_bus"],
        ctx["sampler"], ctx["writer"])
    axis = args.axis
    sampler.phase = f"stage{stage}:{axis}={value}"

    # --- apply the rung -------------------------------------------------------
    for lb, bus in buses:
        for n in by_bus[lb]:
            arm = n.startswith(("left_", "right_"))
            try:
                if axis == "wheel-accel" and n in WHEELS:
                    bus.write("Acceleration", n, value, num_retry=RETRY)
                elif axis == "arm-torque" and arm:
                    bus.write("Torque_Limit", n, value, num_retry=RETRY)
            except Exception:
                pass

    accel_cmd = (value * ACCEL_PER_COUNT if axis == "wheel-accel"
                 else WHEEL_NECK_ACCELERATION * ACCEL_PER_COUNT)
    print(f"\n  stage {stage}: {axis} = {value}"
          + (f"  ({accel_cmd:.3f} m/s^2 commanded)" if axis == "wheel-accel" else ""))

    goals = {}
    for lb, bus in buses:
        for n in by_bus[lb]:
            if n in SLEW:
                try:
                    goals[n] = bus.read("Present_Position", n, normalize=False, num_retry=RETRY)
                except Exception:
                    pass

    latch_run = {n: 0 for n in goals}
    rail_min, cur_max, temp_max = 10 ** 9, 0, 0
    failure, detail = "", ""
    t0 = time.monotonic()
    period = 1.0 / args.rate
    leg = 0

    while time.monotonic() - t0 < args.stage_seconds and not failure:
        loop_t = time.monotonic()
        el = loop_t - t0
        # Base oscillates about its start point so the tether is never dragged out.
        direction = 1.0 if int(el / args.leg_seconds) % 2 == 0 else -1.0
        if int(el / args.leg_seconds) != leg:
            leg = int(el / args.leg_seconds)
        try:
            for n, raw in body_to_wheel_raw(direction * args.peak_vel, 0.0, 0.0).items():
                bus_of[wheel_bus].write("Goal_Velocity", n, raw, num_retry=2)
        except Exception:
            pass

        # Arms slew in phase with the base, so peak lift lands on peak wheel draw.
        phase = math.sin(2.0 * math.pi * el / (2.0 * args.leg_seconds))
        for n, g in goals.items():
            try:
                bus_of[ctx["where"][n]].write(
                    "Goal_Position", n,
                    int(g + phase * ARM_AMPLITUDE_DEG * 4096.0 / 360.0),
                    normalize=False, num_retry=2)
            except Exception:
                pass

        # --- watch ------------------------------------------------------------
        if boot_id() != ctx["boot0"]:
            failure, detail = "RESET", "kernel boot id changed"
            break

        mv = read_rail_mv()
        if mv:
            rail_min = min(rail_min, mv)
            if ctx["rail_base"] and mv < ctx["rail_base"] * (1 - args.sag_pct / 100.0):
                failure = "SAG"
                detail = f"{mv} mV against a {ctx['rail_base']} mV baseline"
                break

        dark = []
        for lb, bus in buses:
            names = by_bus[lb]
            if not names:
                continue
            try:
                cur = bus.sync_read("Present_Current", names, normalize=False, num_retry=2)
                tmp = bus.sync_read("Present_Temperature", names, normalize=False, num_retry=2)
                pos = bus.sync_read("Present_Position", names, normalize=False, num_retry=2)
            except Exception:
                dark.append(lb)
                continue
            total = sum(abs(int(v)) for v in cur.values())
            cur_max = max(cur_max, total)
            temp_max = max(temp_max, max(int(v) for v in tmp.values()))
            limit = BUS_FUSE_MA[lb] * args.abort_frac
            if total * CURRENT_UNIT_MA > limit:
                failure = "OVERCUR"
                detail = (f"bus {lb} at {total * CURRENT_UNIT_MA:.0f} mA, "
                          f"{args.abort_frac:.0%} of its {BUS_FUSE_MA[lb]:.0f} mA fuse")
            if temp_max >= SERVO_TEMP_CEILING_C:
                failure = "THERMAL"
                detail = f"{temp_max} C on bus {lb}"
            for n, g in goals.items():
                if n not in pos:
                    continue
                dev = abs(int(pos[n]) - int(g + phase * ARM_AMPLITUDE_DEG * 4096.0 / 360.0))
                latch_run[n] = latch_run[n] + 1 if dev > LATCH_DEVIATION_COUNTS else 0
                if latch_run[n] >= LATCH_SAMPLES:
                    failure = "LATCH"
                    detail = f"{n} off goal by {dev} counts for {LATCH_SAMPLES} samples"
        if len(dark) == len(buses):
            failure, detail = "DARK", "no bus answered -- fuse or supply"
        elif dark:
            failure, detail = "DARK", f"bus {dark[0]} stopped answering"

        time.sleep(max(0.0, period - (time.monotonic() - loop_t)))

    stop_wheels(bus_of[wheel_bus])
    row = {
        "stage": stage, "axis": axis, "value": value,
        "commanded_accel_ms2": round(accel_cmd, 4) if axis == "wheel-accel" else "",
        "seconds": round(time.monotonic() - t0, 1),
        "rail_min_mv": "" if rail_min == 10 ** 9 else rail_min,
        "bus_peak_ma": round(cur_max * CURRENT_UNIT_MA),
        "servo_max_c": temp_max,
        "failure": failure, "detail": detail,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    writer.writerow(row)
    ctx["fh"].flush()
    print(f"    rail min {row['rail_min_mv']} mV   bus peak {row['bus_peak_ma']} mA   "
          f"max {temp_max} C   " + (f"** {failure}: {detail}" if failure else "survived"))
    return row


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--axis", choices=("wheel-accel", "arm-torque"), default="wheel-accel",
                   help="Which limit to escalate. wheel-accel maps onto Table IV's "
                        "naive 1.00 m/s^2 base acceleration (register 132).")
    p.add_argument("--stage-seconds", type=float, default=25.0)
    p.add_argument("--leg-seconds", type=float, default=2.5,
                   help="Seconds per base leg. Direction alternates each leg.")
    p.add_argument("--peak-vel", type=float, default=0.30, help="Peak base velocity, m/s.")
    p.add_argument("--rate", type=float, default=20.0, help="Control/monitor rate, Hz.")
    p.add_argument("--sag-pct", type=float, default=10.0,
                   help="Compute-rail drop below the stage-0 baseline that counts as SAG.")
    p.add_argument("--abort-frac", type=float, default=0.98,
                   help="Abort when a bus passes this fraction of its own fuse rating. "
                        "Tripping just under the fuse is still an observed failure and "
                        "it is the one that leaves the hardware intact -- raise towards "
                        "1.0 only if you would rather have the fuse than the datapoint.")
    p.add_argument("--battery", type=float, default=None, help="Powerbank charge, percent.")
    p.add_argument("--note", default="")
    p.add_argument("--yes", action="store_true", help="Skip the typed confirmation.")
    args = p.parse_args()

    ladder = WHEEL_ACCEL_LADDER if args.axis == "wheel-accel" else ARM_TORQUE_LADDER
    args.out.mkdir(parents=True, exist_ok=True)

    print("\n  C1 -- unsized configuration to failure")
    print(f"  axis   : {args.axis}")
    print(f"  ladder : {' -> '.join(str(v) for v in ladder)}")
    if args.axis == "wheel-accel":
        print("           " + "    ".join(f"{v * ACCEL_PER_COUNT:.2f}" for v in ladder)
              + "  m/s^2 commanded")
    print(f"  out    : {args.out}")
    print("\n  This is designed to break something. NO PAYLOAD on either arm, nobody")
    print("  in the workspace, and have spare fuses to hand -- a blown fuse is an")
    print("  intended outcome, not a malfunction. The base moves; mind the tether.")
    if not args.yes:
        try:
            if input("\n  Type 'unsized' to start: ").strip().lower() != "unsized":
                print("  aborted."); return 1
        except (EOFError, KeyboardInterrupt):
            print("\n  aborted."); return 1

    layout = discover_layout()
    motors = layout["motors"]
    where = {n: lb for lb, group in motors.items() for n in group}
    wheel_bus = where[WHEELS[0]]

    buses = [(lb, FeetechMotorsBus(
        port=("/dev/xle_arms" if lb == "arms" else "/dev/xle_head"), motors=motors[lb]))
        for lb in ("arms", "head")]
    for _, bus in buses:
        bus.connect()
    bus_of = dict(buses)
    by_bus = {lb: list(motors[lb]) for lb, _ in buses}
    arm_names = {lb: [n for n in by_bus[lb] if n.startswith(("left_", "right_"))]
                 for lb, _ in buses}

    stop = threading.Event()
    sampler = RailSampler(args.out / "rail.csv", stop)
    fh = open(args.out / "stages.csv", "w", newline="")
    writer = csv.DictWriter(fh, fieldnames=[
        "stage", "axis", "value", "commanded_accel_ms2", "seconds", "rail_min_mv",
        "bus_peak_ma", "servo_max_c", "failure", "detail", "timestamp"])
    writer.writeheader()

    def panic(*_):
        stop_wheels(bus_of[wheel_bus], verify=False)
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, panic)
    signal.signal(signal.SIGINT, panic)

    rows, verdict = [], {"failure": "", "detail": "", "stage": None, "value": None}
    ctx = {"buses": buses, "by_bus": by_bus, "bus_of": bus_of, "where": where,
           "wheel_bus": wheel_bus, "sampler": sampler, "writer": writer, "fh": fh,
           "boot0": boot_id(), "rail_base": read_rail_mv()}
    print(f"\n  rail baseline {ctx['rail_base']} mV, boot {ctx['boot0'][:8]}")

    try:
        # Bring every motor up at the DEPLOYED envelope first. The ladder escalates
        # from a known-good baseline, so stage 0 doubles as the control.
        for lb, bus in buses:
            for n in by_bus[lb]:
                if n in WHEELS:
                    bus.write("Operating_Mode", n, OperatingMode.VELOCITY.value,
                              num_retry=RETRY)
                    bus.write("Torque_Limit", n, WHEEL_NECK_TORQUE_LIMIT, num_retry=RETRY)
                    bus.write("Acceleration", n, WHEEL_NECK_ACCELERATION, num_retry=RETRY)
                    bus.write("Goal_Velocity", n, 0, num_retry=RETRY)
                else:
                    arm = n.startswith(("left_", "right_"))
                    bus.write("Torque_Limit", n,
                              ARM_TORQUE_LIMIT if arm else WHEEL_NECK_TORQUE_LIMIT,
                              num_retry=RETRY)
                    bus.write("Acceleration", n,
                              ARM_ACCELERATION if arm else WHEEL_NECK_ACCELERATION,
                              num_retry=RETRY)
                    park_goals(bus, [n])
                bus.enable_torque([n], num_retry=RETRY)
        sampler.active = sum(len(v) for v in by_bus.values())
        sampler.start()
        time.sleep(0.5)

        for stage, value in enumerate(ladder):
            row = run_stage(stage, value, args, ctx)
            rows.append(row)
            if row["failure"]:
                verdict = {"failure": row["failure"], "detail": row["detail"],
                           "stage": stage, "value": value}
                break
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\n  interrupted")
        verdict = {"failure": "ABORT", "detail": "operator stop",
                   "stage": len(rows), "value": None}
    finally:
        stop.set()
        try:
            stop_wheels(bus_of[wheel_bus])
        except Exception:
            pass
        try:
            release_gently(buses, arm_names)
        except Exception:
            pass
        restore_envelope(buses, by_bus)
        for _, bus in buses:
            try:
                bus.disconnect(disable_torque=False)
            except Exception:
                pass
        fh.close()

        (args.out / "run_info.json").write_text(json.dumps({
            "experiment": "C1_unsized_failure",
            "axis": args.axis,
            "ladder": list(ladder),
            "deployed_envelope": {
                "wheel_neck_acceleration": WHEEL_NECK_ACCELERATION,
                "wheel_neck_torque_limit": WHEEL_NECK_TORQUE_LIMIT,
                "arm_torque_limit": ARM_TORQUE_LIMIT,
                "arm_acceleration": ARM_ACCELERATION,
            },
            "stage_seconds": args.stage_seconds,
            "peak_vel_ms": args.peak_vel,
            "sag_pct": args.sag_pct,
            "abort_frac": args.abort_frac,
            "bus_fuse_ma": BUS_FUSE_MA,
            "rail_baseline_mv": ctx["rail_base"],
            "boot_id_before": ctx["boot0"],
            "boot_id_after": boot_id(),
            "battery_percent": args.battery,
            "note": args.note,
            "verdict": verdict,
            "stages": rows,
            "finished": datetime.now(timezone.utc).isoformat(),
        }, indent=2) + "\n")

        print("\n" + "=" * 58)
        if verdict["failure"]:
            print(f"  FAILED at stage {verdict['stage']}: {args.axis} = {verdict['value']}")
            print(f"  class {verdict['failure']} -- {verdict['detail']}")
            if args.axis == "wheel-accel":
                print(f"  commanded {verdict['value'] * ACCEL_PER_COUNT:.3f} m/s^2 "
                      f"against a deployed {WHEEL_NECK_ACCELERATION * ACCEL_PER_COUNT:.3f}")
                if verdict["value"] >= 132:
                    print("  This rung is at or past Table IV's naive 1.00 m/s^2.")
        else:
            print("  Survived the whole ladder. That is a result too, and it is one the")
            print("  paper has to report: the envelope is more conservative than the")
            print("  failure it is justified by, so say so rather than dropping the run.")
        print(f"  wrote {args.out}")
        if verdict["failure"] == "RESET":
            print("  A RESET means the Jetson rebooted -- check dmesg and the SD card.")
        if verdict["failure"] == "LATCH":
            print("  A LATCH does not self-recover. Power-cycle the arm bus before reuse.")
        print("=" * 58)
    return 0


if __name__ == "__main__":
    sys.exit(main())
