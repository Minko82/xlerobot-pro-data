#!/usr/bin/env python3
"""A1 — boot-up current sequencing and compute-rail brownout margin.

Enables every actuator one at a time in a fixed order, then commands a
simultaneous high-torque pose at full count, while sampling the compute rail as
fast as the kernel allows. Repeats for n trials.

Compute-rail voltage comes from the Jetson's onboard INA3221 rather than an
oscilloscope::

    /sys/class/hwmon/hwmon1/in1_input     VDD_IN, millivolts
    /sys/class/hwmon/hwmon1/curr1_input   VDD_IN, milliamps

Measured throughput is ~760 Hz with ~8 mV resolution. That is below the
protocol's 1 kHz and the INA3221 averages internally, so a very short transient
may be smoothed: **the reported V_min is an upper bound on the true dip.** Say so
when reporting. It is entirely adequate for inrush lasting tens of milliseconds,
which is the regime that actually resets the compute module.

Motor-bus current does NOT come from curr1_input, which is the Jetson's own draw.
It is summed from the servos themselves: every STS3215 reports Present_Current, so
the per-bus total is the sum over enabled motors. That needs no bench supply and is
richer than one -- it attributes current to individual joints rather than giving a
single aggregate. Sampled once per enable step, so it is a steady-state figure per
active-motor count, not an inrush capture.

Every sample is flushed to disk as it is taken, so a brownout that resets the
board loses at most the final sample rather than the whole trial. A reset is
detected by comparing the kernel boot id across the trial.

Usage:
    python diagnostics/a1_brownout.py --config baseline --trials 10 --out results/A1/baseline
    python diagnostics/a1_brownout.py --config two-bus  --trials 10 --out results/A1/two_bus
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from xlerobot_pro.firmware_limits import (  # noqa: E402
    ARM_ACCELERATION,
    ARM_TORQUE_LIMIT,
    TORQUE_RELEASE_SECONDS,
)

ARMS_PORT = "/dev/xle_arms"
HEAD_PORT = "/dev/xle_head"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

#: Velocity-mode joints: Goal_Position does not apply to them.
WHEELS = ("base_left_wheel", "base_back_wheel", "base_right_wheel")

#: STS3215 current register unit, milliamps per count (~6.5 mA).
CURRENT_UNIT_MA = 6.5

#: STS3215 Present_Voltage unit: 0.1 V per count.
VOLT_UNIT_MV = 100

RAIL_V = Path("/sys/class/hwmon/hwmon1/in1_input")
RAIL_I = Path("/sys/class/hwmon/hwmon1/curr1_input")
BOOT_ID = Path("/proc/sys/kernel/random/boot_id")

RETRY = 5

#: Seconds to hold at each step of the enable sequence.
STEP_SECONDS = 2.0

#: Active-motor counts the protocol tabulates.
REPORT_COUNTS = (1, 4, 8, 12, 17)


#: Probing costs a second, and the wiring cannot change mid-run, so do it once.
_LAYOUT: dict | None = None


def discover_layout() -> dict:
    """Work out which group sits on which bus by asking the hardware.

    The arms can hang off either port depending on how the robot is cabled, and
    the two layouts are distinguishable without a flag:

        * ID 6 exists only on the left arm -- the neck stops at 2 and the wheels
          at 5 -- so whichever bus answers 6 carries the left arm on 1-6.
        * IDs 7-12 are the right arm wherever they turn up.
        * The bus without the left arm has the neck on 1-2, wheels on 3-5.

    Deducing this beats passing a flag because a flag can be wrong: a stale
    --wiring argument would enable torque on a motor map that does not match the
    robot. The hardware is the authority.

    Both valid layouts resolve uniquely. Anything else means two motors share an
    address on one bus, which garbles every read of either -- so report what was
    found and stop, rather than collecting power numbers from a half-deaf robot.
    """
    global _LAYOUT
    if _LAYOUT is not None:
        return _LAYOUT

    seen: dict[str, set[int]] = {}
    for label, port in (("arms", ARMS_PORT), ("head", HEAD_PORT)):
        probe = FeetechMotorsBus(
            port=port, motors={"probe": Motor(1, "sts3215", MotorNormMode.DEGREES)})
        probe.connect(handshake=False)
        try:
            seen[label] = {i for i in range(1, 13)
                           if probe.ping(i, num_retry=3) is not None}
        finally:
            probe.disconnect(disable_torque=False)

    def refuse(why: str) -> None:
        found = "\n".join(f"      {lb:4s} ({_port_of(lb)}) answers {sorted(ids)}"
                           for lb, ids in seen.items())
        raise SystemExit(
            f"\n  Cannot map the motors: {why}.\n{found}\n\n"
            "  Expected either layout:\n"
            "      arms = left 1-6 + right 7-12   head = neck 1-2 + wheels 3-5\n"
            "      arms = left 1-6                head = neck 1-2 + wheels 3-5 + right 7-12\n")

    left = next((lb for lb, ids in seen.items() if 6 in ids), None)
    if left is None:
        refuse("no bus answers ID 6, so the left arm is unreachable")
    right = next((lb for lb, ids in seen.items() if set(range(7, 13)) <= ids), None)
    if right is None:
        refuse("no bus answers all of 7-12, so the right arm is unreachable")
    base = next((lb for lb in seen if lb != left), left)
    if base == left:
        refuse("the left arm shares a bus with the neck and wheels, so IDs 3-5 collide")
    if not {1, 2, 3, 4, 5} <= seen[base]:
        missing = sorted({1, 2, 3, 4, 5} - seen[base])
        refuse(f"the {base} bus is missing neck/wheel IDs {missing}")

    motors: dict[str, dict] = {"arms": {}, "head": {}}
    motors[left] |= {f"left_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES)
                     for j, i in zip(JOINTS, range(1, 7))}
    motors[right] |= {f"right_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES)
                      for j, i in zip(JOINTS, range(7, 13))}
    motors[base] |= {"head_motor_1": Motor(1, "sts3215", MotorNormMode.DEGREES),
                     "head_motor_2": Motor(2, "sts3215", MotorNormMode.DEGREES),
                     "base_left_wheel": Motor(3, "sts3215", MotorNormMode.DEGREES),
                     "base_back_wheel": Motor(4, "sts3215", MotorNormMode.DEGREES),
                     "base_right_wheel": Motor(5, "sts3215", MotorNormMode.DEGREES)}

    total = sum(len(m) for m in motors.values())
    if total != 17:
        refuse(f"mapped {total} motors, expected 17")

    print("  bus layout (probed):")
    for lb in ("arms", "head"):
        print(f"      {lb:4s} ({_port_of(lb)}): {len(motors[lb]):2d} motors  "
              f"{', '.join(sorted(motors[lb]))}")

    _LAYOUT = {"motors": motors, "left": left, "right": right, "base": base}
    return _LAYOUT


def _port_of(label: str) -> str:
    return ARMS_PORT if label == "arms" else HEAD_PORT


def build_buses():
    motors = discover_layout()["motors"]
    return (("arms", FeetechMotorsBus(port=ARMS_PORT, motors=motors["arms"])),
            ("head", FeetechMotorsBus(port=HEAD_PORT, motors=motors["head"])))


def bus_voltages(buses, by_bus) -> dict[str, float]:
    """Lowest supply voltage any servo reports on each bus, in millivolts.

    The servos are the only voltmeter this robot has on the motor rail, and that
    rail is what the shared-supply question turns on: if enabling 17 actuators
    drags it below what the compute regulator needs, a daisy-chained design
    cannot hold up the Jetson -- which is the claim A1 exists to support.
    Measuring it here means the claim can be made without staging the crash.

    The minimum rather than the mean, deliberately: brownout is a worst-case
    question, and the servo furthest down the chain sees the most cable drop.
    """
    out: dict[str, float] = {}
    for lb, bs in buses:
        if not by_bus[lb]:
            continue
        try:
            v = bs.sync_read("Present_Voltage", by_bus[lb], normalize=False)
            out[lb] = min(int(x) for x in v.values()) * VOLT_UNIT_MV
        except Exception:
            out[lb] = float("nan")
    return out


def enable_order() -> list[tuple[str, str]]:
    """(bus, motor) in the protocol's order: neck, left arm, right arm, wheels."""
    motors = discover_layout()["motors"]
    where = {name: lb for lb, group in motors.items() for name in group}
    names = ["head_motor_1", "head_motor_2"]
    names += [f"left_{j}" for j in JOINTS]
    names += [f"right_{j}" for j in JOINTS]
    names += ["base_left_wheel", "base_back_wheel", "base_right_wheel"]
    return [(where[n], n) for n in names]


def boot_id() -> str:
    try:
        return BOOT_ID.read_text().strip()
    except OSError:
        return "unknown"


class RailSampler(threading.Thread):
    """Polls the INA3221 as fast as the kernel allows, flushing every sample.

    Flushing per sample is deliberate: the whole point of this test is to provoke
    a brownout, and buffered rows would be lost with the process.
    """

    def __init__(self, path: Path, stop: threading.Event):
        super().__init__(daemon=True)
        self.stop = stop
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(["elapsed_s", "timestamp", "millivolts", "milliamps",
                         "active_motors", "phase"])
        self.active = 0
        self.phase = "idle"
        self.n = 0
        self.t0 = time.monotonic()

    def run(self):
        while not self.stop.is_set():
            try:
                mv = int(RAIL_V.read_text())
                ma = int(RAIL_I.read_text())
            except (OSError, ValueError):
                continue
            self.w.writerow([f"{time.monotonic() - self.t0:.4f}",
                             datetime.now(timezone.utc).isoformat(),
                             mv, ma, self.active, self.phase])
            self.fh.flush()
            self.n += 1
        self.fh.close()


def release_gently(buses, names, seconds=TORQUE_RELEASE_SECONDS, steps=24):
    """Bleed torque away rather than cutting it, so nothing drops."""
    try:
        for i in range(steps - 1, -1, -1):
            limit = int(ARM_TORQUE_LIMIT * i / steps)
            for label, bus in buses:
                for name in names.get(label, []):
                    try:
                        bus.write("Torque_Limit", name, limit, num_retry=RETRY)
                    except Exception:
                        pass
            time.sleep(seconds / steps)
    finally:
        for label, bus in buses:
            for name in names.get(label, []):
                try:
                    bus.write("Torque_Limit", name, ARM_TORQUE_LIMIT, num_retry=RETRY)
                except Exception:
                    pass


def run_trial(trial: int, out_dir: Path, args) -> dict:
    """One enable sequence plus a worst-case simultaneous pose command."""
    stop = threading.Event()
    sampler = RailSampler(out_dir / f"trial_{trial:02d}_rail.csv", stop)

    buses = build_buses()
    for _, bus in buses:
        bus.connect()
    by_bus = {"arms": [], "head": []}
    bus_rows: list[dict] = []
    pose_rows: list[dict] = []
    boot_before = boot_id()

    sampler.start()
    time.sleep(1.0)  # a second of quiescent baseline before anything is enabled

    try:
        for idx, (label, motor) in enumerate(enable_order(), start=1):
            bus = dict(buses)[label]
            sampler.phase = f"enable_{motor}"
            bus.write("Torque_Limit", motor, ARM_TORQUE_LIMIT, num_retry=RETRY)
            bus.write("Acceleration", motor, ARM_ACCELERATION, num_retry=RETRY)

            # Park the goal on the CURRENT position before powering the joint.
            #
            # Enabling torque makes a servo drive to whatever Goal_Position still
            # sits in its register. After a power cycle that register reads 0, so a
            # joint resting at 3487 counts would slam ~307 deg into its hard stop at
            # full torque the instant torque is applied. Writing the present position
            # first makes the enable a no-op: the joint holds exactly where it is.
            # Wheels are excluded -- they run in velocity mode, where Goal_Position
            # is not the operative register.
            if motor not in WHEELS:
                here = bus.read("Present_Position", motor, normalize=False, num_retry=RETRY)
                bus.write("Goal_Position", motor, here, normalize=False, num_retry=RETRY)

            bus.enable_torque([motor], num_retry=RETRY)
            by_bus[label].append(motor)
            sampler.active = idx
            time.sleep(args.step_seconds)

            # Settled bus current at this active count, summed over the servos.
            per_bus, total_ma = {}, 0
            for lb, bs in buses:
                if not by_bus[lb]:
                    continue
                try:
                    cur = bs.sync_read("Present_Current", by_bus[lb], normalize=False)
                    amps = sum(abs(int(v)) for v in cur.values()) * CURRENT_UNIT_MA
                except Exception:
                    amps = float("nan")
                per_bus[lb] = amps
                if amps == amps:
                    total_ma += amps
            volts = bus_voltages(buses, by_bus)
            bus_rows.append(dict(active_motors=idx, motor=motor,
                                 arms_ma=round(per_bus.get("arms", 0.0), 1),
                                 head_ma=round(per_bus.get("head", 0.0), 1),
                                 total_ma=round(total_ma, 1),
                                 arms_mv=round(volts.get("arms", float("nan")), 1),
                                 head_mv=round(volts.get("head", float("nan")), 1)))

        # Worst-case: command every joint to the reference pose at once.
        sampler.phase = "simultaneous_pose"
        if args.pose_file.exists():
            import json
            targets = {k: {n: int(v) for n, v in d.items()}
                       for k, d in json.loads(args.pose_file.read_text()).items()}
            for label, bus in buses:
                sel = {n: v for n, v in targets.get(label, {}).items() if n in by_bus[label]}
                if sel:
                    bus.sync_write("Goal_Position", sel, normalize=False, num_retry=RETRY)
        # Sample motor current WHILE the pose is being driven.
        #
        # The per-step readings above are near zero by construction: goals are parked
        # on the present position, so each joint powers up with no position error and
        # commands no torque. This is the only window where the motors do real work --
        # holding both arms up against gravity -- so it is the only place a meaningful
        # bus current exists to measure. Sampled repeatedly because the draw peaks
        # during the move and decays as the arms settle.
        pose_t0 = time.monotonic()
        volts: dict[str, float] = {}
        while time.monotonic() - pose_t0 < 3.0:
            if len(pose_rows) % 5 == 0:
                volts = bus_voltages(buses, by_bus)
            per_bus, total_ma = {}, 0.0
            for lb, bs in buses:
                if not by_bus[lb]:
                    continue
                try:
                    cur = bs.sync_read("Present_Current", by_bus[lb], normalize=False)
                    amps = sum(abs(int(v)) for v in cur.values()) * CURRENT_UNIT_MA
                except Exception:
                    continue
                per_bus[lb] = amps
                total_ma += amps
            pose_rows.append(dict(elapsed_s=round(time.monotonic() - pose_t0, 3),
                                  phase="simultaneous_pose",
                                  arms_ma=round(per_bus.get("arms", 0.0), 1),
                                  head_ma=round(per_bus.get("head", 0.0), 1),
                                  total_ma=round(total_ma, 1),
                                  arms_mv=round(volts.get("arms", float("nan")), 1),
                                  head_mv=round(volts.get("head", float("nan")), 1)))

        sampler.phase = "settled"
        settle_t0 = time.monotonic()
        volts: dict[str, float] = {}
        while time.monotonic() - settle_t0 < 2.0:
            if len(pose_rows) % 5 == 0:
                volts = bus_voltages(buses, by_bus)
            per_bus, total_ma = {}, 0.0
            for lb, bs in buses:
                if not by_bus[lb]:
                    continue
                try:
                    cur = bs.sync_read("Present_Current", by_bus[lb], normalize=False)
                    amps = sum(abs(int(v)) for v in cur.values()) * CURRENT_UNIT_MA
                except Exception:
                    continue
                per_bus[lb] = amps
                total_ma += amps
            pose_rows.append(dict(elapsed_s=round(time.monotonic() - settle_t0, 3),
                                  phase="settled",
                                  arms_ma=round(per_bus.get("arms", 0.0), 1),
                                  head_ma=round(per_bus.get("head", 0.0), 1),
                                  total_ma=round(total_ma, 1),
                                  arms_mv=round(volts.get("arms", float("nan")), 1),
                                  head_mv=round(volts.get("head", float("nan")), 1)))
    finally:
        sampler.phase = "release"
        release_gently(buses, by_bus)
        for label, bus in buses:
            try:
                if by_bus[label]:
                    bus.disable_torque(by_bus[label], num_retry=RETRY)
            except Exception:
                pass
            try:
                bus.disconnect()
            except Exception:
                pass
        stop.set()
        sampler.join(timeout=5)

    if pose_rows:
        with (out_dir / f"trial_{trial:02d}_posecurrent.csv").open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(pose_rows[0].keys()))
            w.writeheader(); w.writerows(pose_rows)

    if bus_rows:
        with (out_dir / f"trial_{trial:02d}_buscurrent.csv").open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(bus_rows[0].keys()))
            w.writeheader(); w.writerows(bus_rows)

    boot_after = boot_id()
    rows = list(csv.DictReader(open(out_dir / f"trial_{trial:02d}_rail.csv")))
    mv = [int(r["millivolts"]) for r in rows] or [0]
    ma = [int(r["milliamps"]) for r in rows] or [0]

    bus_by_count = {int(r["active_motors"]): r["total_ma"] for r in bus_rows}
    per_count = {}
    for c in REPORT_COUNTS:
        sel = [int(r["millivolts"]) for r in rows if int(r["active_motors"]) == c]
        if sel:
            # (compute-rail V_min, motor-bus current) -- the two columns A1 asks for.
            per_count[c] = (min(sel), bus_by_count.get(c, float("nan")))

    pose_peak = max((r["total_ma"] for r in pose_rows), default=0.0)
    pose_arms = max((r["arms_ma"] for r in pose_rows), default=0.0)

    def rail(key: str, worst):
        """Idle/minimum motor-rail voltage on one bus, skipping failed reads."""
        vals = [r[key] for r in bus_rows + pose_rows
                if isinstance(r.get(key), (int, float)) and r[key] == r[key] and r[key] > 0]
        return worst(vals) if vals else 0.0

    return dict(trial=trial, config=args.config, samples=len(rows),
                pose_peak_ma=round(pose_peak, 1), pose_arms_peak_ma=round(pose_arms, 1),
                arms_rail_idle_mv=rail("arms_mv", max), arms_rail_min_mv=rail("arms_mv", min),
                head_rail_idle_mv=rail("head_mv", max), head_rail_min_mv=rail("head_mv", min),
                sample_hz=round(len(rows) / max(float(rows[-1]["elapsed_s"]), 1e-6), 1) if rows else 0,
                v_min_mv=min(mv), v_idle_mv=mv[0] if mv else 0, i_peak_ma=max(ma),
                reset="Y" if boot_before != boot_after else "N",
                per_count=per_count)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--config", required=True, choices=["baseline", "two-bus"],
                   help="Wiring configuration in use. Recorded, not detected -- set it honestly.")
    p.add_argument("--trials", type=int, default=10, help="Protocol asks for n >= 10.")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--step-seconds", type=float, default=STEP_SECONDS)
    p.add_argument("--pose-file", type=Path, default=Path("calibration/hold_pose.json"))
    args = p.parse_args()

    if not RAIL_V.exists():
        print(f"FATAL: {RAIL_V} missing -- INA3221 rail monitoring unavailable.")
        return 1

    args.out.mkdir(parents=True, exist_ok=True)
    print(f"\n  A1 brownout margin -- config={args.config}, {args.trials} trials")
    print(f"  Rail: {RAIL_V} (compute rail only; bus current comes from the bench supply)")
    print("  ARMS WILL MOVE at full motor count. Keep clear.")
    input("  Press ENTER to begin...")

    summary = []
    for t in range(1, args.trials + 1):
        print(f"\n  --- trial {t}/{args.trials} ---")
        r = run_trial(t, args.out, args)
        summary.append(r)
        print(f"    {r['samples']} samples @ {r['sample_hz']:.0f} Hz | "
              f"idle {r['v_idle_mv']} mV | V_min {r['v_min_mv']} mV | "
              f"I_peak {r['i_peak_ma']} mA | reset {r['reset']}")
        if t < args.trials:
            time.sleep(3.0)

    with open(args.out / "trials.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["trial", "config", "samples", "sample_hz", "v_idle_mv",
                    "v_min_mv", "i_peak_ma", "reset"])
        for r in summary:
            w.writerow([r["trial"], r["config"], r["samples"], r["sample_hz"],
                        r["v_idle_mv"], r["v_min_mv"], r["i_peak_ma"], r["reset"]])

    with open(args.out / "by_motor_count.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["active_motors", "config", "v_min_mv", "i_peak_ma", "trials"])
        for c in REPORT_COUNTS:
            vs = [r["per_count"][c][0] for r in summary if c in r["per_count"]]
            is_ = [r["per_count"][c][1] for r in summary if c in r["per_count"]]
            if vs:
                w.writerow([c, args.config, min(vs), max(is_), len(vs)])

    resets = sum(1 for r in summary if r["reset"] == "Y")
    vmin = min(r["v_min_mv"] for r in summary)
    print("\n" + "=" * 58)
    print(f"  config      {args.config}")
    print(f"  trials      {len(summary)}")
    print(f"  V_min       {vmin} mV  ({vmin/1000:.3f} V)")
    print(f"  resets      {resets}")
    print(f"  data        {args.out}")
    print("  NOTE: V_min is an upper bound -- the INA3221 averages internally and")
    print("        sampling is ~760 Hz, so sub-millisecond dips may be smoothed.")
    print("=" * 58)
    return 0


if __name__ == "__main__":
    sys.exit(main())
