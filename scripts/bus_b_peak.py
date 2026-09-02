#!/usr/bin/env python3
"""Tier 0 #1: drive the arm through a worst-case simultaneous multi-axis move while an
inline ammeter on the 12 V arm-bus feed records the peak.

What it does. With the Table III envelope applied (tau 450, accel 40, vmax 100),
every joint of the chosen arm(s) is commanded at the same instant from the start
pose to an "out" pose (shoulder lifted, elbow extended, wrist flexed, pan swung,
jaws closing) and back, N times. That is the highest simultaneous torque demand
the deployed policy can ask for: all six servos accelerating against gravity at
once at the deployed cap. The meter's peak-hold gives Table IV's P2 "Measured".

Alongside, for cross-reference only (uncalibrated register units, ~20 Hz):
Present_Current of every servo, summed per bus, and the Jetson's own VDD_IN via
INA3221 (which is NOT the motor bus). Both are written to CSV.

Keep the object and everything else out of the arm's reach. The move stays
inside each joint's calibrated range and is expressed in normalised units, so
it is the same physical motion on any calibration.

    python scripts/bus_b_peak.py --port /dev/xle_head --arms left --reps 5 --out A1/bus_b_peak
    # from a Mac with the bus board on USB (no Jetson needed):
    python scripts/bus_b_peak.py --port /dev/tty.usbmodemXXXX --arms left --reps 5 --out A1/bus_b_peak_left
    python scripts/bus_b_peak.py --port /dev/xle_head --arms left,right --reps 5 --out A1/bus_b_peak_both
"""
from __future__ import annotations
import argparse, csv, sys, threading, time
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xle_arms import ARM_IDS, JOINTS, SO101FollowerArm  # noqa: E402

# Self-contained copies of the runner's bus hardening and envelope, so this runs from a
# laptop with only lerobot[feetech] installed (no RealSense, no xlerobot_pro package).
ARM_TORQUE_LIMIT, ARM_ACCELERATION, ARM_MAX_VELOCITY, TORQUE_RELEASE_SECONDS = 450, 40, 100, 8.0


def harden_bus(bus, retries=6, backoff=0.01):
    def wrap(fn):
        def retrying(*a, **kw):
            for attempt in range(retries):
                try:
                    return fn(*a, **kw)
                except ConnectionError:
                    if attempt == retries - 1:
                        raise
                    time.sleep(backoff * (2 ** attempt))
        return retrying
    for name in ("sync_read", "sync_write", "read", "write"):
        if hasattr(bus, name):
            setattr(bus, name, wrap(getattr(bus, name)))


def apply_envelope(robot):
    names = list(robot.bus.motors)
    robot.bus.disable_torque(names)
    for n in names:
        robot.bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT)
        robot.bus.write("Acceleration", n, ARM_ACCELERATION)
        robot.bus.write("Maximum_Velocity_Limit", n, ARM_MAX_VELOCITY)
    robot.bus.enable_torque(names)


def release_gently(robot, seconds=TORQUE_RELEASE_SECONDS, steps=24):
    names = list(robot.bus.motors)
    try:
        for i in range(steps - 1, -1, -1):
            for n in names:
                try: robot.bus.write("Torque_Limit", n, int(ARM_TORQUE_LIMIT * i / steps))
                except Exception: pass
            time.sleep(seconds / steps)
    finally:
        for n in names:
            try: robot.bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT)
            except Exception: pass

#: Normalised targets. "home" is the recorded start pose; "out" is a large simultaneous
#: excursion that stays well inside every calibrated range and above the shelf.
HOME = {"shoulder_pan": 0.0, "shoulder_lift": -100.0, "elbow_flex": 99.0, "wrist_flex": 75.0, "wrist_roll": 39.0, "gripper": 90.0}
OUT = {"shoulder_pan": 25.0, "shoulder_lift": -20.0, "elbow_flex": 30.0, "wrist_flex": -30.0, "wrist_roll": 39.0, "gripper": 20.0}

RAIL_V = Path("/sys/class/hwmon/hwmon1/in1_input")
RAIL_I = Path("/sys/class/hwmon/hwmon1/curr1_input")


def read_rail():
    try:
        return int(RAIL_V.read_text()), int(RAIL_I.read_text())
    except Exception:
        return None, None


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", default="/dev/xle_head")
    p.add_argument("--arms", default="left", help="left, right, or left,right")
    p.add_argument("--reps", type=int, default=5)
    p.add_argument("--dwell", type=float, default=1.5, help="Seconds at each end of the move.")
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    arms = a.arms.split(",")
    a.out.mkdir(parents=True, exist_ok=True)

    robots = {}
    for arm in arms:
        r = SO101FollowerArm(SO101FollowerConfig(port=a.port, id=f"{arm}_follower", cameras={}), arm=arm)
        harden_bus(r.bus); r.connect(calibrate=False); apply_envelope(r)
        robots[arm] = r
    # One sampler thread: servo currents on every arm + the compute rail.
    rows = []; stop = threading.Event()

    def sample():
        t0 = time.perf_counter()
        while not stop.is_set():
            t = time.perf_counter() - t0
            cur = {}
            for arm, r in robots.items():
                try:
                    c = r.bus.sync_read("Present_Current", normalize=False)
                    cur.update({f"{arm}.{k}": int(v) for k, v in c.items()})
                except Exception:
                    pass
            v, i = read_rail()
            rows.append((round(t, 4), v, i, sum(cur.values()), cur))
            time.sleep(0.03)

    th = threading.Thread(target=sample, daemon=True); th.start()
    print(f"\n  {', '.join(arms)} arm(s) on {a.port}; envelope applied; {a.reps} simultaneous out-and-back moves.")
    print("  METER: enable peak/max hold now.  Moving in 3 s -- keep clear.")
    time.sleep(3)
    t_start = time.perf_counter(); marks = []
    try:
        for arm, r in robots.items():
            r.send_action({f"{j}.pos": HOME[j] for j in JOINTS})
        time.sleep(3.0)
        for k in range(a.reps):
            marks.append(("out", round(time.perf_counter() - t_start, 3)))
            for r in robots.values():
                r.send_action({f"{j}.pos": OUT[j] for j in JOINTS})
            time.sleep(a.dwell)
            marks.append(("home", round(time.perf_counter() - t_start, 3)))
            for r in robots.values():
                r.send_action({f"{j}.pos": HOME[j] for j in JOINTS})
            time.sleep(a.dwell)
            print(f"    rep {k + 1}/{a.reps}")
    finally:
        stop.set(); th.join(timeout=1)
        for r in robots.values():
            try: release_gently(r)
            except Exception: pass
            try: r.disconnect()
            except Exception: pass

    keys = sorted({k for _, _, _, _, c in rows for k in c})
    with open(a.out / "servo_current.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_s", "vdd_in_mV", "vdd_in_mA", "servo_current_sum_raw"] + keys)
        for t, v, i, s, c in rows:
            w.writerow([t, v, i, s] + [c.get(k, "") for k in keys])
    with open(a.out / "moves.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["event", "t_s"]); w.writerows(marks)
    peak = max((s for _, _, _, s, _ in rows), default=0)
    print(f"\n  {len(rows)} samples; servo register sum peak {peak} (raw units, cross-reference only)")
    print(f"  WRITE THE METER'S PEAK INTO {a.out}/meter.txt as 'peak_A=<value> meter=FNIRSI-2C53T range=<range>'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
