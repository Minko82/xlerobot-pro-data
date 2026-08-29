#!/usr/bin/env python3
"""Pick up a coloured cube using vision, IK, and visual servoing.

Why servoing rather than a calibrated offset. The grasp point sits a fixed distance
from the IK frame in the GRIPPER's frame, so a constant base-frame offset is only
correct at the pose it was measured at -- that is why the first reach landed to one
side. A model estimate of that offset (~6.7 cm from the jaw meshes) and a
hand-measured one (~14 cm) disagree by a factor of two, and neither can be trusted
blind.

So the arm corrects itself instead:

    1. solve for a PRE-GRASP pose above the cube
    2. move there, then look again -- from above, the gripper does not occlude the
       cube, so the residual lateral error is measurable
    3. shift by that error and re-approach; repeat
    4. descend, close, lift

Errors are measured in the same base frame the IK works in, so the loop closes on
the quantity that actually matters and never needs the offset to be right -- only
roughly right enough to get the cube in view from above.

Safety, since this can run unattended:
    * targets outside a plausible workspace box are refused, not attempted
    * every motion is interpolated and speed-limited
    * torque is bled off gradually on every exit path including Ctrl-C
    * repeated detection failures abort rather than reaching blindly
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from xlerobot_pro.color_detect import detect_object
from xlerobot_pro.config import ARMS_PORT, CAPTURE_DIR, HEAD_PORT
from xlerobot_pro.firmware_limits import ARM_ACCELERATION, ARM_TORQUE_LIMIT, TORQUE_RELEASE_SECONDS
from xlerobot_pro.frame_transform import camera_xyz_to_base_xyz
from xlerobot_pro.ik import IK_SO101
from xlerobot_pro.realsense import capture

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi

ARM_IDS = {"shoulder_pan": 7, "shoulder_lift": 8, "elbow_flex": 9,
           "wrist_flex": 10, "wrist_roll": 11, "gripper": 12}
IK_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CALIB_PATH = (Path.home() / ".cache/huggingface/lerobot/calibration/robots"
              / "so101_follower" / "right_follower.json")

#: Grasp point relative to the IK frame, in the GRIPPER's frame. Derived from the
#: jaw meshes: the fingers run to y = -0.106 m and overlap between -0.082 and
#: -0.052, so the object sits near y = -0.067. Only a starting estimate -- the
#: servo loop corrects whatever residual it has.
GRASP_OFFSET_LOCAL = np.array([0.0, -0.067, 0.0])

WORKSPACE = {"x": (-0.45, 0.45), "y": (-0.45, 0.45), "z": (-0.25, 0.55)}
MAX_REACH = 0.50

APPROACH_HEIGHT = 0.10     # metres above the cube for the pre-grasp
LIFT_HEIGHT = 0.12         # how far to lift after closing
GRIPPER_OPEN = 100.0
GRIPPER_CLOSED = 8.0
IK_PASSES = 4


def load_calibration() -> dict:
    if not CALIB_PATH.exists():
        raise SystemExit(f"No calibration at {CALIB_PATH}")
    return {n: MotorCalibration(**v) for n, v in json.loads(CALIB_PATH.read_text()).items()}


def motor_to_mjcf(q: np.ndarray) -> np.ndarray:
    o = q.copy(); o[0] = -o[0]; o[1] = 90.0 - o[1]; o[2] = o[2] + 90.0
    return o


def mjcf_to_motor(q: np.ndarray) -> np.ndarray:
    o = q.copy(); o[0] = -o[0]; o[1] = 90.0 - o[1]; o[2] = o[2] - 90.0
    return o


def ee_pose(ik, q_rad):
    pin.forwardKinematics(ik.model, ik.data, q_rad)
    pin.updateFramePlacements(ik.model, ik.data)
    m = ik.data.oMf[ik.model.getFrameId(ik.EE_FRAME)]
    return (ik._base_R.T @ (m.translation - ik._base_t), ik._base_R.T @ m.rotation)


def solve_grasp(ik, goal_base, seed_rad):
    """IK placing the GRASP POINT at goal_base, by fixed-point iteration."""
    aim = np.asarray(goal_base, float).copy()
    traj, err = None, np.inf
    for _ in range(IK_PASSES):
        t = ik.generate_ik(list(aim), [0.0, 0.0, 0.0], position_tolerance=5e-3,
                           seed_q_rad=seed_rad)
        if not t:
            return None, np.inf
        traj = t
        p, R = ee_pose(ik, traj[-1])
        residual = np.asarray(goal_base) - (p + R @ GRASP_OFFSET_LOCAL)
        err = float(np.linalg.norm(residual))
        if err < 3e-3:
            break
        aim = aim + residual
        seed_rad = traj[-1]
    return traj, err


def in_workspace(p) -> bool:
    return (all(WORKSPACE[a][0] <= v <= WORKSPACE[a][1] for a, v in zip("xyz", p))
            and np.linalg.norm(p) <= MAX_REACH)


def neck_angles():
    b = FeetechMotorsBus(port=HEAD_PORT, motors={
        n: Motor(i, "sts3215", MotorNormMode.DEGREES)
        for n, i in (("head_motor_1", 1), ("head_motor_2", 2))})
    b.connect(handshake=False)
    try:
        raw = {n: b.read("Present_Position", n, normalize=False, num_retry=5)
               for n in ("head_motor_1", "head_motor_2")}
    finally:
        b.disconnect(disable_torque=False)
    return (raw["head_motor_2"] - 2048) * 0.0879, (raw["head_motor_1"] - 2048) * 0.0879


def snapshot(dest: str, label: str) -> None:
    """Save what the camera sees right now, while the arm still holds its pose."""
    try:
        import shutil
        capture()
        shutil.copy(CAPTURE_DIR / "color.png", dest)
        print(f"     photo ({label}) -> {dest}")
    except Exception as exc:
        print(f"     [warn] photo failed: {type(exc).__name__}")


def see_cube(colour: str):
    """Cube centroid in the arm base frame, or None."""
    try:
        capture()
        c = detect_object(color=colour, captures_dir=CAPTURE_DIR)
    except Exception:
        return None
    pan, tilt = neck_angles()
    return np.array(camera_xyz_to_base_xyz(
        c[0], c[1], c[2],
        {"head_pan_joint": pan * DEG2RAD, "head_tilt_joint": tilt * DEG2RAD}))


def glide(bus, cur_deg, goal_deg, seconds, steps, stopping, wrist_roll):
    for k in range(1, steps + 1):
        if stopping["f"]:
            return False
        f = k / steps
        f = f * f * (3.0 - 2.0 * f)
        q = cur_deg + (goal_deg - cur_deg) * f
        g = {j: float(q[i]) for i, j in enumerate(IK_JOINTS)}
        g["wrist_roll"] = wrist_roll
        bus.sync_write("Goal_Position", g, num_retry=2)
        time.sleep(seconds / steps)
    return True


def set_gripper(bus, value, seconds=1.0, steps=20):
    cur = float(bus.read("Present_Position", "gripper", num_retry=3))
    for k in range(1, steps + 1):
        bus.sync_write("Goal_Position", {"gripper": cur + (value - cur) * k / steps}, num_retry=2)
        time.sleep(seconds / steps)


def release_gently(bus, names, seconds=TORQUE_RELEASE_SECONDS, steps=24):
    try:
        for i in range(steps - 1, -1, -1):
            lim = int(ARM_TORQUE_LIMIT * i / steps)
            for n in names:
                try:
                    bus.write("Torque_Limit", n, lim, num_retry=3)
                except Exception:
                    pass
            time.sleep(seconds / steps)
    finally:
        for n in names:
            try:
                bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT, num_retry=3)
            except Exception:
                pass


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--colour", default="red")
    p.add_argument("--servo-passes", type=int, default=3)
    p.add_argument("--stop-after", choices=["approach", "descend", "grasp", "lift"],
                   default="lift", help="Halt after this stage. Build up gradually.")
    p.add_argument("--move-seconds", type=float, default=4.0)
    p.add_argument("--steps", type=int, default=200)
    p.add_argument("--photo", default="/tmp/grasp_state.jpg",
                   help="Capture the scene at the stop point, BEFORE torque is released. "
                        "Photographing after the script exits shows the arm already lowered, "
                        "which says nothing about where it actually got to.")
    args = p.parse_args()

    cube = see_cube(args.colour)
    if cube is None:
        print("  no cube detected."); return 1
    print(f"\n  cube at [{cube[0]:+.3f}, {cube[1]:+.3f}, {cube[2]:+.3f}]  "
          f"|d|={np.linalg.norm(cube):.3f} m")
    if not in_workspace(cube):
        print("  REFUSING: cube outside the plausible workspace."); return 1

    ik = IK_SO101()
    arms = FeetechMotorsBus(port=ARMS_PORT, motors={
        n: Motor(i, "sts3215",
                 MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES)
        for n, i in ARM_IDS.items()}, calibration=load_calibration())
    arms.connect(handshake=False)
    names = list(ARM_IDS)
    stopping = {"f": False}
    signal.signal(signal.SIGINT, lambda *_: stopping.__setitem__("f", True))

    try:
        for n in names:
            arms.write("Operating_Mode", n, OperatingMode.POSITION.value, num_retry=3)
            arms.write("Torque_Limit", n, ARM_TORQUE_LIMIT, num_retry=3)
            arms.write("Acceleration", n, ARM_ACCELERATION, num_retry=3)
        arms.enable_torque(names, num_retry=3)
        set_gripper(arms, GRIPPER_OPEN, seconds=0.8)

        cur = arms.sync_read("Present_Position", IK_JOINTS)
        cur_deg = np.array([float(cur[j]) for j in IK_JOINTS])
        wrist_roll = float(cur_deg[4])

        # ---- approach, with visual correction -----------------------------
        goal = cube + np.array([0.0, 0.0, APPROACH_HEIGHT])
        for attempt in range(args.servo_passes):
            if stopping["f"]:
                break
            traj, err = solve_grasp(ik, goal, motor_to_mjcf(cur_deg) * DEG2RAD)
            if traj is None or err > 0.02:
                print(f"  IK failed for approach (residual {err * 1000:.0f} mm)."); return 1
            tgt = mjcf_to_motor(traj[-1] * RAD2DEG)
            print(f"  approach pass {attempt + 1}: moving to "
                  f"[{goal[0]:+.3f}, {goal[1]:+.3f}, {goal[2]:+.3f}]")
            if not glide(arms, cur_deg, tgt, args.move_seconds, args.steps, stopping, wrist_roll):
                break
            time.sleep(0.8)
            cur = arms.sync_read("Present_Position", IK_JOINTS)
            cur_deg = np.array([float(cur[j]) for j in IK_JOINTS])

            seen = see_cube(args.colour)
            if seen is None:
                print("     lost sight of the cube -- keeping the current aim")
                break
            p_ee, R_ee = ee_pose(ik, motor_to_mjcf(cur_deg) * DEG2RAD)
            grasp_now = p_ee + R_ee @ GRASP_OFFSET_LOCAL
            lateral = (seen - grasp_now)[:2]
            print(f"     cube now [{seen[0]:+.3f}, {seen[1]:+.3f}]  gripper "
                  f"[{grasp_now[0]:+.3f}, {grasp_now[1]:+.3f}]  "
                  f"lateral error {np.linalg.norm(lateral) * 1000:.0f} mm")
            if np.linalg.norm(lateral) < 0.01:
                print("     within 10 mm -- good enough to descend")
                cube = seen
                break
            cube = seen
            goal = seen + np.array([0.0, 0.0, APPROACH_HEIGHT])
        if args.stop_after == "approach" or stopping["f"]:
            snapshot(args.photo, "approach")
            print("\n  stopping after approach."); return 0

        # ---- descend ------------------------------------------------------
        traj, err = solve_grasp(ik, cube, motor_to_mjcf(cur_deg) * DEG2RAD)
        if traj is None or err > 0.02:
            print(f"  IK failed for descent ({err * 1000:.0f} mm)."); return 1
        print("  descending to the cube")
        glide(arms, cur_deg, mjcf_to_motor(traj[-1] * RAD2DEG), 2.5, 120, stopping, wrist_roll)
        time.sleep(0.6)
        cur = arms.sync_read("Present_Position", IK_JOINTS)
        cur_deg = np.array([float(cur[j]) for j in IK_JOINTS])
        if args.stop_after == "descend" or stopping["f"]:
            snapshot(args.photo, "descent")
            print("\n  stopping after descent."); return 0

        # ---- grasp --------------------------------------------------------
        print("  closing gripper")
        set_gripper(arms, GRIPPER_CLOSED, seconds=1.5)
        load = arms.read("Present_Load", "gripper", num_retry=3)
        print(f"     gripper load {load}"
              + ("  (holding something)" if load > 40 else "  (NOTHING GRIPPED)"))
        if args.stop_after == "grasp" or stopping["f"]:
            snapshot(args.photo, "grasp")
            print("\n  stopping after grasp."); return 0

        # ---- lift ---------------------------------------------------------
        lift = cube + np.array([0.0, 0.0, LIFT_HEIGHT])
        traj, err = solve_grasp(ik, lift, motor_to_mjcf(cur_deg) * DEG2RAD)
        if traj is None or err > 0.03:
            print("  IK failed for lift."); return 1
        print("  lifting")
        glide(arms, cur_deg, mjcf_to_motor(traj[-1] * RAD2DEG), 2.5, 120, stopping, wrist_roll)
        time.sleep(1.0)
        print(f"     gripper load after lift: {arms.read('Present_Load', 'gripper', num_retry=3)}")
        snapshot(args.photo, "lift")
    finally:
        print(f"\n  lowering over {TORQUE_RELEASE_SECONDS:g}s...")
        try:
            release_gently(arms, names)
        except Exception:
            pass
        for fn in (lambda: arms.disable_torque(num_retry=3),
                   lambda: arms.disconnect(disable_torque=False)):
            try:
                fn()
            except Exception:
                pass
        print("  torque released.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
