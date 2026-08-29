#!/usr/bin/env python3
"""Reach the right arm to a detected red cube, then stop. SUPERVISED TEST.

The first real motion driven by the vision pipeline. It does the smallest thing
that proves the chain end to end -- capture, detect, transform, solve, move -- and
deliberately does not grasp, lift, or record. If the transform is wrong, the
failure shows up as the arm going to the wrong place with nothing in its hand.

Guards, because a bad transform commands a real arm:

  * the base-frame target must lie inside a plausible workspace box, or it refuses
    to move at all
  * motion is interpolated at 0.01 s per waypoint rather than jumping to the
    solution
  * torque is bled off gradually on every exit path, including Ctrl-C

Run it with the cube on the table and a hand near the power switch.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import numpy as np

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

#: Right arm. The vision transform and IK base are calibrated to this arm.
ARM_IDS = {"shoulder_pan": 7, "shoulder_lift": 8, "elbow_flex": 9,
           "wrist_flex": 10, "wrist_roll": 11, "gripper": 12}
IK_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

#: Offset from the IK frame to the gripper's grasp point, in the GRIPPER's own
#: frame. Set it from measure_ik_offset.py. A base-frame offset cannot work: this
#: vector rotates with the arm, so a constant applied in the base frame is only
#: correct at the pose it was measured at.
GRASP_OFFSET_LOCAL = np.array([0.0, 0.0, 0.0])

#: Solve, check where the grasp point actually lands, correct, repeat. Three passes
#: converge to a few mm; a single open-loop solve cannot, because the offset depends
#: on the very orientation the solve produces.
IK_REFINE_PASSES = 4

#: Plausible workspace in the arm base frame, metres. A target outside this is a
#: transform failure, not a reachable pose -- refuse rather than drive into it.
WORKSPACE = {"x": (-0.45, 0.45), "y": (-0.45, 0.45), "z": (-0.25, 0.55)}
MAX_REACH = 0.50

GRIPPER_OPEN = 100.0

#: Written by `record_kinesthetic.py --arm right --calibrate`.
CALIB_PATH = (Path.home() / ".cache/huggingface/lerobot/calibration/robots"
              / "so101_follower" / "right_follower.json")


def load_calibration() -> dict:
    """Degrees are meaningless without homing offsets, so this is not optional."""
    if not CALIB_PATH.exists():
        raise SystemExit(
            f"No calibration at {CALIB_PATH}.\n"
            "Run: python scripts/record_kinesthetic.py --repo-id local/tmp "
            "--arm right --calibrate")
    raw = json.loads(CALIB_PATH.read_text())
    return {name: MotorCalibration(**vals) for name, vals in raw.items()}


def motor_to_mjcf(q_deg: np.ndarray) -> np.ndarray:
    out = q_deg.copy()
    out[0] = -out[0]
    out[1] = 90.0 - out[1]
    out[2] = out[2] + 90.0
    return out


def mjcf_to_motor(q_deg: np.ndarray) -> np.ndarray:
    out = q_deg.copy()
    out[0] = -out[0]
    out[1] = 90.0 - out[1]
    out[2] = out[2] - 90.0
    return out


def ee_pose_in_base(ik, q_mjcf_rad):
    """Gripper position and orientation in the arm base frame."""
    import pinocchio as pin
    pin.forwardKinematics(ik.model, ik.data, q_mjcf_rad)
    pin.updateFramePlacements(ik.model, ik.data)
    oMf = ik.data.oMf[ik.model.getFrameId(ik.EE_FRAME)]
    return (ik._base_R.T @ (oMf.translation.copy() - ik._base_t),
            ik._base_R.T @ oMf.rotation.copy())


def solve_for_grasp(ik, cube_base, seed_rad, verbose=True):
    """IK such that the GRASP POINT lands on the cube, not the IK frame.

    Fixed-point iteration: aim the IK frame at the cube, see where the grasp point
    ends up, shift the aim by the error, repeat.
    """
    aim = np.asarray(cube_base, dtype=float).copy()
    traj = None
    for i in range(IK_REFINE_PASSES):
        traj = ik.generate_ik(list(aim), [0.0, 0.0, 0.0],
                              position_tolerance=5e-3, seed_q_rad=seed_rad)
        if not traj:
            return None, None
        ee_pos, ee_rot = ee_pose_in_base(ik, traj[-1])
        grasp = ee_pos + ee_rot @ GRASP_OFFSET_LOCAL
        err = np.asarray(cube_base) - grasp
        if verbose:
            print(f"     pass {i + 1}: grasp point [{grasp[0]:+.3f}, {grasp[1]:+.3f}, "
                  f"{grasp[2]:+.3f}]  err {np.linalg.norm(err) * 1000:5.1f} mm")
        if np.linalg.norm(err) < 3e-3:
            break
        aim = aim + err
        seed_rad = traj[-1]
    return traj, float(np.linalg.norm(err))


def release_gently(bus, names, seconds: float = TORQUE_RELEASE_SECONDS, steps: int = 24) -> None:
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


def neck_angles() -> tuple[float, float]:
    """(pan_deg, tilt_deg). ID 1 is tilt, ID 2 is pan on this wiring."""
    bus = FeetechMotorsBus(port=HEAD_PORT, motors={
        n: Motor(i, "sts3215", MotorNormMode.DEGREES)
        for n, i in (("head_motor_1", 1), ("head_motor_2", 2))})
    bus.connect(handshake=False)
    try:
        raw = {n: bus.read("Present_Position", n, normalize=False, num_retry=5)
               for n in ("head_motor_1", "head_motor_2")}
    finally:
        bus.disconnect(disable_torque=False)
    return (raw["head_motor_2"] - 2048) * 0.0879, (raw["head_motor_1"] - 2048) * 0.0879


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--colour", default="red")
    p.add_argument("--seconds", type=float, default=4.0, help="Duration of the move.")
    p.add_argument("--steps", type=int, default=200, help="Interpolation steps.")
    p.add_argument("--dry-run", action="store_true", help="Solve and report, move nothing.")
    args = p.parse_args()

    print("\n  === perceive ===")
    capture()
    centroid = detect_object(color=args.colour, captures_dir=CAPTURE_DIR)
    pan_deg, tilt_deg = neck_angles()
    bx, by, bz = camera_xyz_to_base_xyz(
        centroid[0], centroid[1], centroid[2],
        {"head_pan_joint": pan_deg * DEG2RAD, "head_tilt_joint": tilt_deg * DEG2RAD})
    target = np.array([bx, by, bz])
    dist = float(np.linalg.norm(target))
    print(f"     cube in base frame : [{bx:.3f}, {by:.3f}, {bz:.3f}]  |d|={dist:.3f} m")

    bad = [ax for ax, v in zip("xyz", target)
           if not (WORKSPACE[ax][0] <= v <= WORKSPACE[ax][1])]
    if bad or dist > MAX_REACH:
        print(f"\n  REFUSING TO MOVE: target outside the plausible workspace "
              f"({'axis ' + ','.join(bad) if bad else f'reach {dist:.2f} m > {MAX_REACH}'}).")
        print("  That is a transform failure, not a hard-to-reach pose.")
        return 1

    print("\n  === solve ===")
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
        cur = arms.sync_read("Present_Position", IK_JOINTS)
        cur_deg = np.array([float(cur[j]) for j in IK_JOINTS])
        print(f"     current motor deg  : {', '.join(f'{v:.1f}' for v in cur_deg)}")
        seed = motor_to_mjcf(cur_deg) * DEG2RAD
        traj, resid = solve_for_grasp(ik, target, seed)
        if not traj:
            print("     IK found no solution.")
            return 1
        if resid > 0.02:
            print(f"\n  REFUSING: grasp point still {resid * 1000:.0f} mm from the cube after "
                  f"{IK_REFINE_PASSES} passes.")
            print("  Either GRASP_OFFSET_LOCAL is unset/wrong, or the target is at the edge")
            print("  of the workspace where the arm cannot reach the required orientation.")
            return 1
        final = mjcf_to_motor(traj[-1] * RAD2DEG)
        print(f"     solved             : {len(traj)} solver iterations")
        print(f"     final motor deg    : {', '.join(f'{v:.1f}' for v in final)}")
        swing = np.abs(final - cur_deg)
        print(f"     joint travel       : {', '.join(f'{v:.0f}' for v in swing)} deg"
              f"   (largest {swing.max():.0f})")

        # Interpolate from the CURRENT pose to the solution rather than replaying the
        # solver's iterates. Those are a search path, not a motion plan -- here it
        # converged in 4 iterations across a 164 deg swing, which as waypoints would
        # command the arm to slam across its workspace in 40 ms.
        def ease(f: float) -> float:
            return f * f * (3.0 - 2.0 * f)

        goals = []
        for k in range(1, args.steps + 1):
            f = ease(k / args.steps)
            q = cur_deg + (final - cur_deg) * f
            goals.append({j: float(q[i]) for i, j in enumerate(IK_JOINTS)})
        print(f"     interpolated       : {len(goals)} steps over {args.seconds:g}s")

        if args.dry_run:
            print("\n  dry run -- nothing moved.")
            return 0

        print("\n  === move ===")
        print("  THE ARM WILL MOVE. Ctrl-C stops it.")
        input("  ENTER when clear... ")
        for n in names:
            arms.write("Operating_Mode", n, OperatingMode.POSITION.value, num_retry=3)
            arms.write("Torque_Limit", n, ARM_TORQUE_LIMIT, num_retry=3)
            arms.write("Acceleration", n, ARM_ACCELERATION, num_retry=3)
        arms.enable_torque(names, num_retry=3)
        arms.sync_write("Goal_Position", {"gripper": GRIPPER_OPEN}, num_retry=3)

        wrist_roll = float(cur_deg[4])   # pin it; the solver leaves it unconstrained
        for g in goals:
            if stopping["f"]:
                print("\n  interrupted")
                break
            g = dict(g)
            g["wrist_roll"] = wrist_roll
            arms.sync_write("Goal_Position", g, num_retry=2)
            time.sleep(args.seconds / args.steps)
        time.sleep(1.0)

        reached = arms.sync_read("Present_Position", IK_JOINTS)
        err = [abs(float(reached[j]) - goals[-1][j]) for j in IK_JOINTS[:3]]
        print(f"     reached            : {', '.join(f'{float(reached[j]):.1f}' for j in IK_JOINTS)}")
        print(f"     tracking error     : {', '.join(f'{e:.1f}' for e in err)} deg (first 3 joints)")
        print("\n  Look at the arm: is the gripper at the cube?")
    finally:
        print(f"\n  lowering over {TORQUE_RELEASE_SECONDS:g}s...")
        try:
            release_gently(arms, names)
        except Exception as exc:
            print(f"    [warn] {type(exc).__name__}")
        try:
            arms.disable_torque(num_retry=3)
        except Exception:
            pass
        try:
            arms.disconnect(disable_torque=False)
        except Exception:
            pass
        print("  torque released.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
