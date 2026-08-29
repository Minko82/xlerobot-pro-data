#!/usr/bin/env python3
"""Measure the vision-to-gripper offset instead of guessing it.

``IK_TARGET_OFFSET`` in the reach script was inherited from an earlier build and is
a fudge factor: it absorbs everything the transform does not model -- where the
gripper's grasp point sits relative to the IK frame, camera mounting error, and any
bias in the depth reading.

Guessing it takes many attempts. Measuring it takes one:

    1. vision reports where it thinks the cube is, in the arm base frame
    2. you place the gripper ON the cube by hand
    3. forward kinematics says where the gripper actually is
    4. the offset is the difference

Torque stays OFF the whole time. Nothing moves under power.

    python scripts/measure_ik_offset.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from xlerobot_pro.color_detect import detect_object
from xlerobot_pro.config import ARMS_PORT, CAPTURE_DIR, HEAD_PORT
from xlerobot_pro.frame_transform import camera_xyz_to_base_xyz
from xlerobot_pro.ik import IK_SO101
from xlerobot_pro.realsense import capture

DEG2RAD = np.pi / 180.0
ARM_IDS = {"shoulder_pan": 7, "shoulder_lift": 8, "elbow_flex": 9,
           "wrist_flex": 10, "wrist_roll": 11, "gripper": 12}
IK_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
CALIB_PATH = (Path.home() / ".cache/huggingface/lerobot/calibration/robots"
              / "so101_follower" / "right_follower.json")


def motor_to_mjcf(q_deg: np.ndarray) -> np.ndarray:
    out = q_deg.copy()
    out[0] = -out[0]
    out[1] = 90.0 - out[1]
    out[2] = out[2] + 90.0
    return out


def ee_pose_in_base(ik: IK_SO101, q_mjcf_rad: np.ndarray):
    """Forward kinematics: gripper position AND orientation in the arm base frame."""
    pin.forwardKinematics(ik.model, ik.data, q_mjcf_rad)
    pin.updateFramePlacements(ik.model, ik.data)
    oMf = ik.data.oMf[ik.model.getFrameId(ik.EE_FRAME)]
    pos = ik._base_R.T @ (oMf.translation.copy() - ik._base_t)
    rot = ik._base_R.T @ oMf.rotation.copy()
    return pos, rot


def main() -> int:
    print("\n  === 1. where does vision think the cube is? ===")
    capture()
    centroid = detect_object(color="red", captures_dir=CAPTURE_DIR)

    head = FeetechMotorsBus(port=HEAD_PORT, motors={
        n: Motor(i, "sts3215", MotorNormMode.DEGREES)
        for n, i in (("head_motor_1", 1), ("head_motor_2", 2))})
    head.connect(handshake=False)
    try:
        raw = {n: head.read("Present_Position", n, normalize=False, num_retry=5)
               for n in ("head_motor_1", "head_motor_2")}
    finally:
        head.disconnect(disable_torque=False)
    pan = (raw["head_motor_2"] - 2048) * 0.0879
    tilt = (raw["head_motor_1"] - 2048) * 0.0879
    vision = np.array(camera_xyz_to_base_xyz(
        centroid[0], centroid[1], centroid[2],
        {"head_pan_joint": pan * DEG2RAD, "head_tilt_joint": tilt * DEG2RAD}))
    print(f"     vision  : [{vision[0]:+.4f}, {vision[1]:+.4f}, {vision[2]:+.4f}] m")

    print("\n  === 2. place the gripper on the cube ===")
    arms = FeetechMotorsBus(port=ARMS_PORT, motors={
        n: Motor(i, "sts3215",
                 MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES)
        for n, i in ARM_IDS.items()},
        calibration={k: MotorCalibration(**v) for k, v in json.loads(CALIB_PATH.read_text()).items()})
    arms.connect(handshake=False)
    try:
        arms.disable_torque()
        print("     Arm torque is OFF. Move it by hand so the gripper's grasp point")
        print("     sits exactly where it would be to pick up the cube.")
        print("     Hold it there and press ENTER (support the arm).")
        input("     > ")

        cur = arms.sync_read("Present_Position", IK_JOINTS)
        cur_deg = np.array([float(cur[j]) for j in IK_JOINTS])
        print(f"     motor deg: {', '.join(f'{v:.1f}' for v in cur_deg)}")
    finally:
        try:
            arms.disconnect(disable_torque=False)
        except Exception:
            pass

    print("\n  === 3. forward kinematics ===")
    ik = IK_SO101()
    ee_pos, ee_rot = ee_pose_in_base(ik, motor_to_mjcf(cur_deg) * DEG2RAD)
    print(f"     IK frame: [{ee_pos[0]:+.4f}, {ee_pos[1]:+.4f}, {ee_pos[2]:+.4f}] m")

    print("\n  === 4. the offset, in the GRIPPER's own frame ===")
    # The grasp point sits a fixed distance from the IK frame in the GRIPPER's
    # frame, not the base frame. Rotating the arm rotates that vector, so a constant
    # base-frame offset only holds at the pose it was measured at -- which is why two
    # measurements at different poses disagreed by 2 cm.
    delta_base = vision - ee_pos          # IK frame -> grasp point, base frame
    delta_local = ee_rot.T @ delta_base   # ...expressed in the gripper frame
    print(f"     base frame  : [{delta_base[0]:+.4f}, {delta_base[1]:+.4f}, {delta_base[2]:+.4f}]"
          f"   (pose-dependent, do NOT hardcode)")
    print(f"     GRIPPER frame: [{delta_local[0]:+.4f}, {delta_local[1]:+.4f}, {delta_local[2]:+.4f}]"
          f"   <-- this one is fixed")
    print(f"     magnitude {np.linalg.norm(delta_local) * 100:.1f} cm\n")
    print(f"     GRASP_OFFSET_LOCAL = np.array([{delta_local[0]:+.4f}, "
          f"{delta_local[1]:+.4f}, {delta_local[2]:+.4f}])\n")
    print("     Measure twice at different arm poses. The GRIPPER-frame numbers should")
    print("     now agree even though the base-frame ones do not -- that is the check")
    print("     that this is a real rigid offset.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
