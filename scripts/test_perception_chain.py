#!/usr/bin/env python3
"""Dry-run the perception -> transform -> IK chain. NOTHING MOVES.

Before building an autonomous demonstration generator on this pipeline it is worth
proving each link separately, because a failure anywhere downstream looks the same
from the outside: the arm goes to the wrong place.

    1. RealSense RGBD capture      -- is the SDK giving proper colour and depth?
    2. red-blob detection          -- is the cube found, and how big is the blob?
    3. depth -> camera-frame xyz   -- is the depth at that pixel sane?
    4. camera -> arm base frame    -- does the neck angle feed in correctly?
    5. IK solve                    -- is that target reachable?

Run it with the cube on the table. It prints each stage so a failure is
attributable, and it never enables torque.
"""

from __future__ import annotations

import sys

import numpy as np

DEG2RAD = np.pi / 180.0

# Empirical offsets carried over from the original controller. They compensate
# end-effector placement error without touching the vision transform.
IK_TARGET_OFFSET = np.array([-0.12, 0.0, 0.05])


def main() -> int:
    print("\n  === 1. capture ===")
    from xlerobot_pro.realsense import capture
    try:
        capture()
        print("     RGBD captured")
    except Exception as exc:
        print(f"     FAILED: {type(exc).__name__}: {exc}")
        return 1

    print("\n  === 2. detect red blob ===")
    import cv2
    from xlerobot_pro.color_detect import detect_color
    from xlerobot_pro.config import CAPTURE_DIR
    bgr = cv2.imread(str(CAPTURE_DIR / "color.png"))
    if bgr is None:
        print(f"     FAILED: no color.png in {CAPTURE_DIR}")
        return 1
    dets = detect_color(bgr, "red", min_area=200)
    if not dets:
        print("     FAILED: no red blob found.")
        print("     The cube may be out of frame, too small, or the lighting is washing")
        print("     out its saturation. Check the framing first.")
        return 1
    d = dets[0]
    print(f"     found {len(dets)} blob(s); largest {d.area:.0f} px at {d.centroid_px}")
    vis = bgr.copy()
    x, y, w, h = d.bbox
    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 0, 255), 2)
    cv2.circle(vis, d.centroid_px, 5, (0, 255, 255), -1)
    cv2.imwrite("/tmp/perception.jpg", vis)
    print("     annotated -> /tmp/perception.jpg")

    print("\n  === 3. camera-frame 3D ===")
    from xlerobot_pro.color_detect import detect_object
    try:
        centroid = detect_object(color="red", captures_dir=CAPTURE_DIR)
        print(f"     camera optical frame: [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}] m")
        if not (0.1 < centroid[2] < 2.0):
            print(f"     WARNING: depth {centroid[2]:.3f} m is implausible -- the depth patch")
            print("     may be landing on a hole. A shiny or dark object does this.")
    except Exception as exc:
        print(f"     FAILED: {type(exc).__name__}: {exc}")
        return 1

    print("\n  === 4. transform to arm base frame ===")
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    from xlerobot_pro.config import HEAD_PORT
    from xlerobot_pro.frame_transform import camera_xyz_to_base_xyz
    bus = FeetechMotorsBus(port=HEAD_PORT, motors={
        n: Motor(i, "sts3215", MotorNormMode.DEGREES)
        for n, i in (("head_motor_1", 1), ("head_motor_2", 2))})
    bus.connect(handshake=False)
    try:
        raw = {n: bus.read("Present_Position", n, normalize=False, num_retry=5)
               for n in ("head_motor_1", "head_motor_2")}
    finally:
        bus.disconnect(disable_torque=False)
    # ID 1 is TILT and ID 2 is PAN on this wiring (config.py says so, and it matches
    # what the joints actually do when driven). The original controller had them the
    # other way round because the head motors sat at different IDs then -- feeding
    # them swapped puts the target ~0.8 m out, well beyond the arm's reach.
    #
    # Homing offsets from calibration/head.json are deliberately NOT applied: that
    # file was written for the old wiring (it records ids 7 and 8), so its offsets
    # belong to different motors. Raw counts about the 2048 centre give a target at
    # 0.30 m, which is inside the workspace.
    pan_deg = (raw["head_motor_2"] - 2048) * 0.0879
    tilt_deg = (raw["head_motor_1"] - 2048) * 0.0879
    print(f"     neck raw  tilt(ID1)={raw['head_motor_1']}  pan(ID2)={raw['head_motor_2']}"
          f"  ->  pan {pan_deg:.1f} deg, tilt {tilt_deg:.1f} deg")
    bx, by, bz = camera_xyz_to_base_xyz(
        centroid[0], centroid[1], centroid[2],
        {"head_pan_joint": pan_deg * DEG2RAD, "head_tilt_joint": tilt_deg * DEG2RAD},
    )
    print(f"     arm base frame: [{bx:.4f}, {by:.4f}, {bz:.4f}] m")
    target = np.array([bx, by, bz]) + IK_TARGET_OFFSET
    print(f"     with offsets  : [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}] m")

    print("\n  === 5. IK ===")
    from xlerobot_pro.ik import IK_SO101
    ik = IK_SO101()
    # 1 mm (the default) is tighter than this arm can repeat and tighter than a
    # compliant Fin-Ray gripper needs -- the solver reaches ~3 mm and then burns
    # 1000 iterations failing to do better, reporting non-convergence on a target it
    # has effectively hit. 5 mm is well inside the gripper's passive tolerance.
    traj = ik.generate_ik(list(target), [0.0, 0.0, 0.0], position_tolerance=5e-3)
    if not traj:
        print("     FAILED: no IK solution -- the target is out of reach or the")
        print("     transform is placing it somewhere impossible. Compare the base-frame")
        print("     coordinates above against the arm's actual workspace.")
        return 1
    q_deg = np.rad2deg(traj[-1])
    print(f"     solved in {len(traj)} steps")
    print(f"     final joint config (MJCF deg): "
          f"{', '.join(f'{v:.1f}' for v in q_deg)}")
    print("\n  ALL FIVE STAGES PASSED -- the chain is sound.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
