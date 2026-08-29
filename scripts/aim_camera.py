#!/usr/bin/env python3
"""Aim the neck camera at the workspace and save a preview frame.

The neck has two actuated joints on the head bus. Rather than nudging the camera
by hand and hoping, this drives them in encoder counts and captures what the
camera sees afterwards, so aiming is a closed loop you can iterate on.

    python scripts/aim_camera.py                 # just show where it is and grab a frame
    python scripts/aim_camera.py --tilt -300     # look further down, then grab a frame
    python scripts/aim_camera.py --pan 150 --tilt -200

Positive tilt raises the view, negative lowers it; sign of pan depends on the
build, so try 100 and look. 1 count = 0.0879 deg, so 300 counts is about 26 deg.

Preview is written to /tmp/aim.jpg. Torque is left ENABLED on exit so the neck
holds its aim -- releasing it would let the camera sag under its own weight and
undo the alignment you just set.
"""

from __future__ import annotations

import argparse
import sys

import cv2

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from xlerobot_pro.config import HEAD_PORT
from xlerobot_pro.firmware_limits import WHEEL_NECK_ACCELERATION, WHEEL_NECK_TORQUE_LIMIT

NECK = {"head_motor_1": 1, "head_motor_2": 2}

#: Refuse to move further than this in one go. A neck slewing 60 deg unexpectedly
#: can hit the camera on the frame.
MAX_STEP = 600


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--pan", type=int, default=0, help="head_motor_1 delta, encoder counts.")
    p.add_argument("--tilt", type=int, default=0, help="head_motor_2 delta. Negative looks down.")
    p.add_argument("--camera-index", type=int, default=4)
    p.add_argument("--out", default="/tmp/aim.jpg")
    args = p.parse_args()

    if abs(args.pan) > MAX_STEP or abs(args.tilt) > MAX_STEP:
        print(f"  Step capped at {MAX_STEP} counts (~{MAX_STEP * 0.0879:.0f} deg). "
              "Move in stages.", file=sys.stderr)
        return 1

    bus = FeetechMotorsBus(
        port=HEAD_PORT,
        motors={n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for n, i in NECK.items()},
    )
    bus.connect(handshake=False)
    try:
        for n in NECK:
            bus.write("Torque_Limit", n, WHEEL_NECK_TORQUE_LIMIT, num_retry=5)
            bus.write("Acceleration", n, WHEEL_NECK_ACCELERATION, num_retry=5)

        pos = {n: bus.read("Present_Position", n, normalize=False, num_retry=5) for n in NECK}
        print(f"  current   pan {pos['head_motor_1']}   tilt {pos['head_motor_2']}")

        if args.pan or args.tilt:
            bus.enable_torque(list(NECK), num_retry=5)
            target = {
                "head_motor_1": pos["head_motor_1"] + args.pan,
                "head_motor_2": pos["head_motor_2"] + args.tilt,
            }
            # Interpolate: a single write makes the neck snap, which is jarring and
            # risks banging the camera into the frame.
            import time
            steps = 40
            for i in range(1, steps + 1):
                f = i / steps
                bus.sync_write("Goal_Position",
                               {n: int(round(pos[n] + (target[n] - pos[n]) * f)) for n in NECK},
                               normalize=False, num_retry=5)
                time.sleep(0.03)
            time.sleep(0.6)
            now = {n: bus.read("Present_Position", n, normalize=False, num_retry=5) for n in NECK}
            print(f"  moved to  pan {now['head_motor_1']}   tilt {now['head_motor_2']}")

        # If camera_stream.py is running it owns the device, and that is the better
        # way to aim anyway -- a live view beats a snapshot. Say so instead of
        # reporting it as a failure.
        import subprocess
        streaming = subprocess.run(["pgrep", "-f", "[c]amera_stream"],
                                   capture_output=True).returncode == 0
        if streaming:
            print("  live stream is running -- watch http://10.0.0.197:8080 "
                  "(no snapshot taken)")
            return 0

        cap = cv2.VideoCapture(args.camera_index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ok = False
        for _ in range(15):
            ok, frame = cap.read()
        cap.release()
        if not ok:
            print(f"  could not read /dev/video{args.camera_index} -- is something else "
                  "holding it?", file=sys.stderr)
            return 1
        cv2.imwrite(args.out, frame)
        print(f"  preview   {args.out}")
    finally:
        # Deliberately NOT releasing torque: the neck must hold the aim.
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
