#!/usr/bin/env python3
"""Compare the scene's lighting against what glassbottle_pick_v3 was recorded under.

Why this exists. The v3 demonstrations were side-lit by a neutral-white lamp at
the right edge of frame: that column reads 160.2 against a 99.2 shelf, a contrast
ratio of 1.62, and it held to sd 1.3 across all twenty takes. That is a tightly
controlled setup, and it is part of the training distribution just as much as the
bottle's position is.

Overall frame brightness is a poor check. Turning the room lights off while
daylight comes in raised the shelf to 115.6 and dropped the lamp column to 124.7
-- flat, warm light with a contrast ratio of 1.08 -- while the frame mean barely
moved. A directional light going flat moves every shadow in the scene, including
the bottle's shaded edge, which is where a wrist-camera policy reads its lateral
aim from.

So this measures the two regions separately, plus their ratio and the colour cast.

Run it with the arm AT THE START POSE -- the camera is wrist-mounted, so the
regions below only mean anything from that pose.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

#: Measured from the 20 start frames of glassbottle_pick_v3.
REF = {"lamp": (160.2, 1.3), "shelf": (99.2, 2.6), "ratio": (1.62, 0.03)}

#: Image regions, in pixels, valid only from the reference start pose.
LAMP = (slice(0, 300), slice(596, 640))    # the lamp column at the right edge
SHELF = (slice(60, 260), slice(200, 520))  # plain shelf: no lamp, no arm


def verdict(name: str, value: float, ref: tuple[float, float], tol: float) -> bool:
    mean, sd = ref
    ok = abs(value - mean) <= tol
    print("  %-22s %7.1f   reference %6.1f (sd %.1f)   %s"
          % (name, value, mean, sd, "OK" if ok else "OFF"))
    return ok


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--camera-serial", default="838212073725")
    p.add_argument("--frames", type=int, default=60,
                   help="Frames to discard before measuring. RealSense auto-exposure "
                        "needs ~60 (2 s) to converge; fewer reports the settling "
                        "transient rather than the room. Default 60.")
    p.add_argument("--tol", type=float, default=8.0,
                   help="Units of slack on lamp and shelf. Default 8.")
    args = p.parse_args()

    from lerobot.cameras.configs import ColorMode
    from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
    from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig

    cam = RealSenseCamera(RealSenseCameraConfig(
        serial_number_or_name=args.camera_serial, fps=30, width=640, height=480,
        color_mode=ColorMode.RGB))
    cam.connect()
    try:
        for _ in range(args.frames):
            frame = np.asarray(cam.read())
    finally:
        cam.disconnect()

    lamp = float(frame[LAMP].mean())
    shelf = float(frame[SHELF].mean())
    ratio = lamp / shelf if shelf else 0.0

    print("\n  arm must be at the reference start pose for these regions to mean anything\n")
    ok = verdict("lamp column", lamp, REF["lamp"], args.tol)
    ok &= verdict("shelf background", shelf, REF["shelf"], args.tol)
    ok &= verdict("contrast ratio", ratio, REF["ratio"], 0.15)

    r, g, b = (float(frame[LAMP][..., i].mean()) for i in range(3))
    warm = r - b
    print("  %-22s %.0f %.0f %.0f   reference 162 157 161" % ("lamp colour RGB", r, g, b))
    if warm > 12:
        print("     ^ warm cast (R-B = %+.0f). Daylight or incandescent, not the white lamp." % warm)
        ok = False

    print("\n  %s\n" % ("lighting matches the training set"
                        if ok else "lighting does NOT match -- trials will not be comparable"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
