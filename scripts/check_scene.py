#!/usr/bin/env python3
"""Is the rig where glassbottle_pick_v3 was recorded? Checks geometry, then light.

Why this exists, and why geometry comes first.

The operator taped a boundary line on the table to mark the camera's field of
view. That tape sits at x = 602.3 px in all twenty demonstration start frames,
with a standard deviation of 0.2 px -- it is the best fiducial in the scene by a
wide margin. If it has moved, the table and the robot are no longer in the
relative position the policy was trained in, and the policy will drive to where
a bottle at that image position used to be rather than to where it now is.

Measured once at +33.1 px of drift, of which 5.7 px was the arm sitting 7 counts
off in shoulder_pan; the residual 27.4 px is 2.95 deg, or 2.47 units of
shoulder_pan. The policy's own learned pan bias is 0.9 units, so a drifted rig
outweighs it roughly threefold.

One thing this cannot tell you: whether the table moved or the robot did. A
rotated base moves the camera and the gripper together and largely cancels; a
moved table does not. Both look identical here. You will know which you moved.

Light is checked second because it turned out to be the smaller effect. The
demonstrations were lit so that the tape column reads ~160 against a ~99 shelf, a
ratio of 1.62 and colour-neutral. Frame mean does not capture this -- turning the
room lights off while daylight came in left the mean almost unchanged while the
ratio collapsed to 1.08 and the cast went warm.

Run this with the arm AT THE REFERENCE START POSE. The camera is wrist-mounted,
so every region below is meaningless from anywhere else.

And run it with `camera_preview.py` STOPPED. This opens the camera directly, which
is the point: a frame pulled from the preview's MJPEG stream has the alignment
overlay drawn into it, and the green target line sits at x=602 -- directly on top
of the tape. Measuring colour there reads the annotation as much as the scene. It
produced a tape colour cast swinging between +9 and +46 across four checks minutes
apart while the lamp had not moved at all.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

#: Measured across the 20 start frames of glassbottle_pick_v3.
TAPE_X = 602.3
TAPE_SD = 0.2
REF_LAMP = 160.2
REF_SHELF = 99.2
REF_RATIO = 1.62

TAPE_BAND = (slice(0, 300), slice(596, 640))   # the tape column, at the reference pose
SHELF = (slice(60, 260), slice(200, 520))      # plain shelf: no tape, no arm

#: Geometry, for reporting drift in units the operator can act on.
HFOV_DEG = 69.0                 # D435 colour horizontal field of view
WIDTH = 640
DEG_PER_PX = HFOV_DEG / WIDTH
PAN_COUNTS_PER_UNIT = (3660 - 948) / 200.0     # from the live calibration
DEG_PER_COUNT = 360.0 / 4096
DEG_PER_PAN_UNIT = PAN_COUNTS_PER_UNIT * DEG_PER_COUNT


def find_tape(frame: np.ndarray) -> float | None:
    """Brightness-weighted centroid of the rightmost bright vertical band.

    Returns None when no band stands out, which is what an off-frame tape looks
    like -- report that rather than a fabricated number.
    """
    v = frame[0:300, :].mean(axis=(0, 2)).astype(float)
    base = np.percentile(v, 50)
    thr = base + 0.55 * (v.max() - base)
    xs = np.arange(len(v))[v > thr]
    if len(xs) == 0:
        return None
    run = np.split(xs, np.where(np.diff(xs) > 3)[0] + 1)[-1]
    if len(run) < 3 or run[-1] >= WIDTH - 2:
        return None                             # clipped: the true centre is off-frame
    w = v[run] - thr
    return float((run * w).sum() / w.sum())


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--camera-serial", default="838212073725")
    p.add_argument("--frames", type=int, default=60,
                   help="Frames to discard before measuring. RealSense auto-exposure "
                        "needs ~60 (2 s); fewer reports the settling transient.")
    p.add_argument("--tape-tol", type=float, default=3.0,
                   help="Pixels of slack on tape alignment. Default 3 (~0.22 units "
                        "of shoulder_pan). The recording held 0.2.")
    p.add_argument("--light-tol", type=float, default=8.0)
    args = p.parse_args()

    from lerobot.cameras.configs import ColorMode
    from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
    from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig

    cam = RealSenseCamera(RealSenseCameraConfig(
        serial_number_or_name=args.camera_serial, fps=30, width=WIDTH, height=480,
        color_mode=ColorMode.RGB))
    cam.connect()
    try:
        for _ in range(args.frames):
            frame = np.asarray(cam.read())
    finally:
        cam.disconnect()

    print("\n  the arm must be at the reference start pose, or none of this means anything\n")
    ok = True

    print("  GEOMETRY")
    x = find_tape(frame)
    if x is None:
        print("    tape band            NOT FOUND — off frame, or washed out by glare.")
        print("      The rig has moved far enough that the fiducial has left the view.")
        ok = False
    else:
        d = x - TAPE_X
        print("    tape band centre     %7.1f px   reference %.1f (sd %.1f)   %s"
              % (x, TAPE_X, TAPE_SD, "OK" if abs(d) <= args.tape_tol else "OFF"))
        if abs(d) > args.tape_tol:
            print("      drift %+.1f px = %+.2f deg = %+.2f units of shoulder_pan"
                  % (d, d * DEG_PER_PX, d * DEG_PER_PX / DEG_PER_PAN_UNIT))
            print("      tape reads right of reference — the policy will aim right of the"
                  " bottle" if d > 0 else
                  "      tape reads left of reference — the policy will aim left of the bottle")
            ok = False

    print("\n  LIGHT")
    tape_v = float(frame[TAPE_BAND].mean())
    shelf = float(frame[SHELF].mean())
    ratio = tape_v / shelf if shelf else 0.0
    for name, val, ref, tol in [("tape column", tape_v, REF_LAMP, args.light_tol),
                                ("shelf background", shelf, REF_SHELF, args.light_tol),
                                ("contrast ratio", ratio, REF_RATIO, 0.15)]:
        good = abs(val - ref) <= tol
        ok &= good
        print("    %-20s %7.1f      reference %6.1f            %s"
              % (name, val, ref, "OK" if good else "OFF"))
    r, g, b = (float(frame[TAPE_BAND][..., i].mean()) for i in range(3))
    if r - b > 12:
        print("    colour cast          R-B %+.0f      warm — daylight or incandescent" % (r - b))
        ok = False

    print("\n  %s\n" % ("scene matches the recording" if ok else
                        "scene does NOT match — fix geometry first, it is the larger error"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
