#!/usr/bin/env python3
"""Drive the arm to the reference start pose, then leave it limp.

Why this exists. Every episode in a `--start-pose` dataset begins from the saved
pose, replayed over a ramp -- the measured start spread across the 20 takes of
glassbottle_pick_v3 is 0.3 units. The deployment runner
(`act_policy_control.py`) has no such step: it infers from wherever the arm
happens to be. A policy trained from one starting state and deployed from
another is out of distribution at step 1, and behaviour cloning compounds that
error rather than correcting it.

The trap is that the arm does NOT return to the pose on its own. The STS3215's
gearing holds position with torque disabled -- measured 1886 raw counts away on
`shoulder_lift` after a policy run, with torque off the whole time. "Rest pose"
names where the arm is put, not where it falls.

So run this before every trial, and the policy starts where it was taught to.

No camera is opened: this only moves motors, and RealSense init costs seconds
for nothing.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

sys.path.insert(0, str(Path(__file__).resolve().parent))
from record_kinesthetic import (  # noqa: E402
    BODY,
    START_POSE_SECONDS,
    harden_bus,
    load_start_pose,
    ramp_to_pose,
)
from xle_arms import ARM_IDS, SO101FollowerArm  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", default="/dev/xle_head")
    p.add_argument("--arm", choices=list(ARM_IDS), default="left")
    p.add_argument("--robot-id", default=None,
                   help="Calibration id. Defaults to <arm>_follower.")
    p.add_argument("--start-pose", type=Path,
                   default=Path("calibration/record_start_pose.json"),
                   help="Pose to replay. Refuses one captured under a different "
                        "calibration -- raw counts do not survive a recalibration.")
    p.add_argument("--seconds", type=float, default=START_POSE_SECONDS,
                   help=f"Ramp duration. Default {START_POSE_SECONDS:g}. Give it "
                        "longer when the arm starts far from the pose.")
    args = p.parse_args()

    if args.robot_id is None:
        args.robot_id = f"{args.arm}_follower"

    cfg = SO101FollowerConfig(port=args.port, id=args.robot_id, cameras={})
    robot = SO101FollowerArm(cfg, arm=args.arm)
    harden_bus(robot.bus)
    robot.connect(calibrate=False)
    try:
        target = load_start_pose(robot, args.arm, args.start_pose)

        before = {n: robot.bus.read("Present_Position", n, normalize=False) for n in BODY}
        worst = max(abs(before[n] - target[n]) for n in BODY)
        print(f"\n  {args.arm} arm, replaying {args.start_pose} over {args.seconds:g} s")
        print(f"  furthest joint is {worst} raw counts away -- KEEP CLEAR\n")
        for n in BODY:
            print("    %-16s %7d -> %7d  (%+d)" % (n, before[n], target[n], target[n] - before[n]))

        ramp_to_pose(robot, target, args.seconds)

        after = {n: robot.bus.read("Present_Position", n, normalize=False) for n in BODY}
        err = max(abs(after[n] - target[n]) for n in BODY)
        print(f"\n  at start pose, arm limp again (worst residual {err} counts)")
        # The residual is what the policy actually sees, so report it rather than
        # claiming success: a joint that could not reach the pose under gravity
        # is exactly the case this script exists to catch.
        if err > 50:
            print("  WARNING: residual is large. Check nothing is obstructing the arm.")
    finally:
        robot.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
