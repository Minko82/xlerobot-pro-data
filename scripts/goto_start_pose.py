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

**The gripper is part of the start pose even though the file does not contain
it.** `record_start_pose.json` holds five body joints; the jaws are wherever the
last run left them. After a successful grasp that is closed, around 13 units,
and the next trial then approaches the bottle with the jaws already shut and
knocks it over instead of enclosing it -- observed directly, a trial beginning at
22.7 against a training frame 0 of 94.4. So this opens them too, by default.

No camera is opened: this only moves motors, and RealSense init costs seconds
for nothing.
"""

from __future__ import annotations

import argparse
import sys
import time
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

#: Mean gripper position at frame 0 across the twenty glassbottle_pick_v3
#: episodes. Normalised (the gripper is RANGE_0_100, not RANGE_M100_100).
GRIPPER_START = 94.4


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
    p.add_argument("--gripper", type=float, default=GRIPPER_START,
                   help=f"Normalised position to open the jaws to. Default "
                        f"{GRIPPER_START:g}, the mean of frame 0 across the twenty "
                        f"glassbottle_pick_v3 episodes. Use --no-gripper to leave "
                        f"them alone.")
    p.add_argument("--no-gripper", action="store_true",
                   help="Leave the jaws where they are. They will still be holding "
                        "whatever the last run grasped.")
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

        if not args.no_gripper:
            g0 = robot.bus.read("Present_Position", "gripper")
            # Ramped rather than written once: the jaws may be clamped on the
            # object from the previous run, and snapping them open throws it.
            robot.bus.enable_torque(["gripper"])
            try:
                for i in range(1, 21):
                    robot.bus.write("Goal_Position", "gripper",
                                    g0 + (args.gripper - g0) * i / 20)
                    time.sleep(0.05)
            finally:
                robot.bus.disable_torque(["gripper"])
            g1 = robot.bus.read("Present_Position", "gripper")
            print("    %-16s %7.1f -> %7.1f  (target %.1f)" % ("gripper", g0, g1, args.gripper))
            if abs(g1 - args.gripper) > 5:
                print("  WARNING: jaws did not reach the target. Something may be "
                      "between them.")

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
