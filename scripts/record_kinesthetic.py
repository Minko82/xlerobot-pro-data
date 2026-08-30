#!/usr/bin/env python3
"""Record demonstrations by moving the robot's own arm by hand.

A leader arm is only a way of *generating* follower trajectories. If you pose the
follower directly, you get the same thing without the second arm, the second bus,
or its failure modes. The dataset this writes is a standard LeRobotDataset and
trains with stock ``lerobot-train``.

How it works. Torque is released on every joint, gripper included, so you move and
squeeze the arm entirely by hand with nothing to type mid-motion. The STS3215's gear
reduction gives enough back-drive friction that jaws squeezed onto a light object
usually hold it. If yours slips, --powered-gripper keeps the gripper driven and
adds keyboard open/close instead.

The action for frame *t* is the state at frame *t+1*. In leader-follower recording
the action is the leader's pose, which the follower has not reached yet, so action
leads state by roughly one frame; shifting by one reproduces that relationship.
Using the state itself would make the mapping the identity and teach nothing.

Because it drives ``SO101Follower``, the observation and action spaces are byte-
identical to what the deployment script sees. Calibrate first:

    lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/xle_arms \\
        --robot.id=left_follower

Then:

    python scripts/record_kinesthetic.py --repo-id local/bottle_pickplace --episodes 50

During an episode: press ENTER to end and keep the take, or type d then ENTER to
throw it away. With --powered-gripper, c and o close and open the jaws.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import threading
import time
from pathlib import Path

import numpy as np

from lerobot.cameras.configs import ColorMode
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts, hw_to_dataset_features
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from xle_arms import ARM_IDS, SO101FollowerArm

#: If the neck moves during a session the camera view changes, and the policy is
#: learning pixels -> joint commands. A shifted view mid-dataset means the same
#: scene produces different images, which is the one inconsistency that quietly
#: ruins a recording. Warn above this many counts (~4 degrees).
NECK_DRIFT_WARN = 45


def neck_position(arm_port: str = "") -> dict | None:
    """Current neck angles, or None if the neck cannot be read safely.

    Two reasons this returns None rather than a number.

    The neck lives on the head bus at IDs 1-2, which are also the arm's
    shoulder_pan and shoulder_lift. If the arm is running on the head port --
    as it is whenever the arms adapter is unavailable -- those IDs answer as
    the ARM, and the drift check silently reports shoulder movement between
    takes as camera drift. A wrong warning is worse than no warning: it invites
    you to throw away a good dataset.

    It is also unsafe. This opens a SECOND handle on a port the robot already
    holds, once per saved episode. Two handles interleaving on one half-duplex
    bus corrupts traffic, which shows up later as `sync_read` or `Lock` failing
    mid-session and killing the run.
    """
    if arm_port and Path(arm_port).resolve() == Path("/dev/xle_head").resolve():
        return None
    try:
        b = FeetechMotorsBus(port="/dev/xle_head", motors={
            n: Motor(i, "sts3215", MotorNormMode.RANGE_M100_100)
            for n, i in (("head_motor_1", 1), ("head_motor_2", 2))})
        b.connect(handshake=False)
        pos = {n: b.read("Present_Position", n, normalize=False, num_retry=3)
               for n in ("head_motor_1", "head_motor_2")}
        b.disconnect(disable_torque=False)
        return pos
    except Exception:
        return None

#: Joints released for hand posing. Every joint including the gripper, so the whole
#: demonstration is done by hand with nothing to type mid-motion. The STS3215's gear
#: reduction gives enough back-drive friction that jaws squeezed onto a light object
#: usually hold it; if yours slips, rerun with --powered-gripper.
BODY = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIPPER = "gripper"

#: Only used with --powered-gripper. The gripper normalises 0-100 across the
#: jaw travel found by calibration (2038-3252 counts on this arm), so 45 is not
#: "open" -- it is 45% of the way, a little under half of what the jaws can do.
GRIP_OPEN = 45.0
GRIP_CLOSED = 2.0

#: How wide to open when handing the object back between takes. Deliberately
#: separate from GRIP_OPEN and deliberately much wider: this is commanded after
#: the capture loop has ended, so it never lands in a demonstration and costs
#: nothing to make generous. Widening GRIP_OPEN instead would change what the
#: policy is taught, and would make takes recorded before and after the change
#: disagree about what "open" means.
#:
#: Short of 100 so the servo settles just off its mechanical stop instead of
#: pushing into it for the whole 30-60 s video encode.
GRIP_RELEASE = 95.0

#: Attempts before giving up on the bus. lerobot's own calls default to no retry,
#: so ONE dropped packet raises straight out of whatever is running -- losing the
#: take in progress, or minutes of calibration sweeping, and abandoning the rest.
#:
#: The backoff doubles from 10 ms, so six attempts span ~310 ms. That is sized
#: from observation: three attempts at a flat 10 ms still lost calibration on
#: this arm, which means the dropouts outlast 30 ms.
#:
#: The cost of a wide window is worth stating plainly. On a healthy bus it is
#: never paid. On a failing one, a stall means frames are not captured while the
#: arm keeps moving, and lerobot timestamps frames by index -- so the trajectory
#: replays as if that stretch happened faster than it did. Losing a few frames
#: beats losing the episode, but neither is as good as fixing the connection;
#: see scripts/bus_watch.py.
BUS_RETRIES = 6
BUS_BACKOFF_S = 0.01


def harden_bus(bus, retries: int = BUS_RETRIES):
    """Make every read and write on this bus survive a dropped packet.

    Retrying inside the recording loop is not enough, because the loop is not
    the only thing that talks to the bus. ``robot.calibrate()`` spends minutes
    in lerobot's ``record_ranges_of_motion()``, which runs its own ``sync_read``
    at the default ``num_retry=0`` -- so one dropped packet ends calibration
    after you have already done the physical work of sweeping every joint.
    ``connect()`` -> ``configure()`` is equally exposed and fails as
    ``Failed to write 'Lock' on id_=5``.

    Wrapping the bound methods on the instance covers all of them at once,
    including lerobot's internal ``self.sync_read(...)`` calls, because an
    instance attribute shadows the class method.

    This is mitigation, not a repair. A bus that drops packets while the arm is
    being moved by hand has a marginal connection somewhere, and retries only
    buy enough reliability to finish a session.
    """
    def wrap(fn):
        def retrying(*a, **kw):
            for attempt in range(retries):
                try:
                    return fn(*a, **kw)
                except ConnectionError:
                    if attempt == retries - 1:
                        raise
                    time.sleep(BUS_BACKOFF_S * (2 ** attempt))
        return retrying

    for name in ("sync_read", "sync_write", "read", "write"):
        if hasattr(bus, name):
            setattr(bus, name, wrap(getattr(bus, name)))


def observe(robot, grip: float | None):
    """One observation. Retries happen inside the bus -- see harden_bus."""
    if grip is not None:
        robot.bus.sync_write("Goal_Position", {GRIPPER: grip})
    return robot.get_observation()


class Keys:
    """Non-blocking line reader. stdin is shared with the prompts, so a thread."""

    def __init__(self):
        self.pending: list[str] = []
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        for line in sys.stdin:
            if self.stop.is_set():
                return
            # Keep empty lines: a bare ENTER is the "end this episode" signal.
            self.pending.append(line.strip().lower())

    def take(self) -> str | None:
        return self.pending.pop(0) if self.pending else None

    def drain(self) -> None:
        self.pending.clear()

    def wait(self) -> str:
        """Block until a line arrives. Never call input() alongside this thread."""
        while True:
            k = self.take()
            if k is not None:
                return k
            time.sleep(0.03)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--repo-id", required=True, help="e.g. local/bottle_pickplace")
    p.add_argument("--root", type=Path, default=None, help="Where to write. Default ~/.cache/huggingface/lerobot")
    p.add_argument("--port", default="/dev/xle_arms")
    p.add_argument("--arm", choices=list(ARM_IDS), default="left",
                   help="Which arm to record. left = motor IDs 1-6, right = 7-12.")
    p.add_argument("--robot-id", default=None,
                   help="Calibration id. Defaults to <arm>_follower. Must match what you "
                        "calibrated -- the two arms need different ids or they overwrite "
                        "each other.")
    p.add_argument("--calibrate", action="store_true",
                   help="Run the interactive range-finding calibration for this arm, then exit.")
    p.add_argument("--camera-serial", default="838212073725",
                   help="RealSense serial. Raw V4L2 (/dev/videoN) bypasses the RealSense "
                        "colour pipeline and yields a heavy green cast, which is useless for "
                        "colour-based detection and wrong for paper figures.")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--episodes", type=int, default=50)
    p.add_argument("--task", default="pick up the bottle and place it on the other side")
    p.add_argument("--max-seconds", type=float, default=45.0, help="Hard cap per episode.")
    p.add_argument("--resume", action="store_true",
                   help="Append to an existing dataset instead of failing. Use this after a "
                        "Ctrl-C to keep adding episodes to the same recording session.")
    p.add_argument("--grip-open", type=float, default=GRIP_OPEN, metavar="0-100",
                   help=f"How far 'o' opens the jaws during a take (default {GRIP_OPEN:.0f}). "
                        "This IS recorded, so changing it mid-dataset makes earlier and later "
                        "episodes disagree about what open means.")
    p.add_argument("--no-release", action="store_true",
                   help="Do NOT open the jaws when a take ends. The arm is limp, so an "
                        "automatic release drops whatever is held from wherever it was left "
                        "-- fine for a pill bottle, not for glass. With this set, take the "
                        "object out by hand, or press o before you end the take.")
    p.add_argument("--grip-release", type=float, default=GRIP_RELEASE, metavar="0-100",
                   help=f"How far the jaws open to hand the object back between takes "
                        f"(default {GRIP_RELEASE:.0f}). Commanded after the take ends, so it is "
                        "never recorded -- widen it freely.")
    p.add_argument("--powered-gripper", action="store_true",
                   help="Keep the gripper powered and drive it from the keyboard (c/o) instead "
                        "of squeezing it by hand. Use if a limp gripper will not hold the object.")
    args = p.parse_args()
    if args.robot_id is None:
        args.robot_id = f"{args.arm}_follower"

    cfg = SO101FollowerConfig(
        port=args.port,
        id=args.robot_id,
        cameras={"top": RealSenseCameraConfig(serial_number_or_name=args.camera_serial,
                                              fps=args.fps, width=args.width,
                                              height=args.height, color_mode=ColorMode.RGB)},
    )
    robot = SO101FollowerArm(cfg, arm=args.arm)
    harden_bus(robot.bus)
    robot.connect(calibrate=False)
    print(f"  Arm    : {args.arm} (motor IDs {ARM_IDS[args.arm][0]}-{ARM_IDS[args.arm][-1]})"
          f"  id={args.robot_id}")

    if args.calibrate:
        print("\n  Calibrating. Follow the prompts, then rerun without --calibrate.\n")
        robot.calibrate()
        robot.disconnect()
        return 0

    if not robot.is_calibrated:
        print(f"\n  The {args.arm} arm is not calibrated as so101_follower/{args.robot_id}.\n"
              f"  Run this first:\n"
              f"    python scripts/record_kinesthetic.py --repo-id {args.repo_id} "
              f"--arm {args.arm} --calibrate\n", file=sys.stderr)
        robot.disconnect()
        return 1

    ds_features = combine_feature_dicts(
        hw_to_dataset_features(robot.action_features, "action", True),
        hw_to_dataset_features(robot.observation_features, "observation", True),
    )

    root = args.root
    if args.resume:
        dataset = LeRobotDataset(args.repo_id, root=root)
        dataset.start_image_writer(num_processes=0, num_threads=4)
        print(f"\n  Dataset: {dataset.root}  (resuming, "
              f"{dataset.meta.total_episodes} episodes already recorded)")
    else:
        try:
            dataset = LeRobotDataset.create(
                args.repo_id, args.fps, root=root, robot_type=robot.name,
                features=ds_features, use_videos=True,
                image_writer_processes=0, image_writer_threads=4,
            )
        except FileExistsError as exc:
            print(f"\n  A dataset already exists at {exc.filename}\n"
                  "  Either add --resume to append to it, pick a different --repo-id,\n"
                  "  or delete it if it was a throwaway test.\n", file=sys.stderr)
            robot.disconnect()
            return 1
        print(f"\n  Dataset: {dataset.root}")
    print(f"  Task   : {args.task}")

    neck0 = neck_position(args.port)
    if neck0 is None:
        print("  Camera : drift guard OFF -- the neck is not readable on a separate bus.\n"
              "           Check by hand that the neck is rigid before you start.")
    if neck0:
        print(f"  Camera : neck at {neck0['head_motor_1']}, {neck0['head_motor_2']} "
              "-- do not re-aim it after this point")

    keys = Keys()
    grip = args.grip_open
    period = 1.0 / args.fps
    kept = dataset.meta.total_episodes if args.resume else 0

    try:
        if args.powered_gripper:
            robot.bus.disable_torque(BODY)
            print("\n  Arm is limp; gripper is powered. Type c to close, o to open.")
        else:
            robot.bus.disable_torque()
            print("\n  Whole arm is limp, gripper included -- move and squeeze it by hand.")

        while kept < args.episodes:
            print(f"\n  ---- episode {kept + 1} / {args.episodes} ----")
            print("  Put the bottle at the start mark, arm at rest.")
            if args.powered_gripper:
                print("  ENTER to start.  During: c=close  o=open  ENTER=keep  d=discard  q=quit")
            else:
                print("  ENTER to start, do the task, then ENTER again to keep it.")
                print("  (d then ENTER throws the take away; q then ENTER quits)")
            keys.drain()
            start_cmd = keys.wait()
            if start_cmd in ("q", "quit"):
                break

            keys.drain()          # so the start keystroke cannot end the episode

            # Every take must begin from the same commanded jaw width. The
            # between-takes release leaves `grip` at GRIP_RELEASE, and without
            # this reset the first episode would start at --grip-open while
            # every later one started at --grip-release -- a gripper channel
            # that disagrees with itself across the dataset, for no reason the
            # policy can learn from.
            if args.powered_gripper:
                grip = args.grip_open
                robot.bus.sync_write("Goal_Position", {GRIPPER: grip})

            print("\n  \u25cf RECORDING  --  press ENTER when the task is done\n")
            states, frames, t0 = [], [], time.perf_counter()
            verdict = None
            bus_lost = False
            last_draw = 0.0
            while verdict is None:
                loop_t = time.perf_counter()
                k = keys.take()
                if args.powered_gripper and k in ("c", "close"):
                    grip = GRIP_CLOSED
                    print("    gripper closing")
                elif args.powered_gripper and k in ("o", "open"):
                    grip = args.grip_open
                    print("    gripper opening")
                elif k in ("e", "end", ""):
                    verdict = "keep"
                elif k in ("d", "discard", "r"):
                    verdict = "discard"

                try:
                    obs = observe(robot, grip if args.powered_gripper else None)
                except ConnectionError as exc:
                    print(f"\n    \u26a0 bus dropped out and did not come back after "
                          f"{BUS_RETRIES} tries:\n      {exc}")
                    bus_lost = True
                    verdict = "keep" if len(frames) > args.fps else "discard"
                    break
                states.append({k2: v for k2, v in obs.items() if k2.endswith(".pos")})
                # Copy the image arrays out of librealsense's buffers before keeping
                # them. get_observation() hands back views into the camera's frame
                # pool, which is 16 deep; retaining them un-copied starves the
                # pipeline and it stops delivering after ~14 frames with
                # "read failed (status=False)". Copying releases each frame back.
                frames.append({k2: (np.array(v, copy=True) if hasattr(v, "shape") else v)
                               for k2, v in obs.items()})

                # Live status. Redrawn on one line so it does not scroll away, and it
                # reports arm movement -- a frozen "moving" figure means the arm is not
                # actually being posed and the episode will teach nothing.
                now = time.perf_counter()
                if now - last_draw > 0.2:
                    el = now - t0
                    mv = 0.0
                    if len(states) > 6:
                        a_, b_ = states[-7], states[-1]
                        mv = sum(abs(b_[k2] - a_[k2]) for k2 in b_)
                    bar = "MOVING" if mv > 0.8 else "still "
                    sys.stdout.write(
                        f"\r    {el:6.1f}s   {len(frames):5d} frames   {bar}   "
                        f"(cap {args.max_seconds:.0f}s)   ")
                    sys.stdout.flush()
                    last_draw = now

                if now - t0 > args.max_seconds:
                    print("\n    time cap reached")
                    verdict = "keep"
                time.sleep(max(0.0, period - (time.perf_counter() - loop_t)))

            sys.stdout.write("\r" + " " * 78 + "\r")
            sys.stdout.flush()

            # Hand the object back as soon as the take ends. The jaws hold
            # whatever position was last commanded, so after a take that closed
            # them they stay clamped on the bottle -- and the keyboard is not
            # read again until the next episode prompt, so there is no way to
            # open them without prising the bottle out. Doing it here rather
            # than after save_episode() returns the bottle during the 30-60 s
            # video encode instead of after it, which is when you want to be
            # resetting the scene. The arm is limp, so anything still lifted
            # will drop the moment the jaws open.
            if args.powered_gripper and not args.no_release:
                grip = args.grip_release
                robot.bus.sync_write("Goal_Position", {GRIPPER: grip})
                print(f"    gripper opened to {grip:.0f}/100 -- object released, "
                      "ready for the next take")
            elif args.powered_gripper:
                print("    jaws held (--no-release) -- take the object out by hand")

            n = len(frames)
            if verdict == "discard":
                print(f"    thrown away ({n} frames)" if bus_lost
                      else f"    thrown away on request ({n} frames)")
                if bus_lost:
                    break
                continue
            if n < args.fps:
                print(f"    too short to keep -- only {n} frames ({n / args.fps:.1f}s). "
                      "Give it at least a second.")
                if bus_lost:
                    break
                continue

            # action[t] = state[t+1]; the final frame has no successor, so drop it.
            print(f"  \u25a0 STOPPED  --  {n} frames ({n / args.fps:.1f}s). Building episode...")
            for i in range(n - 1):
                obs_frame = build_dataset_frame(dataset.features, frames[i], prefix="observation")
                act_frame = build_dataset_frame(dataset.features, states[i + 1], prefix="action")
                dataset.add_frame({**obs_frame, **act_frame, "task": args.task})
            print("  \u25b6 SAVING + ENCODING VIDEO  --  the Svt[info] lines below are normal, "
                  "please wait...")
            t_save = time.perf_counter()
            dataset.save_episode()
            kept += 1
            print(f"\n  \u2713 SAVED episode {kept}/{args.episodes}  --  {n - 1} frames "
                  f"({(n - 1) / args.fps:.1f}s), took {time.perf_counter() - t_save:.1f}s to encode")
            now_neck = neck_position(args.port)
            if neck0 and now_neck:
                drift = max(abs(now_neck[k2] - neck0[k2]) for k2 in neck0)
                if drift > NECK_DRIFT_WARN:
                    print(f"  \u26a0 CAMERA MOVED {drift} counts (~{drift * 0.0879:.0f} deg) since "
                          "this session started. Episodes recorded before and after this point "
                          "see different views, which will hurt the policy. Re-aim to "
                          f"{neck0['head_motor_1']}, {neck0['head_motor_2']} or restart the dataset.")

            if bus_lost:
                # The take was long enough to keep and it is now safely on disk.
                # Stop here rather than opening the next episode on a bus that
                # has already failed once -- the saved episodes are what matter,
                # and --resume picks up from them.
                print(f"\n  Stopping after {kept} saved episode(s): the bus dropped out.\n"
                      f"  Re-seat the adapter, then continue with --resume.")
                break

    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        keys.stop.set()
        try:
            if args.powered_gripper and not args.no_release:
                robot.bus.sync_write("Goal_Position", {GRIPPER: args.grip_release})
        except Exception:
            pass
        try:
            robot.bus.enable_torque()
        except Exception:
            pass
        try:
            robot.disconnect()
        except Exception:
            pass
        print(f"\n  {kept} episodes at {dataset.root}")
        if kept == 0:
            shutil.rmtree(dataset.root, ignore_errors=True)
            print("  (nothing recorded -- empty dataset removed)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
