#!/usr/bin/env python3
"""Replay a recorded episode's actions through the deployment path and measure what the arm does.

The question this answers is the one no policy trial can: if the commands were
*perfect* -- exactly the joint positions the operator's hand put the arm through
-- where would the arm actually go?  Same robot class, same bus hardening, same
envelope, same 30 Hz loop and `send_action` as `act_policy_control.py`, but the
actions come from the dataset instead of the policy.  Everything between the
policy's output and the encoder is exercised: calibration, normalisation to raw
counts, the torque/acceleration envelope, and the servo's own tracking under
gravity.

Two measurements per episode:

  tracking   |state[t] - recorded_state[t]| along the whole trajectory, and the
             lag (frames) that best aligns them
  grasp pose the arm's settled pose at the recorded close step, versus the
             recorded grasp pose -- the reach error a perfect plan would leave

Starts from wherever the arm is, so run `goto_start_pose.py` first.  The gripper
is replayed too (it closes on nothing).  KEEP THE OBJECT OUT OF THE ARM'S PATH:
pick episodes whose bottle sat far from anything on the shelf now.

    python scripts/replay_episode.py --repo-id local/glassbottle_pick_v7_masked \
        --episode 20 --port /dev/xle_head --arm left --log /tmp/replay_ep20.csv
"""
from __future__ import annotations
import argparse, csv, glob, sys, time
from pathlib import Path
import numpy as np, pandas as pd

from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.utils.robot_utils import busy_wait

sys.path.insert(0, str(Path(__file__).resolve().parent))
from record_kinesthetic import harden_bus  # noqa: E402
from xle_arms import ARM_IDS, SO101FollowerArm  # noqa: E402

sys.path.insert(0, str(Path.home() / "xlerobot-pro" / "examples" / "policies"))
from act_policy_control import apply_envelope, release_gently  # noqa: E402

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def load_episode(repo_id: str, ep: int, root: Path | None):
    root = root or Path.home() / ".cache/huggingface/lerobot" / repo_id
    d = pd.concat([pd.read_parquet(f) for f in sorted(glob.glob(str(root / "data/**/*.parquet"), recursive=True))])
    d = d[d.episode_index == ep].sort_values("frame_index")
    return np.stack(d["action"].values).astype(np.float32), np.stack(d["observation.state"].values).astype(np.float32)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--repo-id", required=True)
    p.add_argument("--root", type=Path, default=None)
    p.add_argument("--episode", type=int, required=True)
    p.add_argument("--port", default="/dev/xle_head")
    p.add_argument("--arm", choices=list(ARM_IDS), default="left")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--until", default="close+60",
                   help="Last frame to replay: an integer, or close+N for N frames past the recorded close.")
    p.add_argument("--hold", type=float, default=2.0, help="Seconds to hold the final command before measuring.")
    p.add_argument("--log", type=Path, required=True)
    p.add_argument("--no-gripper", action="store_true", help="Leave the gripper alone.")
    args = p.parse_args()

    A, S = load_episode(args.repo_id, args.episode, args.root)
    close = np.where(A[:, 5] < 50)[0]
    close = int(close[0]) if len(close) else len(A) - 1
    if args.until.startswith("close"):
        last = min(len(A) - 1, close + int(args.until[5:] or 0))
    else:
        last = min(len(A) - 1, int(args.until))
    print(f"  episode {args.episode}: {len(A)} frames, recorded close at {close}, replaying 0..{last}")
    print(f"  recorded grasp pose: " + " ".join(f"{n[:5]} {v:+.1f}" for n, v in zip(NAMES, A[close])))

    cfg = SO101FollowerConfig(port=args.port, id=f"{args.arm}_follower", cameras={})
    robot = SO101FollowerArm(cfg, arm=args.arm)
    harden_bus(robot.bus)
    robot.connect(calibrate=False)
    apply_envelope(robot)
    obs0 = robot.get_observation()
    s0 = np.array([obs0[f"{n}.pos"] for n in NAMES])
    print("  arm now:             " + " ".join(f"{n[:5]} {v:+.1f}" for n, v in zip(NAMES, s0)))
    print("  recorded frame 0:    " + " ".join(f"{n[:5]} {v:+.1f}" for n, v in zip(NAMES, S[0])))
    if np.abs(s0[:4] - S[0, :4]).max() > 6:
        print("  arm is not at the recorded start pose -- run goto_start_pose.py first", file=sys.stderr)
        robot.disconnect(); return 1

    period = 1.0 / args.fps
    rows = []
    t0 = time.perf_counter()
    try:
        for t in range(last + 1):
            lt = time.perf_counter()
            obs = robot.get_observation()
            act = {f"{n}.pos": float(A[t, j]) for j, n in enumerate(NAMES)}
            if args.no_gripper:
                act.pop("gripper.pos")
            robot.send_action(act)
            rows.append([t, round(time.perf_counter() - t0, 4)]
                        + [float(obs[f"{n}.pos"]) for n in NAMES] + [float(A[t, j]) for j in range(6)]
                        + [float(S[t, j]) for j in range(6)])
            busy_wait(max(0.0, period - (time.perf_counter() - lt)))
        # hold the last command and record the settled pose
        th = time.perf_counter()
        while time.perf_counter() - th < args.hold:
            obs = robot.get_observation()
            rows.append([len(rows), round(time.perf_counter() - t0, 4)]
                        + [float(obs[f"{n}.pos"]) for n in NAMES] + [float(A[last, j]) for j in range(6)]
                        + [float(S[last, j]) for j in range(6)])
            busy_wait(period)
    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        with open(args.log, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["step", "t_s"] + [f"state.{n}" for n in NAMES] + [f"cmd.{n}" for n in NAMES] + [f"rec.{n}" for n in NAMES])
            w.writerows(rows)
        try:
            release_gently(robot)
        except Exception:
            pass
        robot.disconnect()

    R = np.array([r[2:] for r in rows]); n = min(len(R), last + 1)
    st, cmd, rec = R[:n, :6], R[:n, 6:12], R[:n, 12:18]
    print(f"\n  replayed {n} frames in {rows[n-1][1]:.1f} s ({n / rows[n-1][1]:.1f} Hz)")
    print("  tracking vs recorded state, per joint (frames 20..end):")
    for j, name in enumerate(NAMES[:5]):
        best = min(range(0, 12), key=lambda k: np.abs(cmd[20:n - k, j] - st[20 + k:n, j]).mean())
        e = st[20:n, j] - rec[20:n, j]
        print(f"    {name:14s} lag {best:2d} fr   mean(state-rec) {e.mean():+6.2f}   mean|.| {np.abs(e).mean():5.2f}   max|.| {np.abs(e).max():5.2f}")
    # pose at the recorded close and settled pose at the end of the hold
    at_close = st[close] if close < n else None
    settled = R[-1, :6]
    print("  at recorded close step: " + (" ".join(f"{name[:5]} {v - rec[close, j]:+.1f}" for j, (name, v) in enumerate(zip(NAMES, at_close))) if at_close is not None else "n/a")
          + "   (state - recorded)")
    print("  settled after hold:     " + " ".join(f"{name[:5]} {v - A[last, j]:+.1f}" for j, (name, v) in enumerate(zip(NAMES, settled))) + "   (state - last command)")
    print(f"  log: {args.log}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
