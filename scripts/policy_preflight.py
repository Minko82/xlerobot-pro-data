#!/usr/bin/env python3
"""Preflight for the record -> train -> deploy chain, before you stand at the bench.

Sec. VI-E is the paper's largest gap, and the reason it is still open is not bench
time: it is that **no demonstration dataset and no trained checkpoint exist**.
``diffusion_policy_control.py`` falls through to "No checkpoint loaded. Using random
weights.", so a trial run today would be measuring noise.

The chain is therefore three steps, not one:

    record_kinesthetic.py   ->   lerobot-train   ->   policy_trials.py
    (bench, 2-3 h)               (off-robot, 1-2 h)   (bench, ~75 min)

Everything that can go wrong in step 1 goes wrong *at the bench*, with the arm
limp and an episode half-recorded: an uncalibrated arm, the wrong RealSense
serial, a dataset directory that already exists, a missing dependency. This
checks all of it in advance, moves nothing, and prints the exact commands for
all three steps with the values already filled in.

    python scripts/policy_preflight.py
    python scripts/policy_preflight.py --arm right --repo-id local/can_pickplace

Exit status is 0 when the chain is ready to start, 1 when something blocks it.
"""

from __future__ import annotations

import argparse
import importlib
import shutil
from pathlib import Path

#: Rough cost of one episode on disk: 45 s cap x 30 fps of encoded 640x480 video
#: plus parquet. Measured takes run well under this; it is a ceiling so the disk
#: check errs towards warning early.
MB_PER_EPISODE = 12.0

OK, WARN, BAD = "  ok  ", " warn ", " BLOCK"


class Report:
    def __init__(self) -> None:
        self.blocked = 0
        self.warned = 0

    def line(self, status: str, label: str, detail: str = "") -> None:
        if status == BAD:
            self.blocked += 1
        elif status == WARN:
            self.warned += 1
        print(f"  [{status}] {label:<34} {detail}")

    def ok(self, label, detail=""):    self.line(OK, label, detail)
    def warn(self, label, detail=""):  self.line(WARN, label, detail)
    def bad(self, label, detail=""):   self.line(BAD, label, detail)


def check_imports(r: Report) -> None:
    print("\nDependencies")
    for mod, why in (
        ("lerobot", "the dataset, robot and training stack"),
        ("torch", "training and inference"),
        ("numpy", "recording"),
    ):
        try:
            m = importlib.import_module(mod)
            v = getattr(m, "__version__", "")
            r.ok(mod, v)
        except Exception as exc:
            r.bad(mod, f"{type(exc).__name__}: {exc} -- needed for {why}")

    try:
        importlib.import_module("pyrealsense2")
        r.ok("pyrealsense2")
    except Exception:
        r.bad("pyrealsense2",
              "no RealSense bindings; recording captures no images. "
              "Raw V4L2 is not a substitute -- it green-casts the colour pipeline.")

    try:
        import torch
        if torch.cuda.is_available():
            r.ok("torch device", f"cuda -- {torch.cuda.get_device_name(0)}")
        elif getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
            r.warn("torch device", "mps (Apple Silicon) -- fine for training, not the robot")
        else:
            r.warn("torch device", "cpu only -- ACT training will take many hours")
    except Exception:
        pass


def check_buses(r: Report, port: str) -> None:
    print("\nHardware")
    for p, what in ((port, "arms"), ("/dev/xle_head", "head + wheels")):
        if Path(p).exists():
            r.ok(f"bus {p}", what)
        else:
            r.bad(f"bus {p}", f"missing -- {what} bus not enumerated. "
                              "Check the udev rules and the adapter serials.")


def check_calibration(r: Report, robot_id: str) -> bool:
    try:
        from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
    except Exception:
        r.warn("calibration", "cannot import lerobot constants; skipped")
        return False
    fpath = Path(HF_LEROBOT_CALIBRATION) / ROBOTS / "so101_follower" / f"{robot_id}.json"
    if fpath.exists():
        r.ok("calibration", str(fpath))
        return True
    r.bad("calibration", f"no {fpath.name}. Recording will refuse to start.")
    return False


def check_camera(r: Report, serial: str) -> None:
    try:
        import pyrealsense2 as rs
    except Exception:
        return
    try:
        found = [d.get_info(rs.camera_info.serial_number)
                 for d in rs.context().query_devices()]
    except Exception as exc:
        r.warn("realsense", f"enumeration failed: {exc}")
        return
    if not found:
        r.bad("realsense", "no device found on the bus")
    elif serial in found:
        r.ok("realsense", f"serial {serial}")
    else:
        r.bad("realsense", f"serial {serial} not present. Connected: {', '.join(found)}. "
                           "Pass the right one with --camera-serial.")


def check_dataset(r: Report, repo_id: str, root: Path | None, episodes: int) -> None:
    print("\nDataset")
    if root is None:
        try:
            from lerobot.utils.constants import HF_LEROBOT_HOME
            root = Path(HF_LEROBOT_HOME)
        except Exception:
            root = Path.home() / ".cache/huggingface/lerobot"
    target = Path(root) / repo_id
    if target.exists():
        n = len(list((target / "data").rglob("*.parquet"))) if (target / "data").exists() else 0
        r.warn("target directory", f"{target} exists ({n} parquet files). "
                                   "Add --resume to append, or pick another --repo-id.")
    else:
        r.ok("target directory", f"{target} (will be created)")

    probe = Path(root)
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    free_mb = shutil.disk_usage(probe).free / 1e6
    need_mb = episodes * MB_PER_EPISODE
    if free_mb < need_mb * 2:
        r.bad("disk space", f"{free_mb:,.0f} MB free, ~{need_mb:,.0f} MB needed for "
                            f"{episodes} episodes")
    else:
        r.ok("disk space", f"{free_mb:,.0f} MB free, ~{need_mb:,.0f} MB needed")


#: The repo sits in different places on the Jetson and on a laptop, so look rather
#: than assume -- a hardcoded path reports "nothing here" on the machine that
#: actually has the runners, which is worse than no check at all.
REPO_CANDIDATES = (
    Path.home() / "xlerobot-pro",
    Path.home() / "Desktop/Workspace/xlerobot-pro",
    Path(__file__).resolve().parent.parent.parent / "xlerobot-pro",
)


def find_repo() -> Path | None:
    for c in REPO_CANDIDATES:
        if (c / "examples" / "policies").is_dir():
            return c
    return None


def check_runner(r: Report) -> None:
    """Is there anything that can actually execute an ACT checkpoint on the robot?"""
    repo = find_repo()
    if repo is None:
        r.warn("ACT runner", "cannot find the xlerobot-pro checkout; skipped")
        return
    pol = repo / "examples" / "policies"
    have = sorted(f.name for f in pol.glob("*_policy_control.py"))
    if any("act" in n for n in have):
        r.ok("ACT runner", "present")
    else:
        r.bad("ACT runner",
              f"none. {pol} has {', '.join(have) or 'nothing'} -- a trained ACT "
              "checkpoint has nothing to run it. policy_trials.py --cmd needs a "
              "script that loads the policy and drives the arm.")


def check_checkpoints(r: Report) -> None:
    print("\nExisting artefacts")
    repo = find_repo()
    roots = [Path.home() / ".cache/huggingface/lerobot",
             Path.home() / "outputs",
             Path.home() / "xlerobot-pro-data",
             Path.home() / "Desktop/Workspace/xlerobot-pro-data"]
    if repo:
        roots.append(repo / "outputs")
    found = []
    for root in roots:
        if root.exists():
            found += [p for p in root.rglob("*.safetensors")][:5]
    if found:
        r.ok("checkpoints", f"{len(found)} found, e.g. {found[0]}")
    else:
        r.warn("checkpoints", "none anywhere -- every trial before training runs "
                              "on random weights and measures nothing.")


def commands(args, calibrated: bool) -> None:
    bar = "-" * 72
    ckpt = f"outputs/act_{args.repo_id.split('/')[-1]}/checkpoints/last/pretrained_model"
    print(f"\n{bar}\nRun these, in order\n{bar}")

    if not calibrated:
        print(f"""
1. Calibrate the {args.arm} arm (once; the two arms need different ids or they
   overwrite each other):

   python scripts/record_kinesthetic.py --repo-id {args.repo_id} \\
       --arm {args.arm} --calibrate
""")

    print(f"""{'2' if not calibrated else '1'}. Record. Bench work -- kinesthetic teaching owns the arms bus exactly as
   the thermal holds do, so it cannot overlap with Phase 2.

   python scripts/record_kinesthetic.py \\
       --repo-id {args.repo_id} --arm {args.arm} --episodes {args.episodes} \\
       --task "{args.task}"

   Keep the neck still for the whole session. The policy learns pixels -> joint
   commands, so a re-aimed camera mid-dataset means the same scene produces
   different images. The script warns above ~4 degrees of drift.

{'3' if not calibrated else '2'}. Train ACT. Off-robot, so it can run while the long holds do.

   lerobot-train \\
       --policy.type=act \\
       --dataset.repo_id={args.repo_id} \\
       --output_dir=outputs/act_{args.repo_id.split('/')[-1]} \\
       --job_name=act_{args.repo_id.split('/')[-1]} \\
       --policy.device={args.device} \\
       --steps={args.steps} --batch_size={args.batch_size} \\
       --policy.chunk_size={args.chunk} --policy.n_action_steps={args.chunk} \\
       --save_freq=5000 --log_freq=200 --wandb.enable=false

{'4' if not calibrated else '3'}. Trials. Point policy_trials.py at the checkpoint you just wrote.

   python scripts/policy_trials.py \\
       --out A2/policy_trials/act_static --trials 20 --condition static \\
       --policy ACT --task "{args.task}" \\
       --checkpoint {ckpt} \\
       --cmd "python ../xlerobot-pro/examples/policies/act_policy_control.py \\
              run --checkpoint {ckpt} --duration 30"

   NOTE: act_policy_control.py DOES NOT EXIST YET. examples/policies/ has runners
   for the diffusion policy and SmolVLA only, so a trained ACT checkpoint has
   nothing to execute it and --cmd is a required argument. Write that runner
   while the recording session is still ahead of you, not after.
""")

    print(f"""{bar}
Two choices worth making deliberately
{bar}

chunk_size = {args.chunk}
  ACT's default is 100: one inference, then 100 open-loop action steps. At 30 fps
  that is 3.3 s of committed motion, and a disturbance inside that window cannot
  be corrected. It also decides the realised control rate the trials will report,
  which the paper quotes separately as "Control Rate (K=50) = 50.5 Hz". Keeping
  the two consistent means either training at the K you quote, or saying plainly
  in Sec. VI-E that the deployed chunk differs from the latency-model K.

ACT, not the diffusion policy
  The diffusion policy here is vision-only by construction --
  ConditionalUNet1D.forward takes one conditioning vector built from the visual
  encoder alone, and joint positions are read at :460 then dropped. ACT is
  state+vision by default, so it is both the stronger result and the honest one
  for a paper whose thesis is that proprioceptive load matters.
""")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--repo-id", default="local/bottle_pickplace")
    p.add_argument("--root", type=Path, default=None)
    p.add_argument("--port", default="/dev/xle_arms")
    p.add_argument("--arm", choices=("left", "right"), default="left")
    p.add_argument("--robot-id", default=None)
    p.add_argument("--camera-serial", default="838212073725")
    p.add_argument("--episodes", type=int, default=50)
    p.add_argument("--task", default="pick up the bottle and place it on the other side")
    p.add_argument("--device", default="cuda", help="Training device: cuda, mps or cpu.")
    p.add_argument("--steps", type=int, default=30_000,
                   help="Training steps. 30k is enough for a first pass on ~50 episodes; "
                        "ACT's own default of 100k is for much larger datasets.")
    p.add_argument("--batch-size", type=int, default=8)
    p.add_argument("--chunk", type=int, default=50,
                   help="ACT chunk_size and n_action_steps. Default 50 to match the K "
                        "the paper's latency model quotes, not ACT's own 100.")
    args = p.parse_args()
    if args.robot_id is None:
        args.robot_id = f"{args.arm}_follower"

    print(f"\nPreflight -- {args.arm} arm, dataset {args.repo_id}")
    r = Report()
    check_imports(r)
    check_buses(r, args.port)
    calibrated = check_calibration(r, args.robot_id)
    check_camera(r, args.camera_serial)
    check_dataset(r, args.repo_id, args.root, args.episodes)
    check_runner(r)
    check_checkpoints(r)

    print(f"\n{r.blocked} blocking, {r.warned} to be aware of.")
    commands(args, calibrated)
    if r.blocked:
        print("Fix the BLOCK lines before starting -- each one fails at the bench, "
              "mid-episode, with the arm limp.\n")
    return 1 if r.blocked else 0


if __name__ == "__main__":
    raise SystemExit(main())
