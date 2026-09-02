#!/usr/bin/env python3
"""Reach-accuracy campaign over a grid of bottle positions.

Three commands, one JSON of cells, one CSV of results.

  reference  Record the operator's grasp pose for a cell: bottle on the mark, arm
             limp, put the jaws around the bottle by hand, ENTER.  Stored in the
             grid file as the cell's reference pose.  Do it once per cell (three
             times and take the mean if you want the operator's own spread).

  run        One policy trial at a cell: replay the start pose, run
             act_policy_control.py with every log switched on, then measure the
             close pose against the reference and ask for the outcome.

  report     Bias / position-dependence / variance tables per configuration,
             in joint units and, via the references' own spacing, in cm.

Reach error is measured in JOINT SPACE at the moment the policy commits: the
observed state at the first step whose gripper command drops below 50, minus
the cell's reference pose.  The servo path reproduces a commanded pose to about
one unit (scripts/replay_episode.py), so this is the plan's error, not the
arm's.  A linear map fitted from the references (joints -> cell x/y in cm) turns
it into an X/Y error on the shelf.

Grid file (JSON): {"cells": {"A1": {"x_cm": 0, "y_cm": 0, "ref": [6 joints]}, ...}}

    python scripts/grid_trials.py reference --grid trials/grid/grid.json --cell A1
    python scripts/grid_trials.py run --grid trials/grid/grid.json --cell A1 --config v8_freeze \
        --checkpoint outputs/act_glassbottle_pick_v8/checkpoints/last/pretrained_model \
        -- --freeze-frame --n-action-steps 100
    python scripts/grid_trials.py report --grid trials/grid/grid.json
"""
from __future__ import annotations
import argparse, csv, json, subprocess, sys, time
from pathlib import Path
import numpy as np

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
HERE = Path(__file__).resolve().parent
RUNNER = Path.home() / "xlerobot-pro" / "examples" / "policies" / "act_policy_control.py"
PY = sys.executable
OUTCOMES = ["grasp", "short", "long", "lateral", "no-close", "knocked", "invalid"]


def load_grid(p: Path) -> dict:
    return json.load(open(p)) if p.exists() else {"cells": {}}


def save_grid(p: Path, g: dict) -> None:
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(g, indent=1))


def cmd_reference(a):
    from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
    sys.path.insert(0, str(HERE))
    from record_kinesthetic import harden_bus
    from xle_arms import SO101FollowerArm
    g = load_grid(a.grid)
    cell = g["cells"].setdefault(a.cell, {"x_cm": a.x_cm, "y_cm": a.y_cm})
    if a.x_cm is not None:
        cell["x_cm"], cell["y_cm"] = a.x_cm, a.y_cm
    robot = SO101FollowerArm(SO101FollowerConfig(port=a.port, id=f"{a.arm}_follower", cameras={}), arm=a.arm)
    harden_bus(robot.bus); robot.connect(calibrate=False)
    robot.bus.disable_torque()
    print(f"\n  Cell {a.cell}: bottle on its mark. Arm is limp: put the jaws around the bottle exactly as you"
          "\n  would grasp it, then press ENTER.")
    input()
    obs = robot.get_observation()
    pose = [float(obs[f"{n}.pos"]) for n in NAMES]
    robot.disconnect()
    cell.setdefault("refs", []).append(pose)
    cell["ref"] = np.mean(cell["refs"], axis=0).tolist()
    save_grid(a.grid, g)
    print("  recorded " + " ".join(f"{n[:5]} {v:+.1f}" for n, v in zip(NAMES, pose)) + f"  ({len(cell['refs'])} refs, mean stored)")


def close_pose_from_csv(traj: Path):
    rows = list(csv.DictReader(open(traj)))
    for i, r in enumerate(rows):
        if float(r["cmd.gripper"]) < 50:
            st = [float(r[f"state.{n}"]) for n in NAMES]
            j = min(i + 15, len(rows) - 1)
            settled = [float(rows[j][f"state.{n}"]) for n in NAMES]
            return i, float(r["t_s"]), st, settled
    return None, None, None, None


def cmd_run(a):
    g = load_grid(a.grid)
    cell = g["cells"].get(a.cell)
    if not cell or "ref" not in cell:
        sys.exit(f"cell {a.cell} has no reference pose; run `reference` first")
    out = a.grid.parent / a.config / a.cell
    out.mkdir(parents=True, exist_ok=True)
    n = 1 + len(list(out.glob("t*_traj.csv")))
    tag = f"t{n:02d}"
    print(f"\n  {a.config} / cell {a.cell} / trial {n}.  Put the bottle on mark {a.cell}. ENTER when clear.")
    input()
    subprocess.run([PY, str(HERE / "goto_start_pose.py"), "--port", a.port, "--arm", a.arm, "--seconds", "5"], check=True)
    runner = [PY, str(RUNNER), "run", "--checkpoint", str(a.checkpoint), "--port", a.port, "--arm", a.arm,
              "--duration", str(a.duration), "--task", a.task,
              "--log-trajectory", str(out / f"{tag}_traj.csv"), "--log-frames", str(out / f"{tag}_frames"),
              "--log-plans", str(out / f"{tag}_plans.csv")] + a.runner_args
    t0 = time.time()
    subprocess.run(runner, check=False)
    k, t_close, st, settled = close_pose_from_csv(out / f"{tag}_traj.csv")
    ref = np.array(cell["ref"])
    if st is None:
        print("  policy never closed the gripper")
        err = [float("nan")] * 6
    else:
        err = (np.array(settled) - ref).tolist()
        print("  close at step %d (%.1f s); settled - reference: " % (k, t_close)
              + " ".join(f"{n[:5]} {v:+.1f}" for n, v in zip(NAMES, err)))
    while True:
        o = input(f"  outcome {OUTCOMES}: ").strip()
        if o in OUTCOMES:
            break
    note = input("  note (optional): ").strip()
    res = a.grid.parent / "results.csv"
    new = not res.exists()
    with open(res, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["time", "config", "cell", "trial", "x_cm", "y_cm", "outcome", "close_step", "t_close"]
                       + [f"err.{n}" for n in NAMES] + [f"close.{n}" for n in NAMES] + ["runner_args", "note"])
        w.writerow([time.strftime("%Y-%m-%d %H:%M"), a.config, a.cell, n, cell["x_cm"], cell["y_cm"], o, k, t_close]
                   + [round(v, 2) for v in err] + [round(v, 2) for v in (settled or [float("nan")] * 6)]
                   + [" ".join(a.runner_args), note])
    print(f"  appended to {res}  ({time.time() - t0:.0f} s)")


def cmd_report(a):
    import pandas as pd
    g = load_grid(a.grid)
    res = a.grid.parent / "results.csv"
    df = pd.read_csv(res)
    df = df[df.outcome != "invalid"]
    # joints -> shelf cm map from the references (least squares on pan, lift, elbow, wrist_flex)
    cells = [(c, v) for c, v in g["cells"].items() if "ref" in v]
    J = np.array([v["ref"][:4] for _, v in cells]); XY = np.array([[v["x_cm"], v["y_cm"]] for _, v in cells])
    A = np.c_[J, np.ones(len(J))]
    M, *_ = np.linalg.lstsq(A, XY, rcond=None)          # (5,2)
    fit = A @ M; rmse = np.sqrt(((fit - XY) ** 2).mean(0))
    print(f"# Grid report — {len(cells)} referenced cells, joint->cm map residual rmse x {rmse[0]:.1f} cm, y {rmse[1]:.1f} cm\n")
    E = df[[f"err.{n}" for n in NAMES[:4]]].values
    xy_err = E @ M[:4]
    df["ex_cm"], df["ey_cm"] = xy_err[:, 0], xy_err[:, 1]
    for cfg, d in df.groupby("config"):
        print(f"## {cfg}  (n={len(d)}, grasp rate {(d.outcome == 'grasp').mean():.0%})\n")
        print("| cell | n | grasp | mean err pan/lift/elbow/wflex | sd lift | X err cm (mean±sd) | Y err cm (mean±sd) |")
        print("|---|---|---|---|---|---|---|")
        for cell, dc in d.groupby("cell"):
            e = dc[[f"err.{n}" for n in NAMES[:4]]].values
            print(f"| {cell} | {len(dc)} | {(dc.outcome == 'grasp').sum()} | "
                  + "/".join(f"{v:+.1f}" for v in np.nanmean(e, 0)) + f" | {np.nanstd(e[:,1]):.1f} | "
                  f"{dc.ex_cm.mean():+.1f}±{dc.ex_cm.std():.1f} | {dc.ey_cm.mean():+.1f}±{dc.ey_cm.std():.1f} |")
        ok = d.dropna(subset=["err.shoulder_lift"])
        if len(ok) > 3:
            bias = ok[["ex_cm", "ey_cm"]].mean().values
            Xc = np.c_[ok.x_cm, ok.y_cm, np.ones(len(ok))]
            bx, *_ = np.linalg.lstsq(Xc, ok.ex_cm, rcond=None); by, *_ = np.linalg.lstsq(Xc, ok.ey_cm, rcond=None)
            rx = ok.ex_cm - Xc @ bx; ry = ok.ey_cm - Xc @ by
            print(f"\nconstant bias: X {bias[0]:+.1f} cm, Y {bias[1]:+.1f} cm"
                  f"\nposition dependence (err per cm of cell x, y): X err {bx[0]:+.2f}, {bx[1]:+.2f};  Y err {by[0]:+.2f}, {by[1]:+.2f}"
                  f"\nrandom variance after removing bias+trend: X sd {rx.std():.1f} cm, Y sd {ry.std():.1f} cm\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("reference", "run", "report"):
        s = sub.add_parser(name)
        s.add_argument("--grid", type=Path, default=Path("trials/grid/grid.json"))
        if name != "report":
            s.add_argument("--cell", required=True); s.add_argument("--port", default="/dev/xle_head"); s.add_argument("--arm", default="left")
        if name == "reference":
            s.add_argument("--x-cm", type=float, default=None); s.add_argument("--y-cm", type=float, default=None)
        if name == "run":
            s.add_argument("--config", required=True, help="Name for this deployment configuration, e.g. v8_freeze_n100.")
            s.add_argument("--checkpoint", type=Path, required=True)
            s.add_argument("--duration", type=float, default=25.0)
            s.add_argument("--task", default="pick up the glass bottle from the shelf")
            s.add_argument("runner_args", nargs="*", help="Extra act_policy_control.py args after --")
    a = ap.parse_args()
    {"reference": cmd_reference, "run": cmd_run, "report": cmd_report}[a.cmd](a)


if __name__ == "__main__":
    main()
