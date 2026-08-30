#!/usr/bin/env python3
"""Check that recorded episodes contain a real demonstration.

A saved episode is not necessarily a usable one. The recorder's console
``MOVING``/``still`` indicator cannot settle it: it redraws in place with
``\\r``, so the only lines that survive in scrollback are the instants a
gripper message forced a newline -- exactly the moments your hands were on the
keyboard rather than the arm. The data is the only honest witness, so measure
it here.

Three questions, per episode:

* **Did the arm move?** Per-joint travel, as both range (p99-p1, robust to a
  single spurious count) and total path length. A take where the wrist wandered
  but the shoulder never left its start is not a pick.
* **Did the gripper work?** Range and the number of open/close transitions. A
  pick has at least one close; a pick-and-place has a close and an open.
* **Is the action/state relationship intact?** The recorder defines the action
  at frame *t* as the state at *t+1*. If that shifted or was written as the
  identity, the policy learns to predict where it already is, which trains to a
  low loss and does nothing on the robot. This is the failure that looks most
  like success, so it is checked exactly rather than approximately.

Reads the parquet directly -- no robot, no lerobot import, no GPU. Run it on
whichever machine holds the dataset.

    python scripts/verify_episodes.py --repo-id local/medbottle_pick
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

#: Below this much travel in a joint's normalised units (body joints span
#: -100..100, so this is ~5% of full scale) the joint did not meaningfully
#: participate. Not a pass mark on its own -- a wrist-only motion can clear it
#: -- which is why the per-joint numbers are printed rather than reduced to a
#: verdict.
QUIET_JOINT = 5.0

#: A pick needs the gripper to actually close. Same units, 0..100.
QUIET_GRIPPER = 5.0

#: Half-scale hysteresis for counting gripper transitions, so sensor noise
#: around the threshold is not counted as a dozen separate grasps.
GRIP_HYST = 0.25

#: A joint that moves more than this in one 1/fps frame did not move -- the
#: number did. Real hand-guided motion on this arm peaks near 5 units per frame,
#: so 20 is far above anything physical while still catching the failure this
#: exists for: an encoder wrap, which lands as a step of ~200 (the full -100..100
#: span) when a joint's calibrated range spans the raw 0/4095 boundary. The
#: policy would learn to predict a teleport, so this is checked explicitly.
MAX_STEP = 20.0


def load(root: Path) -> tuple[dict, pd.DataFrame]:
    info = json.loads((root / "meta" / "info.json").read_text())
    files = sorted(root.glob("data/**/*.parquet"))
    if not files:
        raise SystemExit(f"no parquet under {root / 'data'}")
    df = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    return info, df


def names(info: dict, key: str) -> list[str]:
    """Feature names, across the layouts lerobot has used for them."""
    n = info.get("features", {}).get(key, {}).get("names")
    if isinstance(n, dict):                    # {"motors": [...]}
        n = next(iter(n.values()))
    if not n:                                  # fall back to indices
        shape = info["features"][key]["shape"][0]
        n = [f"{key}[{i}]" for i in range(shape)]
    return list(n)


def stack(col: pd.Series) -> np.ndarray:
    return np.asarray([np.asarray(v, dtype=float) for v in col])


def transitions(g: np.ndarray) -> int:
    """Count grasp open/close events with hysteresis around the midpoint."""
    lo, hi = g.min(), g.max()
    if hi - lo < QUIET_GRIPPER:
        return 0
    mid, half = (hi + lo) / 2, (hi - lo) * GRIP_HYST
    state, n = None, 0
    for v in g:
        if v > mid + half and state != "open":
            n, state = n + (state is not None), "open"
        elif v < mid - half and state != "closed":
            n, state = n + (state is not None), "closed"
    return n


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--repo-id", required=True)
    p.add_argument("--root", type=Path, default=None,
                   help="Dataset root. Default ~/.cache/huggingface/lerobot/<repo-id>")
    p.add_argument("--quiet-joint", type=float, default=QUIET_JOINT)
    args = p.parse_args()

    root = args.root or Path.home() / ".cache/huggingface/lerobot" / args.repo_id
    if not root.exists():
        raise SystemExit(f"no dataset at {root}")

    info, df = load(root)
    fps = info.get("fps", 30)
    jn = names(info, "action")
    print(f"\n  {root}")
    print(f"  {info.get('total_episodes', '?')} episodes, {len(df)} frames, {fps} fps, "
          f"robot={info.get('robot_type', '?')}\n")

    bad = 0
    for ep, g in df.groupby("episode_index", sort=True):
        g = g.sort_values("frame_index")
        state, action = stack(g["observation.state"]), stack(g["action"])

        print(f"  episode {int(ep)}   {len(g)} frames   {len(g) / fps:.1f} s")

        jumped = False

        # -- action[t] == state[t+1] -------------------------------------
        shift_ok = True
        if len(g) > 1:
            err = np.abs(action[:-1] - state[1:]).max()
            ident = np.abs(action - state).max()
            ok = err < 1e-6
            print(f"    action[t] == state[t+1] : {'yes' if ok else 'NO'}"
                  f"   (max |diff| {err:.2e};  vs identity action[t]==state[t] {ident:.2e})")
            shift_ok = ok
            if not ok:
                print("      ^ the shift is wrong or absent. A policy trained on this learns "
                      "to output\n        its own current position, which scores a low loss "
                      "and does not move the arm.")

        # -- discontinuities ---------------------------------------------
        if len(g) > 1:
            step = np.abs(np.diff(state, axis=0))
            jumpy = [(jn[i], step[:, i]) for i in range(len(jn))
                     if step[:, i].max() > MAX_STEP]
            if jumpy:
                jumped = True
                print(f"    JUMPS  : {len(jumpy)} joint(s) move impossibly far in one frame")
                for n, d in jumpy:
                    where = np.where(d > MAX_STEP)[0]
                    f = int(where[0])
                    kind = ("encoder wrap -- this joint's calibrated range straddles the "
                            "raw 0/4095\n               boundary; recalibrate it"
                            if d.max() > 150 else "unexplained discontinuity")
                    print(f"      {n:<24} {len(where)} jump(s), max {d.max():.1f} "
                          f"at frame {f}: {state[f, jn.index(n)]:.1f} -> "
                          f"{state[f + 1, jn.index(n)]:.1f}")
                    print(f"               {kind}")
            else:
                print(f"    max single-frame step   : {step.max():.2f} "
                      f"(limit {MAX_STEP:.0f}) -- no wraps")

        # -- travel ------------------------------------------------------
        rng = np.percentile(state, 99, axis=0) - np.percentile(state, 1, axis=0)
        path = np.abs(np.diff(state, axis=0)).sum(axis=0)
        quiet = []
        print("    joint travel (normalised units)      range     path")
        for i, n in enumerate(jn):
            gripper = "gripper" in n
            thr = QUIET_GRIPPER if gripper else args.quiet_joint
            flag = "  <- barely moved" if rng[i] < thr else ""
            if rng[i] < thr:
                quiet.append(n)
            print(f"      {n:<28} {rng[i]:8.1f} {path[i]:8.1f}{flag}")

        gi = [i for i, n in enumerate(jn) if "gripper" in n]
        grasped = True
        if gi:
            t = transitions(state[:, gi[0]])
            print(f"    gripper transitions     : {t}"
                  f"{'   <- never actuated' if t == 0 else ''}")
            grasped = t > 0

        # The arm and the gripper fail independently: a take can be a flawless
        # reach that never closed the jaws, or a clean grasp that never went
        # anywhere. Judge them separately or one masks the other.
        arm = [n for n in jn if "gripper" not in n]
        arm_quiet = [n for n in quiet if "gripper" not in n]
        if len(arm_quiet) >= max(1, len(arm) - 1):
            faults = [f"{len(arm_quiet)} of {len(arm)} arm joints barely moved"]
        elif arm_quiet:
            faults = []
            print(f"    note   : {', '.join(arm_quiet)} contributed little.")
        else:
            faults = []
        if not grasped:
            faults.append("the gripper never actuated, so there is no grasp in it")
        if not shift_ok:
            faults.insert(0, "action[t] != state[t+1]")
        if jumped:
            faults.append("a joint jumps discontinuously")

        if faults:
            bad += 1
            print(f"    VERDICT: not a usable demonstration -- {'; '.join(faults)}.")
        else:
            print("    VERDICT: arm and gripper both worked.")
        print()

    if bad:
        print(f"  {bad} problem(s) found. Do not train on this dataset until they are "
              "understood.\n")
        return 1
    print("  All episodes look like real demonstrations.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
