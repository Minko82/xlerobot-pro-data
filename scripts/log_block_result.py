#!/usr/bin/env python3
"""Append one Block A trial to trials/blockA/results.csv from its logs plus the operator's verdict.

    python scripts/log_block_result.py t01 fail APPR centre "swept the bottle right before closing"
"""
import csv, re, sys
from pathlib import Path
import numpy as np, pandas as pd

tag, outcome, stage, location = sys.argv[1:5]
note = sys.argv[5] if len(sys.argv) > 5 else ""
D = Path("trials/blockA")
d = pd.read_csv(D / f"{tag}.csv")
k = np.where(d["cmd.gripper"] < 50)[0]
if len(k):
    k = int(k[0]); s = d.iloc[min(k + 15, len(d) - 1)]
    close = [k, round(float(d.iloc[k].t_s), 2)] + [round(float(s[f"state.{j}"]), 2) for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")]
else:
    close = [-1, None, None, None, None, None]
grip_end = round(float(d["state.gripper"].iloc[-1]), 1)
summ = (D / f"{tag}_summary.txt").read_text() if (D / f"{tag}_summary.txt").exists() else ""
g = lambda pat: (re.search(pat, summ) or [None, None])[1]
temps = lambda f: (D / f).read_text().strip() if (D / f).exists() else ""
t0 = temps(f"{tag}_temps_start.txt"); t1 = temps(f"{tag}_temps_end.txt")
lift0 = re.search(r"shoul=\d+ shoul=(\d+)", t0); lift0 = int(lift0[1]) if lift0 else None
row = [tag, outcome, stage, location, lift0, t0, t1] + close + [grip_end, g(r"realised\s+([\d.]+) Hz"), g(r"chunk inf\.\s+([\d.]+) ms"), g(r"max (\d+) ms\)"), g(r"peak joint\s+(\d+)"), note]
res = D / "results.csv"; new = not res.exists()
with open(res, "a", newline="") as f:
    w = csv.writer(f)
    if new:
        w.writerow(["trial", "outcome", "stage", "location", "shoulder_lift_start_C", "temps_start", "temps_end", "close_step", "t_close_s", "close_pan", "close_lift", "close_elbow", "close_wflex", "gripper_end", "realised_hz", "chunk_inf_ms", "chunk_inf_max_ms", "peak_joint_C", "note"])
    w.writerow(row)
print(row)
