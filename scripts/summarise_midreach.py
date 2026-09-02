#!/usr/bin/env python3
"""Tables from `eval_midreach.py` JSON: plan error by reach fraction, by bottle depth, and by variant.

    python scripts/summarise_midreach.py trials/midreach_v7_040000.json [--live trials/midreach_v7_040000_live_v7.json]
"""
from __future__ import annotations
import argparse, json
import numpy as np

NAMES = ["pan", "lift", "elbow", "wflex"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json")
    ap.add_argument("--live", nargs="*", default=[])
    a = ap.parse_args()
    R = json.load(open(a.json))
    variants = R["variants"]
    labels = ["start"] + [f"f{f:.2f}" for f in R["fracs"]] + ["dwell", "close-5"]
    eps = R["episodes"]
    print(f"# {R['checkpoint']}   {len(eps)} episodes\n")
    print("onset/arrive/grasp frames: median "
          f"{np.median([e['onset'] for e in eps]):.0f} / {np.median([e['arrive'] for e in eps]):.0f} / {np.median([e['grasp'] for e in eps]):.0f}\n")
    truth = {}
    for e in eps:
        truth[e["ep"]] = np.array(e["rows"][0]["true"])
    for v in variants:
        print(f"## variant `{v}`\n")
        print("| frame | n | never closes | planned close vs true frames-to-close (median) | "
              "signed err pan / lift / elbow / wflex (mean) | abs err lift / elbow (mean) | corr(err_lift, true_lift) | slope err_lift~true_lift |")
        print("|---|---|---|---|---|---|---|---|")
        for lab in labels:
            errs, closes, trues, tl = [], [], [], []
            for e in eps:
                for r in e["rows"]:
                    if r["label"] != lab:
                        continue
                    p = r["plans"][v]
                    pose = np.array(p["pose"][:4]); t = np.array(r["true"][:4])
                    errs.append(pose - t); tl.append(t[1])
                    closes.append((p["close"], r["frames_to_close"]))
            E = np.array(errs); tl = np.array(tl)
            never = sum(1 for c, _ in closes if c is None)
            cs = [(c, f) for c, f in closes if c is not None]
            med = f"{np.median([c for c, _ in cs]):.0f} vs {np.median([f for _, f in cs]):.0f}" if cs else "—"
            corr = np.corrcoef(E[:, 1], tl)[0, 1] if len(E) > 2 else float("nan")
            slope = np.polyfit(tl, E[:, 1], 1)[0] if len(E) > 2 else float("nan")
            print(f"| {lab} | {len(E)} | {never} | {med} | "
                  + " / ".join(f"{x:+.1f}" for x in E.mean(0)) + f" | {np.abs(E[:,1]).mean():.1f} / {np.abs(E[:,2]).mean():.1f} | {corr:+.2f} | {slope:+.2f} |")
        print()
        # depth split at f0.50: far (true lift > median) vs near
        errs = []
        for e in eps:
            for r in e["rows"]:
                if r["label"] == "f0.50":
                    p = r["plans"][v]; errs.append((r["true"][1], np.array(p["pose"][:4]) - np.array(r["true"][:4]), p["close"], r["frames_to_close"]))
        if errs:
            tl = np.array([x[0] for x in errs]); med = np.median(tl)
            for name, m in (("near (true lift < median)", tl < med), ("far (true lift >= median)", tl >= med)):
                E = np.array([x[1] for x in errs])[m]
                print(f"  f0.50 {name}: n={m.sum()} signed err pan/lift/elbow/wflex = "
                      + " / ".join(f"{x:+.1f}" for x in E.mean(0))
                      + f"; never closes {sum(1 for x,mm in zip(errs,m) if mm and x[2] is None)}")
            print()
    for lp in a.live:
        L = json.load(open(lp))["live"]
        print(f"## live frames `{lp}`\n")
        print("| frame | state lift/elbow/grip | " + " | ".join(f"{v}: close, lift/elbow/wflex, max lift" for v in variants) + " |")
        print("|---|---|" + "---|" * len(variants))
        for r in L:
            st = r["state"]
            cells = []
            for v in variants:
                p = r["plans"][v]
                cells.append(f"{p['close']}, {p['pose'][1]:+.0f}/{p['pose'][2]:+.0f}/{p['pose'][3]:+.0f}, {p['max_lift']:+.0f}")
            print(f"| {r['name']} | {st[1]:+.0f}/{st[2]:+.0f}/{st[5]:.0f} | " + " | ".join(cells) + " |")
        print()


if __name__ == "__main__":
    main()
