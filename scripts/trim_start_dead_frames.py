#!/usr/bin/env python3
"""Drop the still frames at the start of every episode, into a new dataset.

Why. Every take begins with the operator holding the arm at the reference pose
before starting to move -- a median 1.43 s, about 10% of all frames in
glassbottle_pick_v3. With a *varying* start pose that dead time is spread across
twenty different states and harmless; the earlier glassbottle_pick carries the
same 9.7% and its policy moved. With the start pose replayed to 0.3 units it all
lands on ONE state, and roughly a tenth of the dataset then says "when you see
exactly this observation, stay where you are". ACT is Markovian on (image,
state) with no clock, so at deployment it cannot know it has already been still
for thirty seconds. Measured: six consecutive inferences from the start pose
varied by 0.19 units. The arm does not move.

The cut is made on the COMMANDED step, `|action[t] - state[t]|`, not on how
far the arm has travelled. Actions are absolute next positions
(`action[t] == state[t+1]`, exact here), so this is precisely the motion the
policy would ask for at frame 0. Cutting on travelled distance instead keeps a
frame whose command is ~0.09 units -- about one raw count, inside the servo
deadband -- and the deadlock survives one step further along.

The two requirements pull in opposite directions and both are satisfied at the
default threshold, which is why it is 1.0:

  * frame 0's command must be large enough to actually move a servo. At T=1.0
    the smallest is 1.08 normalised units, about 13 raw counts.
  * frame 0 must still look like the pose replayed before every trial, or
    deployment is out of distribution again. It does: the arm has barely moved
    when the command grows, because the servo lags the operator's push. Drift
    from the captured start pose is 0.38 units on shoulder_lift, at most 1.55
    on any joint, with a 1.48-unit spread across the twenty episodes.

The video is never re-encoded. Frames are located by `from_timestamp +
timestamp`, so trimming means rebasing each episode's timestamps to zero and
pushing `from_timestamp` forward by the same amount. The dropped frames simply
stop being referenced.

Image statistics are left as they are: the training config sets
`use_imagenet_stats: True`, so image normalisation uses ImageNet constants and
never reads them. Every other feature's statistics are recomputed.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

#: Joints whose motion decides that a take has started. The gripper is excluded:
#: it is commanded open at t=0 and its small settle would read as "moving".
BODY = slice(0, 5)

#: Normalised units the COMMANDED step must exceed for the take to count as
#: started. See the module docstring for why this is not a travel threshold.
DEFAULT_THRESHOLD = 1.0

STAT_FEATURES = ["action", "observation.state", "timestamp", "frame_index",
                 "episode_index", "index", "task_index"]


def stats_for(arr: np.ndarray) -> dict:
    """Match the layout lerobot writes: every entry a list, count a 1-element list."""
    a = np.asarray(arr, dtype=np.float64)
    if a.ndim == 1:
        a = a[:, None]
    return {
        "min": a.min(axis=0).tolist(),
        "max": a.max(axis=0).tolist(),
        "mean": a.mean(axis=0).tolist(),
        "std": a.std(axis=0).tolist(),
        "count": [int(a.shape[0])],
        "q01": np.percentile(a, 1, axis=0).tolist(),
        "q10": np.percentile(a, 10, axis=0).tolist(),
        "q50": np.percentile(a, 50, axis=0).tolist(),
        "q90": np.percentile(a, 90, axis=0).tolist(),
        "q99": np.percentile(a, 99, axis=0).tolist(),
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--src", type=Path, required=True, help="Dataset to read.")
    p.add_argument("--dst", type=Path, required=True, help="Dataset to write. Must not exist.")
    p.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD,
                   help=f"Normalised units the commanded step |action-state| must "
                        f"exceed. Default {DEFAULT_THRESHOLD}, which yields a first "
                        f"command of at least 1.08 units (~13 raw counts).")
    p.add_argument("--dry-run", action="store_true",
                   help="Report what would be trimmed and write nothing.")
    args = p.parse_args()

    src, dst = args.src, args.dst
    if not (src / "meta" / "info.json").exists():
        raise SystemExit(f"{src} does not look like a LeRobot dataset")
    if dst.exists() and not args.dry_run:
        raise SystemExit(f"{dst} already exists -- refusing to overwrite")

    info = json.loads((src / "meta" / "info.json").read_text())
    fps = float(info["fps"])

    data_files = sorted((src / "data").glob("chunk-*/file-*.parquet"))
    ep_files = sorted((src / "meta" / "episodes").glob("chunk-*/file-*.parquet"))
    episodes = {}
    for f in ep_files:
        for r in pq.read_table(f).to_pylist():
            episodes[r["episode_index"]] = r

    # Per data file, keep rows grouped so the file layout survives: meta rows
    # point at data/chunk_index and data/file_index, and rewriting that mapping
    # is a second chance to get something wrong.
    kept_tables, trim_report = {}, {}
    for f in data_files:
        tbl = pq.read_table(f)
        d = tbl.to_pydict()
        ep = np.array(d["episode_index"])
        fi = np.array(d["frame_index"])
        state = np.stack([np.asarray(x, dtype=np.float32) for x in d["observation.state"]])
        action = np.stack([np.asarray(x, dtype=np.float32) for x in d["action"]])
        keep_mask = np.zeros(len(ep), dtype=bool)
        for e in sorted(set(ep.tolist())):
            m = np.where(ep == e)[0]
            m = m[np.argsort(fi[m])]
            step = np.abs(action[m][:, BODY] - state[m][:, BODY]).max(axis=1)
            drop = int(np.argmax(step > args.threshold)) if (step > args.threshold).any() else 0
            trim_report[e] = (drop, len(m), float(step[drop]))
            keep_mask[m[drop:]] = True
        kept_tables[f] = (tbl, keep_mask)

    total_drop = sum(v[0] for v in trim_report.values())
    total_old = sum(v[1] for v in trim_report.values())
    print(f"\n  source      {src}")
    print(f"  threshold   {args.threshold:g} normalised units\n")
    print("   ep   drop   was ->  now   frame-0 command")
    for e in sorted(trim_report):
        drop, was, cmd = trim_report[e]
        print("   %2d   %4d   %4d -> %4d   %8.2f units" % (e, drop, was, was - drop, cmd))
    print(f"\n  dropping {total_drop} of {total_old} frames "
          f"({100.0 * total_drop / total_old:.1f}%), leaving {total_old - total_drop}")
    if args.dry_run:
        print("\n  --dry-run: nothing written")
        return 0

    # ---- write data, rebasing timestamp/frame_index and renumbering index ----
    dst.mkdir(parents=True)
    new_lengths, new_from_ts, per_ep_arrays = {}, {}, {}
    running = 0
    for f, (tbl, keep) in kept_tables.items():
        d = tbl.to_pydict()
        idx = np.where(keep)[0]
        ep = np.array(d["episode_index"])[idx]
        out = {k: [d[k][i] for i in idx] for k in d}
        ts = np.array(out["timestamp"], dtype=np.float64)
        fi = np.array(out["frame_index"], dtype=np.int64)
        for e in sorted(set(ep.tolist())):
            m = ep == e
            drop = trim_report[e][0]
            fi[m] = fi[m] - drop
            # Derive from the renumbered frame index rather than subtracting, so
            # the invariant timestamp == frame_index / fps holds exactly instead
            # of carrying float32 residue from the shift.
            ts[m] = fi[m] / fps
            new_lengths[e] = int(m.sum())
            new_from_ts[e] = episodes[e]["videos/observation.images.top/from_timestamp"] + drop / fps
        out["timestamp"] = [float(x) for x in ts]
        out["frame_index"] = [int(x) for x in fi]
        out["index"] = list(range(running, running + len(idx)))
        running += len(idx)
        for e in sorted(set(ep.tolist())):
            m = ep == e
            per_ep_arrays[e] = {
                "action": np.stack([np.asarray(a, dtype=np.float64)
                                    for a, keep_it in zip(out["action"], m) if keep_it]),
                "observation.state": np.stack([np.asarray(a, dtype=np.float64)
                                               for a, keep_it in zip(out["observation.state"], m) if keep_it]),
                "timestamp": ts[m], "frame_index": fi[m],
                "episode_index": np.array(out["episode_index"])[m],
                "index": np.array(out["index"])[m],
                "task_index": np.array(out["task_index"])[m],
            }
        rel = f.relative_to(src)
        (dst / rel.parent).mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pydict(out, schema=tbl.schema), dst / rel)
        print(f"  wrote {rel}  ({len(idx)} rows)")

    # ---- meta/episodes ----
    cum = 0
    new_rows = []
    for e in sorted(episodes):
        r = dict(episodes[e])
        r["length"] = new_lengths[e]
        r["dataset_from_index"] = cum
        cum += new_lengths[e]
        r["dataset_to_index"] = cum
        r["videos/observation.images.top/from_timestamp"] = new_from_ts[e]
        for feat in STAT_FEATURES:
            for stat, val in stats_for(per_ep_arrays[e][feat]).items():
                r[f"stats/{feat}/{stat}"] = val
        new_rows.append(r)
    schema = pq.read_table(ep_files[0]).schema
    for f in ep_files:
        rel = f.relative_to(src)
        want = {r["episode_index"] for r in pq.read_table(f).to_pylist()}
        (dst / rel.parent).mkdir(parents=True, exist_ok=True)
        rows = [r for r in new_rows if r["episode_index"] in want]
        pq.write_table(pa.Table.from_pylist(rows, schema=schema), dst / rel)
        print(f"  wrote {rel}  ({len(rows)} episodes)")

    # ---- meta/stats.json: every feature but the images, which ImageNet covers ----
    old_stats = json.loads((src / "meta" / "stats.json").read_text())
    new_stats = dict(old_stats)
    for feat in STAT_FEATURES:
        allv = np.concatenate([per_ep_arrays[e][feat].reshape(len(per_ep_arrays[e][feat]), -1)
                               for e in sorted(per_ep_arrays)], axis=0)
        new_stats[feat] = stats_for(allv)
    (dst / "meta").mkdir(parents=True, exist_ok=True)
    (dst / "meta" / "stats.json").write_text(json.dumps(new_stats, indent=4))

    # ---- info.json ----
    info["total_frames"] = cum
    (dst / "meta" / "info.json").write_text(json.dumps(info, indent=4))

    # ---- carried across untouched ----
    shutil.copy2(src / "meta" / "tasks.parquet", dst / "meta" / "tasks.parquet")
    shutil.copytree(src / "videos", dst / "videos")
    print(f"\n  videos copied unchanged (never re-encoded)")
    print(f"  {info['total_episodes']} episodes, {cum} frames -> {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
