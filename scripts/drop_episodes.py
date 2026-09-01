#!/usr/bin/env python3
"""Remove whole episodes from a dataset, into a new one.

Why this has to exist. `lerobot-train --dataset.episodes=[...]` does NOT exclude
episodes from a local dataset. It sets `num_episodes` for the log line and picks
which files to fetch from the Hub, but `load_hf_dataset()` reads every parquet
under `data/` with no filter, `__len__` returns that full length, and
`__getitem__` indexes it directly. Passing the flag and watching the log report
the episode count you asked for is not evidence that anything was excluded --
check `dataset.num_frames` against the frames you expect.

glassbottle_pick_v5 episode 5 is what prompted this: a 45 s take that hit the
time cap with the gripper never actuated, 1346 frames of the arm moving with the
jaws held open at 94.5. `goto_start_pose.py` opens the jaws to 94.4 before every
trial, so deployment begins in exactly the state that episode occupies for its
whole length -- the worst possible 6% of a dataset to leave in.

What this does NOT need to do, unlike trim_start_dead_frames.py: surviving
episodes are untouched, so their frame_index, timestamp, video from_timestamp and
per-episode statistics all carry across unchanged. Only three things move --
`episode_index` is renumbered to stay contiguous from zero (lerobot indexes
`meta.episodes` positionally, so a gap breaks lookups), the global `index` column
is renumbered, and `dataset_from_index`/`dataset_to_index` are recomputed.

A file that ends up holding no episodes is not written at all. That is not a
corner case: `--resume` commits one file per batch, so a batch that recorded a
single episode gives that episode a file to itself, and dropping it empties both
the data file and the meta file. A zero-row parquet is valid on disk but pyarrow
refuses to read it back ("BatchSize must be greater than 0"), which breaks the
whole dataset rather than the one file. Gaps in the file numbering are harmless:
`load_nested_dataset` globs the directory, and the meta rows' `data/file_index`
still resolves for every surviving episode.

The video is never re-encoded. Dropped episodes' frames stay in the file and
simply stop being referenced.
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
    p.add_argument("--drop", type=int, nargs="+", required=True, metavar="EP",
                   help="Episode indices to remove, as numbered in --src.")
    p.add_argument("--dry-run", action="store_true", help="Report and write nothing.")
    args = p.parse_args()

    src, dst = args.src, args.dst
    drop = set(args.drop)
    if not (src / "meta" / "info.json").exists():
        raise SystemExit(f"{src} does not look like a LeRobot dataset")
    if dst.exists() and not args.dry_run:
        raise SystemExit(f"{dst} already exists -- refusing to overwrite")

    info = json.loads((src / "meta" / "info.json").read_text())
    data_files = sorted((src / "data").glob("chunk-*/file-*.parquet"))
    ep_files = sorted((src / "meta" / "episodes").glob("chunk-*/file-*.parquet"))

    episodes = {}
    for f in ep_files:
        for r in pq.read_table(f).to_pylist():
            episodes[r["episode_index"]] = r

    missing = drop - set(episodes)
    if missing:
        raise SystemExit(f"episodes {sorted(missing)} are not in {src}")

    keep_eps = [e for e in sorted(episodes) if e not in drop]
    renumber = {old: new for new, old in enumerate(keep_eps)}

    print(f"\n  source   {src}")
    print(f"  dropping episodes {sorted(drop)}\n")
    print("   ep   frames   fate")
    total_old = total_kept = 0
    for e in sorted(episodes):
        n = episodes[e]["length"]
        total_old += n
        if e in drop:
            print("   %2d   %5d   DROPPED" % (e, n))
        else:
            total_kept += n
            print("   %2d   %5d   -> ep %d" % (e, n, renumber[e]))
    print(f"\n  {len(keep_eps)} of {len(episodes)} episodes, "
          f"{total_kept} of {total_old} frames "
          f"({100.0 * (total_old - total_kept) / total_old:.1f}% removed)")
    if args.dry_run:
        print("\n  --dry-run: nothing written")
        return 0

    # ---- data files: filter rows, renumber episode_index and the global index ----
    dst.mkdir(parents=True)
    running = 0
    per_ep_arrays = {}
    for f in data_files:
        tbl = pq.read_table(f)
        d = tbl.to_pydict()
        ep = np.array(d["episode_index"])
        keep = np.array([e not in drop for e in ep.tolist()])
        idx = np.where(keep)[0]
        out = {k: [d[k][i] for i in idx] for k in d}
        out["episode_index"] = [renumber[e] for e in np.array(ep)[idx].tolist()]
        out["index"] = list(range(running, running + len(idx)))
        running += len(idx)
        for old_e in sorted(set(np.array(ep)[idx].tolist())):
            m = np.array(out["episode_index"]) == renumber[old_e]
            per_ep_arrays[renumber[old_e]] = {
                "action": np.stack([np.asarray(a, dtype=np.float64)
                                    for a, k in zip(out["action"], m) if k]),
                "observation.state": np.stack([np.asarray(a, dtype=np.float64)
                                               for a, k in zip(out["observation.state"], m) if k]),
                "timestamp": np.array(out["timestamp"])[m],
                "frame_index": np.array(out["frame_index"])[m],
                "episode_index": np.array(out["episode_index"])[m],
                "index": np.array(out["index"])[m],
                "task_index": np.array(out["task_index"])[m],
            }
        rel = f.relative_to(src)
        if len(idx) == 0:
            print(f"  SKIPPED {rel}  (every episode in it was dropped)")
            continue
        (dst / rel.parent).mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pydict(out, schema=tbl.schema), dst / rel)
        print(f"  wrote {rel}  ({len(idx)} rows)")

    # ---- meta/episodes: drop rows, renumber, recompute the index spans ----
    # Per-episode stats are copied verbatim: the episodes themselves are
    # unmodified, so recomputing them would only introduce float drift.
    cum = 0
    new_rows = {}
    for e in keep_eps:
        r = dict(episodes[e])
        r["episode_index"] = renumber[e]
        r["dataset_from_index"] = cum
        cum += r["length"]
        r["dataset_to_index"] = cum
        for feat in ("episode_index", "index"):
            for stat, val in stats_for(per_ep_arrays[renumber[e]][feat]).items():
                r[f"stats/{feat}/{stat}"] = val
        new_rows[e] = r
    schema = pq.read_table(ep_files[0]).schema
    for f in ep_files:
        rel = f.relative_to(src)
        want = {r["episode_index"] for r in pq.read_table(f).to_pylist()}
        rows = [new_rows[e] for e in keep_eps if e in want]
        if not rows:
            print(f"  SKIPPED {rel}  (every episode in it was dropped)")
            continue
        (dst / rel.parent).mkdir(parents=True, exist_ok=True)
        pq.write_table(pa.Table.from_pylist(rows, schema=schema), dst / rel)
        print(f"  wrote {rel}  ({len(rows)} episodes)")

    # ---- meta/stats.json over the surviving episodes ----
    old_stats = json.loads((src / "meta" / "stats.json").read_text())
    new_stats = dict(old_stats)
    for feat in STAT_FEATURES:
        allv = np.concatenate([per_ep_arrays[e][feat].reshape(len(per_ep_arrays[e][feat]), -1)
                               for e in sorted(per_ep_arrays)], axis=0)
        new_stats[feat] = stats_for(allv)
    (dst / "meta").mkdir(parents=True, exist_ok=True)
    (dst / "meta" / "stats.json").write_text(json.dumps(new_stats, indent=4))

    info["total_episodes"] = len(keep_eps)
    info["total_frames"] = cum
    (dst / "meta" / "info.json").write_text(json.dumps(info, indent=4))

    shutil.copy2(src / "meta" / "tasks.parquet", dst / "meta" / "tasks.parquet")
    shutil.copytree(src / "videos", dst / "videos")
    print(f"\n  videos copied unchanged (never re-encoded)")
    print(f"  {len(keep_eps)} episodes, {cum} frames -> {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
