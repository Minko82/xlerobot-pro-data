#!/usr/bin/env python3
"""Copy a LeRobot v3 dataset so every frame of an episode is that episode's first frame.

Why. The camera is fixed, so the only thing in the image that the policy needs
is where the bottle is, and that is already in frame 0. Everything that changes
after frame 0 -- the arm, and in kinesthetic demonstrations the operator's hand
on it -- is what the policy learned to read instead, and none of it looks the
same at deployment (2 September diagnosis: with the training arm-and-hand pixels
pasted onto a live frame the plan is right; with the live arm alone it closes at
once). Freezing the image makes the observation (bottle position, joint state),
which the servo path reproduces to ~1 unit when the plan is right.

Deploy the resulting policy with `act_policy_control.py run --freeze-frame` (and
the same --overlay mask the source dataset was painted with), so the policy sees
one frame from the start pose for the whole run, exactly as in training.

Videos are rebuilt from the decoded first frame of each episode, repeated
`length` times, keeping file layout, frame counts and timestamps. meta/ and
data/ are copied unchanged except for the codec name in info.json.

    python scripts/freeze_dataset_videos.py SRC_DIR DST_DIR
"""
from __future__ import annotations
import argparse, json, shutil, subprocess, sys
from pathlib import Path
import av, numpy as np, pandas as pd

KEY = "observation.images.top"


def nb_frames(p: Path) -> int:
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v", "-count_frames",
                          "-show_entries", "stream=nb_read_frames", "-of", "csv=p=0", str(p)],
                         capture_output=True, text=True, check=True).stdout.strip()
    return int(out)


def decode_first_frames(path: Path, starts: dict[int, int]) -> dict[int, np.ndarray]:
    """starts: frame number -> episode index. Returns episode -> RGB frame."""
    got = {}
    c = av.open(str(path)); s = c.streams.video[0]
    n = 0
    for fr in c.decode(s):
        if n in starts:
            got[starts[n]] = fr.to_ndarray(format="rgb24")
        n += 1
        if n > max(starts):
            break
    c.close()
    return got


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src", type=Path); ap.add_argument("dst", type=Path)
    ap.add_argument("--crf", type=int, default=18)
    ap.add_argument("--codec", default="libx264")
    ap.add_argument("--gop", type=int, default=30, help="Keyframe interval. Frames within an episode are identical, so a long GOP costs nothing to decode and shrinks the file ~10x.")
    a = ap.parse_args()
    if a.dst.exists():
        sys.exit(f"{a.dst} exists; refusing to overwrite")
    a.dst.mkdir(parents=True)
    for sub in ("meta", "data"):
        shutil.copytree(a.src / sub, a.dst / sub)
    info = json.load(open(a.src / "meta/info.json"))
    fps = int(info["fps"])
    meta = pd.concat([pd.read_parquet(f) for f in sorted((a.src / "meta/episodes").rglob("*.parquet"))])
    fi, ci, ts = (f"videos/{KEY}/{k}" for k in ("file_index", "chunk_index", "from_timestamp"))
    files = sorted(set(zip(meta[ci].astype(int), meta[fi].astype(int))))
    total = 0
    for chunk, fidx in files:
        rel = Path(f"videos/{KEY}/chunk-{chunk:03d}/file-{fidx:03d}.mp4")
        src, dst = a.src / rel, a.dst / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        eps = meta[(meta[ci] == chunk) & (meta[fi] == fidx)].sort_values(ts)
        starts = {int(round(float(r[ts]) * fps)): int(r.episode_index) for _, r in eps.iterrows()}
        first = decode_first_frames(src, starts)
        h, w = next(iter(first.values())).shape[:2]
        cmd = ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
               "-r", str(fps), "-i", "-", "-c:v", a.codec, "-preset", "fast", "-crf", str(a.crf),
               "-g", str(a.gop), "-pix_fmt", "yuv420p", "-an", str(dst)]
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        n_written = 0
        for _, r in eps.iterrows():
            frame = first[int(r.episode_index)].tobytes()
            for _ in range(int(r.length)):
                p.stdin.write(frame); n_written += 1
        p.stdin.close(); p.wait()
        if p.returncode:
            sys.exit(f"ffmpeg failed on {rel}")
        n_src, n_dst = nb_frames(src), nb_frames(dst)
        print(f"  {rel}  episodes {list(starts.values())}  {n_src} -> {n_dst} frames  "
              f"{src.stat().st_size/1e6:.1f} -> {dst.stat().st_size/1e6:.2f} MB", flush=True)
        if not (n_src == n_dst == n_written):
            sys.exit(f"frame count mismatch on {rel}: src {n_src} dst {n_dst} written {n_written}")
        total += n_dst
        # verify: decoded frames inside each episode equal that episode's first frame
        c = av.open(str(dst)); s = c.streams.video[0]; n = 0; worst = 0.0
        starts_sorted = sorted(starts.items())
        for fr in c.decode(s):
            ep_i = max(e for st, e in starts_sorted if st <= n)
            if n % 37 == 0:
                d = np.abs(fr.to_ndarray(format="rgb24").astype(int) - first[ep_i].astype(int))
                worst = max(worst, float(d.mean()))
            n += 1
        c.close()
        print(f"      verify: worst mean |decoded - frame0| {worst:.2f} over sampled frames")
    info["features"][KEY]["info"]["video.codec"] = "h264" if a.codec == "libx264" else a.codec
    json.dump(info, open(a.dst / "meta/info.json", "w"), indent=4)
    print(f"done: {info['total_episodes']} episodes, {total} frames (info.json says {info['total_frames']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
