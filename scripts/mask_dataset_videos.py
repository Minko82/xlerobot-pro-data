#!/usr/bin/env python3
"""Copy a LeRobot v3 dataset with a fixed image region painted out of every video frame.

Why: kinesthetic demonstrations leave the operator's hand on the wrist, inside the
wrist camera's view, in every approach and grasp frame. A policy trained on them
learns to read the hand; at deployment the hand is absent and the policy closes
early and short (glassbottle_pick_v6, checkpoint 040000, trials 6-8). Painting the
region a constant colour in training AND at deployment removes the cue.

Region default matches act_policy_control.py --overlay-region: y 200:480, x 0:260,
the gripper body and hand at the bottom-left, leaving the moving jaw visible.

Videos are re-encoded with the same codec family (AV1, yuv420p, 30 fps, keyframe
every 2 frames as lerobot writes them). Frame counts are verified to match the
source file for file. meta/ and data/ are copied unchanged, then repo_id is not
stored in info.json so nothing else needs editing.

    python scripts/mask_dataset_videos.py SRC_DIR DST_DIR [--region 200,480,0,260] [--grey 90]
"""
import argparse, json, shutil, subprocess, sys
from pathlib import Path

def nb_frames(p: Path) -> int:
    out = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v", "-count_frames",
                          "-show_entries", "stream=nb_read_frames", "-of", "csv=p=0", str(p)],
                         capture_output=True, text=True, check=True).stdout.strip()
    return int(out)

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("src", type=Path); ap.add_argument("dst", type=Path)
    ap.add_argument("--region", default="200,480,0,260", help="Y0,Y1,X0,X1")
    ap.add_argument("--grey", type=int, default=90)
    ap.add_argument("--crf", type=int, default=30)
    a = ap.parse_args()
    y0, y1, x0, x1 = (int(v) for v in a.region.split(","))
    if a.dst.exists():
        sys.exit(f"{a.dst} exists; refusing to overwrite")
    a.dst.mkdir(parents=True)
    for sub in ("meta", "data"):
        shutil.copytree(a.src / sub, a.dst / sub)
    colour = f"0x{a.grey:02x}{a.grey:02x}{a.grey:02x}"
    vids = sorted((a.src / "videos").rglob("*.mp4"))
    for i, v in enumerate(vids, 1):
        out = a.dst / v.relative_to(a.src); out.parent.mkdir(parents=True, exist_ok=True)
        cmd = ["ffmpeg", "-v", "error", "-y", "-i", str(v),
               "-vf", f"drawbox=x={x0}:y={y0}:w={x1-x0}:h={y1-y0}:color={colour}@1:t=fill",
               "-fps_mode", "passthrough", "-c:v", "libsvtav1", "-preset", "6", "-crf", str(a.crf),
               "-g", "2", "-pix_fmt", "yuv420p", "-an", str(out)]
        subprocess.run(cmd, check=True)
        n_src, n_dst = nb_frames(v), nb_frames(out)
        print(f"  [{i}/{len(vids)}] {v.relative_to(a.src)}  {n_src} -> {n_dst} frames  "
              f"{v.stat().st_size/1e6:.1f} -> {out.stat().st_size/1e6:.1f} MB", flush=True)
        if n_src != n_dst:
            sys.exit(f"frame count changed on {v}; aborting")
    info = json.load(open(a.dst / "meta/info.json"))
    print("done:", info["total_episodes"], "episodes,", info["total_frames"], "frames")
    return 0

if __name__ == "__main__":
    sys.exit(main())
