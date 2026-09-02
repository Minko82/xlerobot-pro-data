"""Bottle centroid in start frames (training + trials) and the grasp pose that followed."""
import glob, sys, json, cv2, numpy as np, pandas as pd
root = "/home/xle/.cache/huggingface/lerobot/local/glassbottle_pick_v6"
key = "observation.images.top"

def bottle_xy(bgr):
    # dark blob on a grey shelf; mask out the gripper (bottom-left) and the tape/right edge
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    m = (g < 45).astype(np.uint8)
    m[330:, :260] = 0          # gripper
    m[:, 540:] = 0             # tape and right edge hardware
    m[:15, :] = 0
    n, lab, stats, cents = cv2.connectedComponentsWithStats(m)
    if n < 2: return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[i, cv2.CC_STAT_AREA] < 800: return None
    x, y, w, h = stats[i, :4]
    return dict(cx=float(cents[i][0]), cy=float(cents[i][1]), area=int(stats[i, cv2.CC_STAT_AREA]), w=int(w), h=int(h))

meta = pd.concat([pd.read_parquet(f) for f in sorted(glob.glob(root+"/meta/episodes/**/*.parquet", recursive=True))]).sort_values("episode_index")
data = pd.concat([pd.read_parquet(f) for f in sorted(glob.glob(root+"/data/**/*.parquet", recursive=True))]).sort_values("index")
fi = [c for c in meta.columns if c.startswith("videos/"+key) and c.endswith("file_index")][0]
ci = [c for c in meta.columns if c.startswith("videos/"+key) and c.endswith("chunk_index")][0]
ts = [c for c in meta.columns if c.startswith("videos/"+key) and c.endswith("from_timestamp")][0]
names = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
rows = []
for _, r in meta.iterrows():
    ep = int(r.episode_index)
    path = f"{root}/videos/{key}/chunk-{int(r[ci]):03d}/file-{int(r[fi]):03d}.mp4"
    cap = cv2.VideoCapture(path); cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(r[ts]*cap.get(cv2.CAP_PROP_FPS))))
    ok, fr = cap.read(); cap.release()
    b = bottle_xy(fr) if ok else None
    d = data[data.episode_index == ep]
    a = np.stack(d["action"].values)
    closed = np.where(a[:, 5] < 50)[0]
    g = int(closed[0]) if len(closed) else -1
    row = dict(ep=ep, g=g, **(b or dict(cx=np.nan, cy=np.nan, area=0, w=0, h=0)))
    if g >= 0:
        for j, n in enumerate(names): row["grasp_"+n] = float(a[g, j])
    rows.append(row)
df = pd.DataFrame(rows); df.to_csv("/tmp/bottle_map.csv", index=False)
print(df[["ep","cx","cy","area","grasp_shoulder_pan","grasp_shoulder_lift","grasp_elbow_flex","grasp_wrist_flex"]].round(1).to_string())
print("\nTRIAL START FRAMES")
for f in sys.argv[1:]:
    print(f, bottle_xy(cv2.imread(f)))
