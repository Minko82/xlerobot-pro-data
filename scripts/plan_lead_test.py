"""(1) Training frames whose state matches trial 7 step 120; dump them. (2) Does the policy's plan run ahead of the demonstration?"""
import sys, glob, cv2, numpy as np, pandas as pd, torch
sys.path.insert(0, "/home/xle/xlerobot-pro-data/scripts")
from eval_offline import Policy, close_step, item_rgb
from lerobot.datasets.lerobot_dataset import LeRobotDataset
root = "/home/xle/.cache/huggingface/lerobot/local/glassbottle_pick_v6"
d = pd.concat([pd.read_parquet(f) for f in sorted(glob.glob(root+"/data/**/*.parquet", recursive=True))]).sort_values("index")
S = np.stack(d["observation.state"].values); A = np.stack(d["action"].values); E = d.episode_index.values
q = np.array([-11.58, 6.57, 71.38, -33.67])
ds6 = LeRobotDataset("local/glassbottle_pick_v6")
for ep in (0, 40, 13):
    idx = np.where(E == ep)[0]; dist = np.abs(S[idx, :4] - q).sum(1); j = idx[int(np.argmin(dist))]
    g = idx[0] + int(np.where(A[idx][:, 5] < 50)[0][0])
    img, st = item_rgb(ds6, int(j)); cv2.imwrite(f"/tmp/match_ep{ep:02d}.jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    print(f"ep{ep}: nearest frame {j-idx[0]} (L1 dist {dist.min():.1f}) state {np.round(S[j,:4],1)}  frames to grasp {g-j}  grasp pose {np.round(A[g,:4],1)}")
# (2) lead test: approach frames between onset and grasp
def onset(a): k = np.where(a[:, 1] - a[0, 1] > 4)[0]; return int(k[0]) if len(k) else None
rng = np.random.default_rng(1); picks = []
for ep in np.unique(E):
    idx = np.where(E == ep)[0]; a = A[idx]; c = np.where(a[:, 5] < 50)[0]; o = onset(a)
    if len(c) == 0 or o is None: continue
    g = int(c[0])
    for t in rng.integers(o, max(o+1, g-40), size=2): picks.append((int(idx[t]), int(idx[0]+g), int(t), g))
for label, ck, repo in (("v6 040000", "outputs/act_glassbottle_pick_v6/checkpoints/040000/pretrained_model", "local/glassbottle_pick_v6"),
                        ("v7 010000", "outputs/act_glassbottle_pick_v7/checkpoints/010000/pretrained_model", "local/glassbottle_pick_v7_masked")):
    P = Policy(ck); ds = LeRobotDataset(repo); leads, tclose = [], []
    for fi, gi, t, g in picks:
        img, st = item_rgb(ds, fi); ch = P.chunk(img, st); n = min(100, gi - fi)
        true = A[fi:fi+n]
        leads.append((ch[:n, 1] - true[:, 1]).mean()); k = close_step(ch)
        tclose.append((k if k is not None else 100, g - t))
    leads = np.array(leads); tc = np.array(tclose)
    print(f"\n{label}: {len(picks)} approach frames")
    print(f"  shoulder_lift lead over the chunk (pred - true, + = plan ahead of demo): mean {leads.mean():+.2f}  median {np.median(leads):+.2f}  sd {leads.std():.2f}")
    print(f"  planned close step vs true frames-to-grasp: planned median {np.median(tc[:,0]):.0f}  true median {np.median(tc[:,1]):.0f}   (never closes: {(tc[:,0]==100).sum()})")
    m = tc[:, 0] < 100; print(f"  where the plan closes within the chunk: planned {tc[m,0].mean():.0f} vs true {tc[m,1].mean():.0f} frames  (n={m.sum()})")
