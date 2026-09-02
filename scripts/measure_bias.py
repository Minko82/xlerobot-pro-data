#!/usr/bin/env python3
"""Per-joint bias of an ACT checkpoint against its own training frames.

For every episode: find the grasp frame g (first frame where the recorded gripper
command drops below 50), then run the policy on frames g-90, g-60, g-30 and compare
the predicted chunk against the recorded actions over the same horizon. Reports,
per joint, the signed error at the predicted grasp moment (where predicted gripper
first < 50 within the chunk) versus the recorded grasp pose, plus slope/intercept
of predicted-vs-true. Slope ~1 with a non-zero intercept = constant aim bias.
"""
import sys, numpy as np, torch
from pathlib import Path
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ckpt = Path(sys.argv[1]); repo = sys.argv[2]
task = "pick up the glass bottle from the shelf"
cfg = PreTrainedConfig.from_pretrained(ckpt); cfg.pretrained_path = str(ckpt)
policy = ACTPolicy.from_pretrained(ckpt).eval()
pre, post = make_pre_post_processors(cfg, pretrained_path=str(ckpt))
dev = torch.device("cuda")
policy.to(dev)
ds = LeRobotDataset(repo)
names = ds.meta.features["action"]["names"]; J = len(names); GI = names.index("gripper.pos")
print("frames", ds.num_frames, "episodes", ds.num_episodes)

hf = ds.hf_dataset.with_format(None)
acts = np.stack(hf["action"]).astype(np.float32)
eps = np.asarray(hf["episode_index"])

def predict_chunk(idx):
    item = ds[idx]
    img = (item["observation.images.top"].permute(1, 2, 0) * 255).round().clamp(0, 255).to(torch.uint8).numpy()
    obs = {"observation.images.top": img, "observation.state": item["observation.state"].numpy().astype(np.float32)}
    with torch.inference_mode():
        obs = prepare_observation_for_inference(obs, dev, task, "so101_follower")
        obs = pre(obs)
        chunk = policy.predict_action_chunk(obs)[0]          # (100, J) normalised
        out = post({"action": chunk}) if False else None
        # unnormalise via the postprocessor one action at a time is slow; do it batched:
        try:
            un = post(chunk)                                  # PolicyAction = tensor
        except Exception:
            un = torch.stack([post(chunk[i:i+1])[0] for i in range(chunk.shape[0])])
    return np.asarray(un.detach().cpu()).reshape(-1, J)

rows = []   # (ep, offset, joint, pred_grasp, true_grasp)
step0 = []  # (pred0 - true0) per joint
for ep in range(ds.num_episodes):
    idxs = np.where(eps == ep)[0]
    a = acts[idxs]
    closed = np.where(a[:, GI] < 50)[0]
    if len(closed) == 0:
        continue
    g = closed[0]
    true_grasp = a[g]
    for off in (90, 60, 30):
        t = g - off
        if t < 0:
            continue
        ch = predict_chunk(idxs[t])
        step0.append(ch[0] - a[t])
        pc = np.where(ch[:, GI] < 50)[0]
        if len(pc) == 0:
            rows.append((ep, off, None, None, None)); continue
        k = pc[0]
        rows.append((ep, off, k, ch[k], true_grasp))
    print(f"  ep {ep:2d} grasp frame {g:3d}  done", end="\r", flush=True)
print()

step0 = np.array(step0)
print("\nSTEP-0 PREDICTION vs RECORDED ACTION, approach frames (n=%d)" % len(step0))
print("  %-16s %8s %8s" % ("joint", "mean err", "mean|e|"))
for j, n in enumerate(names):
    print("  %-16s %+8.2f %8.2f" % (n[:-4], step0[:, j].mean(), np.abs(step0[:, j]).mean()))

good = [r for r in rows if r[2] is not None]
miss = [r for r in rows if r[2] is None]
print("\nPREDICTED GRASP POSE vs RECORDED GRASP POSE (n=%d chunks; %d chunks never closed)" % (len(good), len(miss)))
for off in (90, 60, 30):
    sub = [r for r in good if r[1] == off]
    if not sub: continue
    ks = np.array([r[2] for r in sub]); print(f"  from g-{off}: predicted close at chunk step {ks.mean():.0f} (true {off})  n={len(sub)}")
P = np.array([r[3] for r in good]); T = np.array([r[4] for r in good])
print("  %-16s %6s %10s %9s %8s" % ("joint", "slope", "intercept", "mean err", "mean|e|"))
for j, n in enumerate(names):
    if n[:-4] == "gripper": continue
    sl, ic = np.polyfit(T[:, j], P[:, j], 1)
    e = P[:, j] - T[:, j]
    print("  %-16s %6.3f %+10.2f %+9.2f %8.2f" % (n[:-4], sl, ic, e.mean(), np.abs(e).mean()))
print("\n  positive mean err = policy commands MORE than the operator did at the grasp.")
print("  --aim-offset should be the NEGATIVE of mean err for the reach joints.")
