#!/usr/bin/env python3
"""Offline evaluation of an ACT checkpoint: no motors, no camera, code only.

Three views of one checkpoint, each answering a question the hardware trials of
1 September could not answer cleanly:

1. TRAINING-FRAME BIAS   From frames 90/60/30 before each recorded grasp, where does
   the policy plan to close, and when?  Slope ~1 with a non-zero intercept is a
   constant aim bias.  Predicted close step vs the true one is the dwell handling.
2. CHUNK ERROR           Mean |predicted - recorded| over the 100-step chunk on
   random frames, per joint.  A plain fit-quality number to compare checkpoints.
3. LIVE FRAMES           Frames the wrist camera actually saw during trials, with
   the joint state at that moment, from `trials/live_frames_manifest.json`.  Where
   does the policy plan to close from each?  For frames whose bottle position is
   known, the operator's grasp pose from the bottle->grasp regression is printed
   beside it; for trial 9 the pose that really grasped the bottle is.

`--mask IMAGE` pastes region `--region` of IMAGE onto every live frame first (the
grey mask a hand-free policy trained with; or a hand patch).  `--invariance`
additionally re-runs each live frame with the region filled from a second image
and reports the largest joint difference -- how much the policy still reads that
region.

    python scripts/eval_offline.py CKPT --repo-id local/glassbottle_pick_v7_masked \
        --manifest trials/live_frames_manifest.json --mask calibration/hand_mask_grey88.png \
        --invariance calibration/hand_patch_ep13_g-120.png --out trials/eval_v7_040000.md
"""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
import numpy as np, torch, cv2
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import prepare_observation_for_inference
from lerobot.datasets.lerobot_dataset import LeRobotDataset

TASK = "pick up the glass bottle from the shelf"
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
GI = 5

class Policy:
    def __init__(self, ckpt: Path):
        cfg = PreTrainedConfig.from_pretrained(ckpt); cfg.pretrained_path = str(ckpt)
        self.policy = ACTPolicy.from_pretrained(ckpt).eval().to("cuda")
        self.pre, self.post = make_pre_post_processors(cfg, pretrained_path=str(ckpt))
        self.dev = torch.device("cuda")
    def chunk(self, rgb_u8: np.ndarray, state: np.ndarray) -> np.ndarray:
        obs = {"observation.images.top": rgb_u8.astype(np.uint8), "observation.state": state.astype(np.float32)}
        with torch.inference_mode():
            o = prepare_observation_for_inference(obs, self.dev, TASK, "so101_follower")
            ch = self.policy.predict_action_chunk(self.pre(o))[0]
            try: un = self.post(ch)
            except Exception: un = torch.stack([self.post(ch[i:i+1])[0] for i in range(ch.shape[0])])
        return np.asarray(un.detach().cpu()).reshape(-1, 6)

def close_step(ch): 
    k = np.where(ch[:, GI] < 50)[0]; return int(k[0]) if len(k) else None
def onset_step(ch, state, thr=4.0):
    # Upward shoulder_lift motion only: the first planned command can sit a few units
    # below the -100 floor (it is clamped downstream), which is not a motion.
    k = np.where(ch[:, 1] - state[1] > thr)[0]; return int(k[0]) if len(k) else None
def item_rgb(ds, idx):
    it = ds[idx]; return (it["observation.images.top"].permute(1, 2, 0) * 255).round().clamp(0, 255).to(torch.uint8).numpy(), it["observation.state"].numpy()

def eval_dataset(P, ds, n_chunk=150, seed=0):
    hf = ds.hf_dataset.with_format(None)
    acts = np.stack(hf["action"]).astype(np.float32); eps = np.asarray(hf["episode_index"])
    rows, closes, onsets = [], [], []
    for ep in sorted(set(eps.tolist())):
        idxs = np.where(eps == ep)[0]; a = acts[idxs]
        c = np.where(a[:, GI] < 50)[0]
        if len(c) == 0: continue
        g = int(c[0])
        # start-frame behaviour: onset of shoulder_lift motion, predicted vs recorded
        img, st = item_rgb(ds, idxs[0]); ch = P.chunk(img, st)
        true_on = onset_step(a, a[0]); onsets.append((onset_step(ch, st), true_on))
        for off in (90, 60, 30):
            t = g - off
            if t < 0: continue
            img, st = item_rgb(ds, idxs[t]); ch = P.chunk(img, st)
            k = close_step(ch); closes.append((off, k))
            if k is not None: rows.append((ch[k], a[g]))
    out = {}
    if rows:
        Pm = np.array([r[0] for r in rows]); T = np.array([r[1] for r in rows])
        for j, n in enumerate(NAMES[:4]):
            sl, ic = np.polyfit(T[:, j], Pm[:, j], 1); e = Pm[:, j] - T[:, j]
            out[n] = dict(slope=float(sl), intercept=float(ic), mean_err=float(e.mean()), mean_abs=float(np.abs(e).mean()))
    out["close"] = {off: dict(n=len([k for o, k in closes if o == off and k is not None]),
                              never=len([k for o, k in closes if o == off and k is None]),
                              mean_pred=float(np.mean([k for o, k in closes if o == off and k is not None])) if any(o == off and k is not None for o, k in closes) else None)
                    for off in (90, 60, 30)}
    on_ok = [(p, t) for p, t in onsets if p is not None and t is not None]
    out["onset"] = dict(pred_mean=float(np.mean([p for p, _ in on_ok])) if on_ok else None,
                        true_mean=float(np.mean([t for _, t in on_ok])) if on_ok else None,
                        never=len([1 for p, _ in onsets if p is None]), n=len(onsets))
    rng = np.random.default_rng(seed); n_frames = len(acts); errs = []
    for idx in rng.choice(n_frames - 100, size=min(n_chunk, n_frames - 100), replace=False):
        if len(set(eps[idx:idx+100].tolist())) != 1: continue
        img, st = item_rgb(ds, int(idx)); ch = P.chunk(img, st)
        errs.append(np.abs(ch - acts[idx:idx+100]).mean(axis=0))
    E = np.array(errs); out["chunk_l1"] = {n: float(E[:, j].mean()) for j, n in enumerate(NAMES)}; out["chunk_n"] = len(errs)
    return out

def paste(rgb, src, region):
    y0, y1, x0, x1 = region; out = rgb.copy(); out[y0:y1, x0:x1] = src[y0:y1, x0:x1]; return out

def eval_live(P, manifest, root: Path, mask, inv, region):
    res = []
    for e in manifest:
        rgb = cv2.cvtColor(cv2.imread(str(root / e["frame"])), cv2.COLOR_BGR2RGB)
        st = np.array(e["state"], np.float32)
        img = paste(rgb, mask, region) if mask is not None else rgb
        ch = P.chunk(img, st); k = close_step(ch); pose = ch[k] if k is not None else ch[-1]
        r = dict(name=e["name"], close=k, pose=pose[:4].round(1).tolist(), onset=onset_step(ch, st),
                 expected=e.get("expected"), note=e.get("note", ""))
        if inv is not None:
            ch2 = P.chunk(paste(rgb, inv, region), st); k2 = close_step(ch2); pose2 = ch2[k2] if k2 is not None else ch2[-1]
            r["inv_close"] = k2; r["inv_max_joint_diff"] = float(np.abs(pose2[:4] - pose[:4]).max())
        res.append(r)
    return res

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--repo-id", default=None); ap.add_argument("--manifest", type=Path, default=None)
    ap.add_argument("--root", type=Path, default=Path("."), help="Directory the manifest's frame paths are relative to.")
    ap.add_argument("--mask", type=Path, default=None); ap.add_argument("--invariance", type=Path, default=None)
    ap.add_argument("--region", default="200,480,0,260"); ap.add_argument("--n-chunk", type=int, default=150)
    ap.add_argument("--out", type=Path, default=None); ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    region = tuple(int(v) for v in a.region.split(","))
    t0 = time.time(); P = Policy(a.checkpoint)
    rd = lambda p: cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB) if p else None
    mask, inv = rd(a.mask), rd(a.invariance)
    report = {"checkpoint": str(a.checkpoint), "mask": str(a.mask), "region": region}
    L = [f"# Offline evaluation — `{a.checkpoint}`", "", f"mask: `{a.mask}`  region y{region[0]}:{region[1]} x{region[2]}:{region[3]}", ""]
    if a.repo_id:
        ds = LeRobotDataset(a.repo_id); d = eval_dataset(P, ds, a.n_chunk); report["dataset"] = d
        L += [f"## Training frames — `{a.repo_id}` ({ds.num_episodes} ep, {ds.num_frames} frames)", "",
              "Planned grasp pose vs recorded, from 90/60/30 frames before the grasp:", "",
              "| joint | slope | intercept | mean err | mean abs |", "|---|---|---|---|---|"]
        L += [f"| {n} | {d[n]['slope']:.3f} | {d[n]['intercept']:+.2f} | {d[n]['mean_err']:+.2f} | {d[n]['mean_abs']:.2f} |" for n in NAMES[:4]]
        L += ["", "| from | planned close step (true) | never closes |", "|---|---|---|"]
        L += [f"| g-{off} | {c['mean_pred']:.0f} ({off}) | {c['never']}/{c['n']+c['never']} |" if c['mean_pred'] is not None else f"| g-{off} | — | {c['never']} |" for off, c in d["close"].items()]
        o = d["onset"]; L += ["", f"Start-frame motion onset: planned {o['pred_mean']:.0f} vs recorded {o['true_mean']:.0f} frames (never moves in chunk: {o['never']}/{o['n']})" if o["pred_mean"] is not None else f"Start-frame onset: never moves in {o['never']}/{o['n']}", ""]
        L += [f"Chunk L1 error over 100 steps, {d['chunk_n']} random frames:", "", "| " + " | ".join(NAMES) + " |", "|" + "---|" * 6,
              "| " + " | ".join(f"{d['chunk_l1'][n]:.2f}" for n in NAMES) + " |", ""]
    if a.manifest:
        man = json.load(open(a.manifest)); live = eval_live(P, man, a.root, mask, inv, region); report["live"] = live
        L += ["## Live frames (from trials)", "", "| frame | planned close step | onset | planned pose pan/lift/elbow/wflex | reference pose | region sensitivity | note |", "|---|---|---|---|---|---|---|"]
        for r in live:
            exp = r["expected"]; exp_s = "/".join(f"{v:+.0f}" for v in exp) if exp else "—"
            sens = f"{r['inv_max_joint_diff']:.1f} (close {r['inv_close']})" if "inv_max_joint_diff" in r else "—"
            L.append(f"| {r['name']} | {r['close']} | {r['onset']} | {'/'.join(f'{v:+.1f}' for v in r['pose'])} | {exp_s} | {sens} | {r['note']} |")
        L.append("")
    L.append(f"_{time.time()-t0:.0f} s on cuda_")
    text = "\n".join(L); print(text)
    if a.out: a.out.write_text(text)
    if a.json: a.json.write_text(json.dumps(report, indent=1, default=float))

if __name__ == "__main__":
    sys.exit(main())
