#!/usr/bin/env python3
"""Where does the policy plan to close, as a function of how far through the reach it is?

`eval_offline.py` scores the plan from frames 90/60/30 before the recorded close.
The demonstrations dwell a median 76 frames at the bottle before closing, so
those frames are mostly *after arrival*: the plan only has to hold still and
close, and it scores well.  That never tested the frames that matter at
deployment -- the ones taken mid-reach, when the policy has to decide how far to
go with the arm still on its way.

This script samples each episode's approach at fixed fractions of the way from
motion onset to arrival (first frame within ARRIVE units of the recorded grasp
pose), plus the start frame and the middle of the dwell, and records for each:

    planned close step, planned grasp pose (pose at the planned close, or the
    end of the chunk if it never closes), the recorded grasp pose, the recorded
    frames-to-close, and the joint state at that frame.

Variants re-run the same frames with one thing changed, to measure sensitivity:

    grip57     gripper state set to 57.4 (what the runner holds it at; the
               demonstrations relax to 60.3 with torque off)
    bright     image gain 1.25 (daylight-ish)
    dark       image gain 0.8

Live frames (from the trial logs) are scored the same way when a manifest is
given.  Output is JSON; summarise with `summarise_midreach.py`.

    python scripts/eval_midreach.py CKPT --repo-id local/glassbottle_pick_v7_masked \
        --json trials/midreach_v7_040000.json
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
ARRIVE = 3.0          # units, all four reach joints, to count as "arrived at the grasp pose"
FRACS = (0.0, 0.25, 0.5, 0.75, 1.0)


class Policy:
    def __init__(self, ckpt: Path, device="cuda"):
        cfg = PreTrainedConfig.from_pretrained(ckpt); cfg.pretrained_path = str(ckpt)
        self.policy = ACTPolicy.from_pretrained(ckpt).eval().to(device)
        self.pre, self.post = make_pre_post_processors(cfg, pretrained_path=str(ckpt))
        self.dev = torch.device(device)

    def chunk(self, rgb_u8: np.ndarray, state: np.ndarray) -> np.ndarray:
        obs = {"observation.images.top": np.ascontiguousarray(rgb_u8, dtype=np.uint8),
               "observation.state": state.astype(np.float32)}
        with torch.inference_mode():
            o = prepare_observation_for_inference(obs, self.dev, TASK, "so101_follower")
            ch = self.policy.predict_action_chunk(self.pre(o))[0]
            try:
                un = self.post(ch)
            except Exception:
                un = torch.stack([self.post(ch[i:i + 1])[0] for i in range(ch.shape[0])])
        return np.asarray(un.detach().cpu()).reshape(-1, 6)


def close_step(ch):
    k = np.where(ch[:, GI] < 50)[0]
    return int(k[0]) if len(k) else None


def item_rgb(ds, idx):
    it = ds[idx]
    img = (it["observation.images.top"].permute(1, 2, 0) * 255).round().clamp(0, 255).to(torch.uint8).numpy()
    return img, it["observation.state"].numpy()


def gain(img, g):
    return np.clip(img.astype(np.float32) * g, 0, 255).astype(np.uint8)


VARIANTS = {
    "base": lambda img, st: (img, st),
    "grip57": lambda img, st: (img, np.r_[st[:5], 57.4].astype(np.float32)),
    "bright": lambda img, st: (gain(img, 1.25), st),
    "dark": lambda img, st: (gain(img, 0.8), st),
}


def score(P, img, st, true_grasp, frames_to_close, variants):
    out = {}
    for name in variants:
        im, s = VARIANTS[name](img, st)
        ch = P.chunk(im, s)
        k = close_step(ch)
        pose = ch[k] if k is not None else ch[-1]
        out[name] = dict(close=k, pose=pose.tolist(), step0=ch[0].tolist(),
                         end=ch[-1].tolist(), max_lift=float(ch[:, 1].max()))
    return dict(state=st.tolist(), true=None if true_grasp is None else true_grasp.tolist(),
                frames_to_close=frames_to_close, plans=out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("checkpoint", type=Path)
    ap.add_argument("--repo-id", default=None)
    ap.add_argument("--episodes", type=int, default=None, help="Limit to the first N episodes.")
    ap.add_argument("--variants", default="base,grip57,bright,dark")
    ap.add_argument("--manifest", type=Path, default=None,
                    help="JSON list of {name, frame, state, expected} live frames to score.")
    ap.add_argument("--root", type=Path, default=Path("."))
    ap.add_argument("--mask", type=Path, default=None, help="Paste region of this image onto live frames.")
    ap.add_argument("--region", default="200,480,0,260")
    ap.add_argument("--json", type=Path, required=True)
    a = ap.parse_args()
    variants = a.variants.split(",")
    t0 = time.time()
    P = Policy(a.checkpoint)
    report = dict(checkpoint=str(a.checkpoint), variants=variants, arrive=ARRIVE, fracs=FRACS, episodes=[], live=[])

    if a.repo_id:
        ds = LeRobotDataset(a.repo_id)
        hf = ds.hf_dataset.with_format(None)
        acts = np.stack(hf["action"]).astype(np.float32)
        eps = np.asarray(hf["episode_index"])
        for ep in sorted(set(eps.tolist()))[: a.episodes]:
            idxs = np.where(eps == ep)[0]
            act = acts[idxs]
            c = np.where(act[:, GI] < 50)[0]
            if len(c) == 0:
                continue
            g = int(c[0]); grasp = act[g]
            on = np.where(act[:, 1] - act[0, 1] > 4)[0]
            onset = int(on[0]) if len(on) else 0
            near = np.where(np.abs(act[:, :4] - grasp[:4]).max(axis=1) < ARRIVE)[0]
            near = near[near <= g]
            arrive = int(near[0]) if len(near) else g
            if arrive <= onset:
                arrive = g
            samples = [("start", 0)]
            samples += [(f"f{f:.2f}", int(round(onset + f * (arrive - onset)))) for f in FRACS]
            samples += [("dwell", (arrive + g) // 2), ("close-5", max(0, g - 5))]
            rows = []
            for label, t in samples:
                img, st = item_rgb(ds, int(idxs[t]))
                r = score(P, img, st, grasp, g - t, variants)
                r.update(label=label, t=t)
                rows.append(r)
            report["episodes"].append(dict(ep=int(ep), onset=onset, arrive=arrive, grasp=g, len=len(idxs), rows=rows))
            print(f"  ep {ep:2d} onset {onset:3d} arrive {arrive:3d} grasp {g:3d}", end="\r", flush=True)
        print()

    if a.manifest:
        region = tuple(int(v) for v in a.region.split(","))
        mask = cv2.cvtColor(cv2.imread(str(a.mask)), cv2.COLOR_BGR2RGB) if a.mask else None
        for e in json.load(open(a.manifest)):
            rgb = cv2.cvtColor(cv2.imread(str(a.root / e["frame"])), cv2.COLOR_BGR2RGB)
            if mask is not None:
                y0, y1, x0, x1 = region
                rgb = rgb.copy(); rgb[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
            st = np.array(e["state"], np.float32)
            exp = np.array(e["expected"], np.float32) if e.get("expected") else None
            r = score(P, rgb, st, exp, None, variants)
            r.update(name=e["name"], note=e.get("note", ""))
            report["live"].append(r)
            print(f"  live {e['name']}: " + ", ".join(
                f"{v}: close {r['plans'][v]['close']} lift {r['plans'][v]['pose'][1]:+.1f}" for v in variants))

    report["seconds"] = time.time() - t0
    a.json.parent.mkdir(parents=True, exist_ok=True)
    a.json.write_text(json.dumps(report, indent=1))
    print(f"wrote {a.json} in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    sys.exit(main())
