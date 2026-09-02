"""Does the policy depend on the operator's hand? Swap the bottom-left region between training and deployment frames."""
import sys, cv2, numpy as np, torch
from pathlib import Path
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import prepare_observation_for_inference
ckpt = Path(sys.argv[1])
cfg = PreTrainedConfig.from_pretrained(ckpt); cfg.pretrained_path = str(ckpt)
policy = ACTPolicy.from_pretrained(ckpt).eval().to("cuda")
pre, post = make_pre_post_processors(cfg, pretrained_path=str(ckpt)); dev = torch.device("cuda")
def rgb(p): return cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2RGB)
def plan(img, state):
    obs = {"observation.images.top": img.astype(np.uint8), "observation.state": np.array(state, np.float32)}
    with torch.inference_mode():
        o = prepare_observation_for_inference(obs, dev, "pick up the glass bottle from the shelf", "so101_follower")
        ch = policy.predict_action_chunk(pre(o))[0]
        try: un = post(ch)
        except Exception: un = torch.stack([post(ch[i:i+1])[0] for i in range(ch.shape[0])])
    a = np.asarray(un.cpu()).reshape(-1, 6); k = np.where(a[:, 5] < 50)[0]
    kk = int(k[0]) if len(k) else None; g = a[kk] if kk is not None else a[-1]
    return kk, g
def show(label, img, state):
    k, g = plan(img, state); print("%-46s close %4s   pan %6.1f  lift %6.1f  elbow %6.1f  wflex %6.1f" % (label, k, g[0], g[1], g[2], g[3]))
REG = (slice(200, 480), slice(0, 260))      # bottom-left: gripper body, hand, cyan part
t7_120 = rgb("/tmp/t7_120.jpg"); s7_120 = [-11.58,6.57,71.38,-33.67,38.85,59.58]
t7_090 = rgb("/tmp/t7_090.jpg"); s7_090 = [-7.96,-23.13,81.85,-15.84,38.85,59.92]
ep0 = rgb("/tmp/v6_ep00_g-120.jpg"); s0 = [-7.7,54.2,38.8,-84.7,38.8,60.3]
ep13 = rgb("/tmp/v6_ep13_g-120.jpg"); s13 = [-1.8,-57.4,99.0,-8.2,39.0,60.3]
ep13b = rgb("/tmp/v6_ep13_g-060.jpg"); s13b = [-6.0,48.4,58.8,-75.1,38.9,60.3]
def swap(dst, src):
    out = dst.copy(); out[REG] = src[REG]; return out
print("expected operator grasp for trial-7 bottle: pan -10  lift +49  elbow +30  wflex -58\n")
show("trial7 step120, as seen", t7_120, s7_120)
show("trial7 step120 + hand region from ep0", swap(t7_120, ep0), s7_120)
show("trial7 step120 + hand region from ep13 g-60", swap(t7_120, ep13b), s7_120)
show("trial7 step120 + hand region from ep13 g-120", swap(t7_120, ep13), s7_120)
show("trial7 step090, as seen", t7_090, s7_090)
show("trial7 step090 + hand region from ep0", swap(t7_090, ep0), s7_090)
show("trial7 step090 + hand region from ep13 g-120", swap(t7_090, ep13), s7_090)
print()
show("ep0 g-120, as recorded (truth lift 67 elbow 18)", ep0, s0)
show("ep0 g-120, hand region replaced by trial7", swap(ep0, t7_120), s0)
show("ep13 g-60, as recorded (truth lift 57 elbow 57)", ep13b, s13b)
show("ep13 g-60, hand region replaced by trial7", swap(ep13b, t7_120), s13b)
cv2.imwrite("/tmp/hand_swap_demo.jpg", cv2.cvtColor(np.hstack([t7_120, swap(t7_120, ep0), swap(ep0, t7_120)]), cv2.COLOR_RGB2BGR))
