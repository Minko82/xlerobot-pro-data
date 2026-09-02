"""Does temporal ensembling break the start-pose deadlock? Feed the ensembler the same start-frame chunk
every step (the arm has not moved yet, so the observation is unchanged) and watch the executed command."""
import sys, cv2, numpy as np, torch
sys.path.insert(0, "/home/xle/xlerobot-pro-data/scripts")
from eval_offline import Policy
from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
ck, frame, mask_p = sys.argv[1], sys.argv[2], (sys.argv[3] if len(sys.argv) > 3 else None)
P = Policy(ck)
rgb = cv2.cvtColor(cv2.imread(frame), cv2.COLOR_BGR2RGB)
if mask_p:
    m = cv2.cvtColor(cv2.imread(mask_p), cv2.COLOR_BGR2RGB); rgb[200:480, 0:260] = m[200:480, 0:260]
st = np.array([-0.07, -99.5, 99.46, 75.63, 38.85, 94.2], np.float32)
chunk = torch.tensor(P.chunk(rgb, st))[None]          # (1, 100, 6) in joint units
k = np.where(chunk[0, :, 1].numpy() - st[1] > 4)[0]; print(f"open-loop chunk from the start frame: lift onset at step {k[0] if len(k) else None}, close at "
      f"{(np.where(chunk[0,:,5].numpy()<50)[0][0] if (chunk[0,:,5].numpy()<50).any() else None)}")
for coeff in (0.01, 0.05):
    ens = ACTTemporalEnsembler(coeff, 100); ens.reset(); cmds = []
    for t in range(140):
        cmds.append(ens.update(chunk.clone())[0].numpy())
    cmds = np.array(cmds); lift = cmds[:, 1]
    on = np.where(lift - st[1] > 4)[0]; on = int(on[0]) if len(on) else None
    print(f"coeff {coeff}: ensembled lift command at steps 0/30/60/90/120: " + " ".join(f"{lift[i]:+.1f}" for i in (0, 30, 60, 90, 120))
          + f"   -> motion onset step {on} (open-loop chunk: {k[0] if len(k) else None})")
