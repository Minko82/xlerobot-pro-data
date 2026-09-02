# glassbottle_pick_v7_masked — offline evaluation of the hand-free retrain

Written 2 September 2026, ~2:10 am, from code-only tests. No motors moved, no
camera opened. Per-checkpoint reports: `eval_v7_0?0000.md`; baseline
`eval_v6_040000.md`; tool `scripts/eval_offline.py`; frames
`live_frames_manifest.json`, `train_frames/`; figures `figures/`.

## What v7 is

The same 49 episodes as v6 with region y 200:480, x 0:260 of every frame painted
grey (88 after codec rounding). That region held the operator's hand in every
approach and grasp frame of the kinesthetic demonstrations; v6 learned to read
it and, with no hand present at deployment, planned the grasp ~25 units of
shoulder_lift short and ~40 steps early (see `../handoff/2026-09-01-SESSION-B.md`).

Training: ACT defaults, chunk 100, batch 4, 40 000 steps, 3 h 49 m on the Orin
Nano at 0.34 s/step, tj peak 71 °C with a room fan on the case, no throttling,
no guard event. Loss 0.087 at 40k (v6: 0.078 at 40k, 0.063 at 53k).

**Deploy it only with `--overlay calibration/hand_mask_grey88.png`.** The policy
has never seen that region unmasked; on raw live frames its plan moves by 40–70
units (table 3, "raw").

## 1. Training-frame accuracy improves monotonically

Planned grasp pose vs the operator's, from frames 90/60/30 before each grasp
(mean signed error / mean absolute error, joint units):

| ckpt | pan | lift | elbow | wrist_flex | close step from g-60 / g-30 (true 60/30) | never closes from g-90 |
|---|---|---|---|---|---|---|
| v6 040000 (raw frames) | +0.57 / 1.38 | +1.57 / 1.83 | -1.63 / 2.89 | -0.68 / 1.87 | 60 / 33 | 31/49 |
| v7 010000 | -0.04 / 1.67 | +2.15 / 4.85 | -0.25 / 4.33 | -3.55 / 5.29 | 61 / 35 | 35/49 |
| v7 020000 | +0.70 / 2.04 | +1.62 / 3.46 | -0.88 / 4.29 | -2.68 / 3.59 | 59 / 29 | 29/49 |
| v7 030000 | +1.06 / 1.76 | +0.53 / 1.63 | -1.40 / 3.45 | -1.45 / 2.44 | 60 / 32 | 19/49 |
| **v7 040000** | **+0.23 / 1.08** | +2.04 / 2.47 | -2.54 / 3.48 | -1.55 / 1.97 | 60 / 27 | **10/49** |

Chunk L1 error over 100 steps, 100 random frames:

| ckpt | pan | lift | elbow | wrist_flex | gripper |
|---|---|---|---|---|---|
| v6 040000 (raw frames, 79 frames) | 1.45 | 4.51 | 3.70 | 4.69 | 1.38 |
| v7 010000 | 2.53 | 9.64 | 7.06 | 10.39 | 4.58 |
| v7 020000 | 2.34 | 6.60 | 5.64 | 7.11 | 2.41 |
| v7 030000 | 2.00 | 5.07 | 4.38 | 6.28 | 2.36 |
| **v7 040000** | **1.37** | **4.95** | **4.06** | **5.04** | **1.76** |

Start-frame motion onset is 57 planned vs 56 recorded at every checkpoint: the
mask did not disturb the dwell behaviour. The "never closes from g-90" column
falling from 35 to 10 means later checkpoints commit to the grasp inside the
100-step chunk far more often, which is what the deployment horizon needs.

**Pick 040000.** Every metric is best there and the curve had not flattened. On its
own frames it matches v6's fit quality within noise while carrying no hand cue.

## 2. Both policies read the bottle (image-swap ablation)

Same joint state (the recorded start pose), image replaced by a training start
frame with the bottle elsewhere. Last planned step of the chunk, v7 040000:

| image | bottle | planned pan | planned lift | planned elbow |
|---|---|---|---|---|
| trial 7's own frame | left-centre | -10.1 | +9.7 | +77.9 |
| ep20 | right, near | +21 | -21 | +99 |
| ep15 | left, near | -31 | +16 | +87 |
| ep45 | far left, far | -16 | +36 | -32 |
| ep11 | centre, far | +7 | -2 | +37 |

Pan follows the bottle across the full ±30 range and lift/elbow follow its
depth. Vision is used; the failures below are not blindness.

## 3. Live frames — what the wrist camera saw in trials 2–9

Planned close step and pose from each frame with the joint state at that
moment. Reference = operator's grasp for that bottle position (regression from
`v6_040000/bottle_map.csv`), or for trial 9 the pose that actually grasped.

| frame | v6 040000 (raw) | v7 040000 (masked) | reference |
|---|---|---|---|
| trial 7 step 120, mid-reach | close 47 @ -10.0/+25.2/+60.2/-47.1 | close 45 @ -8.9/+30.9/+62.4/-55.3 | -10/+49/+30/-58 |
| trial 7 step 90 | none @ +26.5 lift | none @ +34.4 lift | -10/+49/+30/-58 |
| trial 9 step 150, hovering at bottle | close 83 @ +7.7/+51.9/+56.7/-92.0 | close 38 @ +8.8/+50.1/+51.3/-81.6 | +10/+53/+65/-97 |
| trial 9 step 90 | none | close 43 @ +7.1/+46.4/+56.2/-81.0 | +10/+53/+65/-97 |
| trial 8 step 150, bottle between jaws | close 72 (hesitant on hardware) | **close 8** @ +0.7/+52.4/+37.8/-71.2 | — |

Region sensitivity of v7 040000 (largest change in the planned pose when the
masked region is filled with the hand patch / with the raw camera pixels):
16–58 / 17–70 units. Both are out of distribution for v7; the grey mask is not
optional.

Reading: where the arm is already at the bottle (trial 9 step 150, trial 8 step
150) v7 closes promptly and near the right pose, which is where v6 with the hand
patch dithered. Mid-reach (trial 7 step 120) v7 still plans short: lift +31
against +49, better than v6's +25 but not fixed.

## 4. Why mid-reach is still short, and what fixes it

`figures/trial7_step120_vs_state_matched_training_frames.png`: the training
frames whose joint state matches trial 7 step 120 come from episode 40, whose
bottle sat next to the robot, and at that arm state its bottle lands on the same
pixels as trial 7's. One wrist RGB camera cannot tell the two depths apart; the
operator told them apart by continuing to move until the bottle was between the
jaws. The policy instead executes a 100-step chunk open-loop (3.3 s) and closes
where the plan said.

This is a **deployment horizon** problem, not a weights problem:

- `--n-action-steps 100` was chosen so the start-pose dwell could not lock the
  arm (31 Aug). It costs all visual correction during the reach.
- **Temporal ensembling** (`--temporal-ensemble 0.01`, added to the runner)
  infers every step and executes the weighted average of every chunk's prediction
  for now. `scripts/ensemble_sim.py`: fed the unchanged start observation each
  step, the ensembled lift command starts rising at step 73 (0.01) / 65 (0.05)
  for both v6 and v7 — the dwell does not lock. Mid-reach, every step's fresh
  chunk sees the bottle still above the jaws and extends the plan. The runner
  path was exercised in code against 040000: action shape (1, 6), lift rising
  from step ~65. A full inference every step costs 47 ms on the Orin Nano, so
  the loop will run about 21 Hz and the arm about 30 % slower than demonstrated.
  Untested on hardware.

## 5. Protocol for the first hardware session (lights on, scene checked)

Every trial: `goto_start_pose.py` first; `--log-frames` and `--log-trajectory`
always; no `--aim-offset`; bottle cap-up, body down the image, mid-shelf.

1. **Control**: v7 040000, `--overlay hand_mask_grey88.png --n-action-steps 100`.
   Expect: correct hover, decisive close, possibly short mid-reach.
2. **Ensembling**: same, `--temporal-ensemble 0.01` (drop `--n-action-steps`).
   Expect: arm starts within ~2.5 s, reach corrects continuously.
   If it hovers without closing, try 0.05.
3. Repeat the winner left / centre / right, 3 each. Success = grasp and hold to
   the end of the run. Then re-run `bottle_map.py`-style position logging so
   every trial has a start frame and a known bottle position.

If 2 grasps reliably where 1 closes short, the depth-ambiguity claim is
confirmed on hardware and Block A can start with that configuration.

## 6. Open

- The pre-grasp dwell (mean 67 frames) is still in the data. Ensembling makes
  it survivable; shortening it in post-processing remains untried.
- 49 episodes, one camera, no depth. A second (fixed) camera or the D435's depth
  stream would remove the ambiguity in section 4 at the data level.
- Disk on the robot: 1.5 GB free with four 591 MB v7 checkpoints. 010000 and
  020000 can go once 040000 has been tried on hardware.
