# Why the ACT pickup policy reaches short (or long) — 2 September 2026

Written from the afternoon's diagnostics. Artefacts: `diag_2026-09-02/`,
`replay/`, `midreach_v7_040000*.json`, `v7_040000_diag/`; tools
`scripts/eval_midreach.py`, `summarise_midreach.py`, `replay_episode.py`,
`freeze_dataset_videos.py`, `grid_trials.py`; runner flags `--freeze-frame`,
`--color-match`, `--log-plans` in `act_policy_control.py`.

## Verdict

The control pipeline is correct. The model is accurate on its own data at every
point of the reach. The reach error comes from the **observation**: the policy
learned to read the arm *with the operator's hand on it* to know where it is in
the reach, and that appearance never occurs at deployment. When the image is out
of distribution the policy falls back on the joint state, and the joint state
alone cannot tell "arrived at this bottle" from "passing through the grasp pose
of a nearer bottle", because the grasp poses of near bottles lie on the path to
far ones. So it closes at the first plausible grasp pose: **short for far
bottles, and the same mechanism can run long for near ones.**

Fix under test: train on frames frozen at the episode's first image
(`glassbottle_pick_v8_frozen`), deploy with `--freeze-frame`. The image then
carries only the bottle position; the arm is represented by the state alone,
which is identical in training and deployment. Training started 2:17 pm.

## What was checked and found clean

| item | test | result |
|---|---|---|
| action/observation timing | dataset: `action[t] == state[t+1]` in 49/49 episodes, timestamps = frame/30 exactly; camera `async_read` returns a fresh frame every 30 Hz poll (0/89 repeats, 23 ms median wait); same `get_observation` path in recorder and runner | consistent; a 1-frame lead in both |
| normalisation | checkpoint mean/std vs dataset stats: max diff 3e-6; state and image through the real preprocessor match (x-mean)/std exactly (image: ImageNet stats, RGB, [0,1], (1,3,480,640)); unnormaliser round trip 1e-5 | exact |
| units / order / clamps | joint order pan,lift,elbow,wflex,wroll,gripper in dataset, checkpoint and runner; RANGE_M100_100 with the live calibration, gripper 0-100; int truncation ≤0.09 units; no training action outside ±100 except lift/elbow at the range floor | consistent |
| calibration + servo | **open-loop replay of three episodes' recorded actions** through the runner's own path (`replay_episode.py`): pose at the recorded close step differs from the recorded grasp pose by ≤1.0 lift, ≤2.0 elbow, ≤0.9 pan/wrist (eps 7, 46, 22, spanning pan +28..+37, lift +7..+68). Tracking lag 3-7 frames during motion, mean |err| 1-4 units, settles within 4 units of a held command | a correct plan is executed to ~1 unit |
| chunk / replanning | `n_action_steps=100` executes the whole chunk open loop (3.3 s); re-plans at 0, 100, 200…; each re-plan's step-0 action ≈ observed state, so the arm reverses ~20 units for 0.4 s at every chunk boundary (trial 1 steps 100-110) | costs time, not the final pose |
| temporal ensembling | lerobot's ensembler weights the OLDEST chunk most (w=exp(-0.01·i), i=0 oldest); with `action = state[t+1]` every chunk is anchored to a state that lags the plan by 3-7 frames, so the average is behind the newest plan and the arm slows and stalls (v7 trials 2-3: peaked at lift 13) | correctly implemented, wrong tool for this action convention |

## What was found wrong

### 1. The camera is fixed, not a wrist camera

Every handoff since 31 August calls it a wrist camera. The tape fiducial stays
at the same column while the arm crosses the frame, and the gripper enters
from the lower-left. The "depth ambiguity of a wrist camera" story for the
mid-reach short close (EVAL_v7_masked §4) is therefore not the mechanism.

### 2. On training frames the plan is right all along the reach

`eval_midreach.py`, v7 040000, all 49 episodes, frames at 0/25/50/75/100 % of
the way from motion onset to arrival, then mid-dwell and close-5:

| frame | never closes in chunk | signed err lift / elbow | abs err lift | slope err~true lift |
|---|---|---|---|---|
| f0.25 | 49/49 | -1.4 / +1.6 | 6.5 | +0.05 |
| f0.50 | 48/49 | +1.6 / -1.9 | 3.5 | +0.03 |
| f0.75 | 36/49 | +2.6 / -3.5 | 3.0 | +0.03 |
| f1.00 (arrived) | 11/49 | +2.5 / -3.4 | 2.8 | +0.04 |
| dwell | 1/49 | +1.7 / -1.8 | 2.3 | +0.06 |

No shrink toward the mean depth; near and far bottles alike. Gripper state
57 vs 60, image gain ×0.8 or ×1.25: no change. So there is nothing wrong with
the weights on their own distribution, and the earlier g-90/60/30 scores were
measuring the dwell (median 76 frames), not the reach.

### 3. On live frames from the same arm states the plan closes at once

Trial 1 (2 Sept, daylight), state at s150 = lift +33, s180 = lift +48, gripper
57 (open), bottle needing roughly lift 65:

| image given to the policy (same live state) | planned close step | lift at close / peak |
|---|---|---|
| live frame | **0** | 46 / 46 (s150), 61 / 61 (s180) |
| live, gripper inpainted away | 3 / 0 | same |
| live, colour-matched to training shelf statistics | 4 / 2 | same |
| live, bottle inpainted away | 33 / 27 | 48 / 56 |
| live + training arm-and-hand pixels pasted (state-matched episode) | 18-37 / 7-78 | 50-56 / 59-64 |
| training frame at a matched state, live joint state | none / none | 67 / 68 |

The joint state is fine (row 6). Colour is not it (row 3). What restores the
plan is the arm as it looked in training, with the hand on it (row 5). The
bottle contributes to "close now" (row 4) but only in combination.

### 4. Why the mask did not remove the hand

`hand_mask_grey88.png` covers y 200:480, x 0:260. In 25 of 49 episodes the
hand and forearm are in the open frame mid-reach and at arrival (contact sheet
`diag_2026-09-02/sheet_grasp.jpg`): every centre or right bottle, and every deep
reach. Mean grasp lift with the hand visible: 55; without: 37. The v7 retrain
therefore still learned the hand, and learned it as a depth cue.

### 5. The plan log shows the failure directly

`v7_040000_diag/base_t1_plans.csv` (v7 040000, daylight, bottle at cap
(178,120), needs ~lift 65): at the step-100 re-plan the state was lift -8.6,
elbow 82, gripper 61; the policy planned to close 15 steps later at lift 19 /
elbow 76. It did, on nothing, peak lift 40. With `--color-match` it reached
lift 50 before closing and drifted 10 units right. Both short.

### 6. Two more data facts that limit any policy trained on this set

- **Camera or table moved after the second recording batch**: tape at 597-601 px
  in episodes 0-8, 580-584 px in 9-48 (`diag_2026-09-02/episodes_table.csv`).
  About 1.3 pan units; small, but it means the first nine episodes map pixels
  to poses differently.
- **Two grasp styles for the same spot**: ep 13 (cap at 147,165) was grasped at
  lift 57 / elbow 57 / wrist -99; ep 37 (cap at 191,165) at lift 16 / elbow 67
  / wrist -47 (`diag_2026-09-02/pair_grasp.jpg`). Cap position predicts pan to
  1.7 units (LOO) but lift only to 21 units: the operator chose the arm
  configuration, not the bottle. A policy has to pick one from the start frame;
  the mid-reach state then disambiguates.
- Coverage: caps span x 7-513, y 28-348 px, 35 detected of 49; the far row and
  the near-right are sparse (`episodes_table.csv`).

## What was NOT the cause

- Normalisation, joint order, units, clamping (exact).
- Calibration drift, gravity sag, envelope limits (replay: ≤2 units).
- A timing offset between image and action (same path both sides; a 1-frame
  lead by construction).
- The 100-step open-loop horizon (a correct 100-step plan lands within 1 unit).
- Lighting alone (colour matching does not undo the mid-reach close, though
  daylight does bias the early-reach plans and every 2 Sept trial ran in it).

## The fix and how it will be judged

`glassbottle_pick_v8_frozen`: v7's parquet unchanged; every video frame of an
episode replaced by that episode's decoded frame 0 (h264 crf 18, frame counts
and timestamps verified file by file, decoded frames within an episode differ
by 0.7 grey levels, across episodes by 10). Training: ACT defaults, chunk 100,
30 000 steps, save every 10 000 (disk allows three checkpoints), no image
augmentation (lerobot's default transforms include ±5° rotation and 5 %
translation, which would corrupt a fixed camera's pixel-to-pose map).

Deployment: `--freeze-frame --overlay hand_mask_grey88.png --n-action-steps 100`.
The image is the step-0 frame for the whole run; the policy cannot be surprised
by the arm because it never sees it.

Offline gate (`eval_midreach.py` on v8 with the frozen dataset): the same table
as §2 — plan error ≤3 units at f0.50-f1.00, never-closes ≥45/49 at f0.50.
Then the live start frames from the v7 trials with their recorded states.

Hardware gate: `GRID_PROTOCOL.md` — 9 cells × 3 trials per configuration,
night lighting, v7 baseline vs v8 frozen, bias / trend / SD in cm.

## Still open after this

- The dataset's two grasp styles and the batch-2 camera shift are in v8 too.
  If v8's error is position-dependent, re-record those regions with one style.
- Daylight: the frozen start frame is still a daylight frame; `--color-match`
  is a partial remedy. Recording a second set under daylight is the real one.
- The ensembler's weighting and the `action = state[t+1]` convention pull
  against each other; if closed-loop correction is ever wanted, record with
  the actions leading the state by the servo lag (leader-follower with the
  right arm as leader), or weight the newest chunk most.
