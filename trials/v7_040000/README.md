# v7 040000 hardware trials — 2 September 2026, daytime

Scene deviation, all trials: blinds closed, lamp on, daylight still dominant.
check_scene: tape 580.6 (ref 583.6) OK; shelf 131 (ref 84.5); tape column 72
(ref 88); R-B +8 (ref -32). Bottle placed label-DOWN after the label-up frame
produced a plan aimed at the shelf centre. All trials: grey mask overlay
(`calibration/hand_mask_grey88.png`), no aim offset, frames every 30 steps.

| trial | horizon | result | note |
|---|---|---|---|
| 1 | n-action-steps 100 | miss, jaw tips caught the bottle | reach 62.6/36.1 at 7.7 s, closed just below the bottle; same failure as v6 trial 7 |
| 2 | temporal ensemble 0.01 | undershoot | gripper command smeared into a ramp from 3 s; policy read a closing gripper as a finished grasp, retracted, locked at start pose with jaws shut. Loop 22.6 Hz. Fixed in runner: gripper now from the newest chunk |
| 3 | temporal ensemble 0.01, gripper from newest chunk | undershoot, never closed | reach peaked at lift 13 / elbow 85 at 7.5 s, then hovered at half depth drifting right to pan +19; the averaged plan is shallower than any single plan. Ensembling abandoned |
| 4 | n-action-steps 100 then 40 once moving | early close at lift -16, retracted, locked | every re-plan from a mid-reach view is biased short; a shorter horizon closes earlier, not later |
| 5 | n-action-steps 100, aim offset lift +4 / elbow -4 | short, closed at lift 5 / elbow 85, pan drifted to +12.5 | reach far shallower than trial 1 from the same config; bottle position differs from trial 1 (see frames) |

## Conclusion at ~1:30 pm: daylight is the confound

First-chunk plans from the recorded start pose, v7 040000, grey mask, same joint state:

| frame | light | bottle (px) | shoulder_lift at step 80 | step-99 lift / elbow |
|---|---|---|---|---|
| trial 5 start | day, blinds | (270,115) | -52.8 | -21 / +93 |
| trial 1 start | day, blinds | (130,90) | -67.8 | -24 / +82 |
| ep11 frame 0 | night | (275,153) | -26.9 | +12 / +36 |
| ep25 frame 0 | night | (158,134) | +3.7 | +36 / +30 |
| v6 trial 7 start | night | (140,204) | -28.2 | +10 / +78 |

Pan is right in every case; the reach is planned 25–70 units shallower from
daylight frames. Software brightness / colour matching does not recover it
(tested: gain, cast, per-channel and Lab statistics, histogram matching). The
scene itself has to be dark: shelf ≤ ~95, R-B negative, tape brighter than shelf.
No trial from this morning tests the retrain. Ensembling (trials 2–3) and the
adaptive horizon (trial 4) were rejected on the same shallow-reach evidence and
should be re-judged only under night light; the 100-step horizon stays the default.

## Trial 6, ~1:45 pm — blanket shade over the rig (shelf 68, tape 58, R-B -16)

| 6 | n-action-steps 100, no offset, shaded | short: closed at lift 14 / elbow 80 at 5.7 s | bottle centre (230,180); same shallow close as trial 5 in daylight at a similar spot. **Light was not the cause of the short reach.** |

The first-chunk comparison above measured reach *speed* over 3.3 s, not final depth
(trial 1 reached lift 62 after an equally slow first chunk). Withdrawn as evidence.
| 7 | n-action-steps 100, shaded, bottle far left | INVALID: the start frame shows no bottle at all — it was placed outside the camera's view. The arm searched left (pan -30), crept, closed on nothing | not a policy result |
