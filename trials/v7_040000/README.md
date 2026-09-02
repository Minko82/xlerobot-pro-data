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
