# Block A — 50 back-to-back grasps, untethered, 2 September 2026

Policy `act_glassbottle_pick_v8` checkpoint 010000 (frozen-frame observation,
`--freeze-frame --overlay hand_mask_grey88.png --n-action-steps 100`), Orin Nano
untethered, GPU otherwise idle (training stopped at 3:48 pm). One marked
bottle position at the right of the shelf, cap up, operator replaces the bottle
on the mark between trials; no cooldown. Trials 1-16 driven from the Mac,
17-50 by the operator with `scripts/block_a.sh`. Scene at block start
(`scene_start.txt`): tape 57.7, shelf 69, R-B +26 (darker and warmer than the
recording reference); blinds closed, afternoon. 3:52 pm to ~5:30 pm.

Every trial: `tNN.csv` trajectory, `tNN_plans.csv` every re-plan,
`tNN_frames/` a frame every 30 steps plus the frozen start frame,
`tNN_summary.txt` timing, `tNN_temps_{start,end}.txt` all six servo
temperatures. `tegrastats.log` for the whole block. `results.csv` one row per
trial with the operator's verdict.

## Result

| | |
|---|---|
| grasps | **49 / 50** |
| failure | t43: acquired, dropped during the lift (GRASP stage) |
| close pose, mean ± sd (pan / lift / elbow / wrist_flex) | +33.6 ± 1.2 / +49.2 ± 4.2 / +42.0 ± 7.0 / -66.9 ± 2.2 |
| time to close | 9.38 ± 0.16 s |
| gripper reading with the bottle held | 11.2 ± 0.1 (t43: 6.9, jaws fully closed) |

The lift/elbow spread is the two arm configurations the demonstrations contain
for one spot (`../DIAGNOSIS_2026-09-02.md` §6); pan varies by 1.2 units.

## Compute, per trial (n = 50)

| | mean ± sd | min / max |
|---|---|---|
| realised control rate | 28.60 ± 0.03 Hz (30 Hz target, camera-bound) | 28.51 / 28.66 |
| chunk inference (ACT, full 100-step chunk) | 104.5 ± 3.2 ms | 96.9 / 111.1 |
| first inference of a run (CUDA warm-up) | 564 ± 14 ms | 540 / 602 |
| queue-pop step (no inference) | ~10 ms median loop 13 ms | |
| open-loop horizon | 100 steps = 3.3 s at 30 Hz | |

tegrastats over 97 min: VDD_IN 6.27 W mean, 7.80 W max; SoC tj 46.1 °C mean,
56.3 °C max; GPU utilisation 3 % mean (bursty: one chunk per 3.3 s).

## Servo temperature (start of trial, °C)

| servo | trial 1 | trial 50 | block max |
|---|---|---|---|
| shoulder_pan | 36 | 35 | 36 |
| shoulder_lift | 34 | 35 | 36 |
| elbow_flex | 33 | 33 | 33 |
| wrist_flex | 34 | 35 | 35 |
| gripper | 37 | 42 | 44 |

No thermal accumulation in the arm at this duty (25 s runs, ~10 s of motion,
light object); only the gripper warms, from holding the bottle. Success did
not decay with trial index. This bounds the thermal claim: arm-only grasping
does not approach the ceiling; the loaded-hold results in `A2/` remain the
thermal evidence, and grasp-and-rotate with the drive bus active is the next
block.

Runner peak-temperature readouts of 51-63 °C in trials 3-5, 12, 14 were single
corrupted packets (the direct before/after reads never exceeded 44 °C).
