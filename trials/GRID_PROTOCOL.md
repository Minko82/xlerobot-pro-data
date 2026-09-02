# Reach-accuracy grid protocol

The question this answers: does the policy put the jaws where the operator put
them, everywhere on the shelf, repeatably?  It separates a constant bias from a
position-dependent error from random spread, and compares deployment
configurations on the same cells.  Tool: `scripts/grid_trials.py`.

## Setup (once, ~20 min)

1. Lighting as recorded: **night, lamp on, blinds closed**. Check with
   `scripts/check_scene.py` before the first trial and again every ~10 trials
   (shelf ~85-90, R-B negative, tape brighter than shelf). Daylight trials go
   in the log as `invalid`; they are not measurements of the policy.
2. Mark a grid on the shelf with tape: 3 columns x 3 rows, 8 cm apart, inside
   the camera's view (all 9 marks visible in `camera_preview.py` with the arm
   at the start pose) and inside the demonstrated area -- the training bottle
   caps span x 7-513 px, y 28-348 px in the start frame.  Name them A1..C3,
   A = left column, 1 = far row.  Record each mark's x/y in cm from the
   centre mark.
3. Reference pose per cell: bottle on the mark, cap up, label away from the
   camera (the recorded orientation), then

   ```bash
   python scripts/grid_trials.py reference --grid trials/grid/grid.json --cell A1 --x-cm -8 --y-cm 8
   ```

   The arm goes limp; put the jaws around the bottle the way you grasped it
   when recording (same style -- the demonstrations contain two styles for the
   same spot, shoulder-high/wrist-down and shoulder-low, see
   `DIAGNOSIS_2026-09-02.md`; pick one and use it for every cell). ENTER
   records the pose. Do it twice per cell; the mean is stored and the spread
   tells you the operator's own repeatability.

## Trials

Per configuration, **3 trials per cell, 9 cells = 27 trials**, cells in a
different order each pass so drift does not alias with position. Before each:
bottle back on the mark (it moves when hit), jaws open, `goto_start_pose.py`
(the harness does this).

```bash
python scripts/grid_trials.py run --grid trials/grid/grid.json --cell A1 \
    --config v8_freeze_n100 \
    --checkpoint outputs/act_glassbottle_pick_v8/checkpoints/last/pretrained_model \
    -- --freeze-frame --overlay calibration/hand_mask_grey88.png --n-action-steps 100
```

Every trial logs the trajectory, a frame every 30 steps, and every plan the
policy made (`--log-plans`: state at the re-plan, planned close step, planned
close pose, peak lift). After the run the harness prints the close pose minus
the reference and asks for the outcome:

| outcome | meaning |
|---|---|
| grasp | jaws closed on the bottle and held to the end of the run |
| short | closed on the near side of the bottle (jaws between robot and bottle) |
| long | closed beyond the bottle / pushed it away |
| lateral | closed beside the bottle (pan error) |
| no-close | never closed |
| knocked | bottle knocked over or moved before the close |
| invalid | scene problem (light, operator in view, bottle off the mark) |

Configurations to compare, in this order, same cells:

1. `v7_n100`: v7 040000, `--overlay hand_mask_grey88.png --n-action-steps 100` (the current default; expect short closes, this is the baseline)
2. `v8_freeze_n100`: v8, `--freeze-frame --overlay hand_mask_grey88.png --n-action-steps 100`
3. `v8_freeze_n30`: same with `--n-action-steps 30` (visual correction is irrelevant with a frozen frame, so this tests only whether frequent re-anchoring to the lagging state slows or stalls the reach)
4. `v8_freeze_te`: same with `--temporal-ensemble 0.01` (expect slower and shallower: the ensembler weights the OLDEST chunk most, and each chunk is anchored to a state that lags the plan by 3-7 frames)

## Report

```bash
python scripts/grid_trials.py report --grid trials/grid/grid.json
```

Per configuration and cell: mean signed joint error at the close, its SD, grasp
count; X/Y error in cm via the joint-to-shelf map fitted from the references
(its residual is printed -- if it is above ~1 cm the grid is too small or the
references too noisy); then the constant bias, the trend of error with cell
position, and the residual SD.  "Fixed" means: |bias| and |trend x 8 cm| under
1 cm in X and Y on every row and column, residual SD under 1 cm, and grasp
rate above 8/9 at every cell -- for the same policy, not one tuned per cell.

## What each failure pattern means

- **Same sign everywhere** (bias): calibration or a learned offset -> `--aim-offset` is legitimate only after this is measured here.
- **Grows with distance from the centre** (trend): the policy shrinks toward the mean grasp; more data at the edges, or the image is not being read.
- **Large SD at the same cell**: the plan depends on something other than the bottle position (lighting, hand, arm appearance) -- check the frames, then the plan log: if the planned close pose differs between trials from the same start frame, the observation differs.
- **Deep cells short, near cells long**: the state is deciding "arrived" instead of the image (the 2 September failure); the image is out of distribution.
