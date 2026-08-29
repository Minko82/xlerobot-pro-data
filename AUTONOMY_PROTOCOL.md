# C1 — Autonomous mobile pick-and-transport protocol

**Purpose.** Not a demo. This experiment is designed so that three claims the
paper already makes are put at risk by it, and so that the infrastructure it
requires is the same infrastructure the tactile work will need later.

What it tests:

| Claim in the paper | How this experiment can falsify it |
|---|---|
| Tri-Bus + envelope holds under concurrent load (Sec. VII-E) | First **three-way** concurrency: inference + drive bus + loaded arm bus. Currently only tested with a *static* arm. |
| The platform is thermally bound, not torque bound (Sec. VII-D) | Back-to-back trials with no cooldown → does task success decay as joint temperature climbs? |
| Grasp geometry is the binding manipulation constraint (Sec. VII-C) | Object set deliberately spans the Fin-Ray boundary, not just the bottle we know works. |

---

## 1. Task

Fixed, short, repeatable. Resist adding stages.

1. Robot starts at pose **A**, arms at the reference pose, object on a marked
   stand at known height.
2. **Perceive** → **approach** → **grasp** the object.
3. **Transit** to pose **B** (≈2 m), object held.
4. **Place** on a marked stand at B.
5. Return to A unloaded. This is the inter-trial interval — **do not add cooldown.**

**Learned vs scripted must be declared.** The realistic split given the stack:
grasp is the learned policy (ACT), base motion is waypoint/open-loop. That is
publishable as long as the paper says so plainly. Do **not** describe the base
motion as autonomous navigation — RTAB-Map/Nav2 are not installed.

**Policy: ACT, not SmolVLA.** At 1.4 Hz a SmolVLA failure cannot be distinguished
from a loop-rate failure, which would confound the compute result.

---

## 2. Conditions

Run in this order. Block A is the one that must happen.

### Block A — thermal accumulation (the headline)
- **N = 30**, continuous, back-to-back, no cooldown between trials.
- Record shoulder-lift temperature at the *start* of every trial.
- This is the block that produces a result nobody else reports: task success
  as a function of actuator temperature and duty cycle.
- Abort the block if the joint reaches 65 °C (the run itself is the measurement —
  record the trial index at which it happens).

### Block B — tethered vs untethered (restores a deleted claim)
- **N = 20 untethered**, **N = 20 tethered** (wall power + external PC running
  the policy, robot otherwise identical).
- Start each block from a cooled joint (33–35 °C) so Block A's thermal effect
  does not contaminate it.
- This fixes the "no controlled comparison anywhere in the paper" objection and
  restores the tethered-baseline claim that had to be cut from the abstract.

### Block C — grasp boundary (sets up the tactile thesis)
- **N = 20**, cooled start, three objects:
  1. 500 ml bottle (known-good control, from B1)
  2. thin-walled can (deformed at grip load 38 in earlier testing)
  3. an object at/over the jaw width limit (the 475 g can failed to close at all)
- Expect failures. **The failure modes here are the motivation for tactile
  sensing** — log them in detail, including whether the failure was detectable
  from encoder data alone.

### Optional Block D — envelope on/off
Same task, deployed caps (τ 450/650, accel 20/40) vs raised caps. Tests whether
the envelope imposed for power integrity actually costs task performance. Only
if A–C are done and the arm is healthy.

---

## 3. Logging

### Per trial — one row in `trials.csv`
```
trial_idx, block, condition, object, t_start, t_end,
outcome,                       # SUCCESS | FAIL
fail_stage,                    # see taxonomy below, blank if SUCCESS
shoulder_temp_start_c, shoulder_temp_end_c,
grip_load_at_close, grip_dev_during_transit,
realized_control_hz,           # measured, not bench
policy_latency_median_ms,
boot_id,                       # to catch a brownout reset
notes
```

### Continuous, for the whole block
- `servo_telemetry.csv` — existing 1 Hz × 17 format, unchanged
- `samples.csv` — tegrastats via `log_thermal_power.py`, running the entire block
- `run_info.json` — existing provenance format + `policy`, `checkpoint`,
  `learned_stages`, `scripted_stages`

Run `log_thermal_power.py` across the **whole block**, not per trial, so the
thermal accumulation is one continuous trace.

---

## 4. Failure taxonomy

Attribute every failure to exactly one stage. A bare success rate is close to
worthless; the stage breakdown is the result.

| Code | Stage | Definition |
|---|---|---|
| `PERC` | Perceive | object not detected, or centroid off by enough to doom the grasp |
| `APPR` | Approach | IK unreachable, or arm stopped short |
| `GRASP` | Grasp | jaws closed but object not acquired, or acquired then dropped at lift |
| `TRANSIT` | Carry | object lost or displaced during base motion |
| `PLACE` | Place | released outside the target, or knocked over |
| `LATCH` | System | **servo overload latch** — output drops to 20 %, arm collapses |
| `RESET` | System | compute brownout (boot id changed) |
| `ABORT` | System | thermal ceiling or operator stop |

**`LATCH` must be its own class and must not be silently retried.** Per
`A2/STATE.md` the servos latch and do not self-recover; one observed collapse was
1195 counts (~105°). For a thesis heading toward human-adjacent operation this is
a safety finding, not a nuisance — record the trial, the joint, and the load.

---

## 5. What it fills in the paper

**Table — autonomous task performance**

| Condition | N | Success | PERC | APPR | GRASP | TRANSIT | PLACE | System |
|---|---|---|---|---|---|---|---|---|
| Untethered, cooled | 20 | | | | | | | |
| Tethered, cooled | 20 | | | | | | | |
| Untethered, accumulated | 30 | | | | | | | |
| Grasp-boundary set | 20 | | | | | | | |

**Figure — the one worth having:** trial index on x; two y-axes: shoulder-lift
start temperature, and cumulative success rate (or a rolling window). If success
decays as temperature climbs, that single panel cashes the entire thermal
characterization into a task-level consequence. If it does not decay, report that
plainly — it bounds the thermal claim to bench conditions, which is also useful.

**Sentence the paper currently cannot write:** "Across N trials of concurrent
inference, base motion and loaded actuation, we observed zero brownout resets."

---

## 6. Safety and abort

- Clear the transit path. The base is slow (0.152 m/s²) but the arm carries mass
  at 1.2 m height.
- **Nobody in the workspace during autonomous runs.** The latch failure drops the
  arm without warning.
- Abort criteria: shoulder-lift ≥ 65 °C, any `LATCH`, any `RESET`, or object
  leaving the table.
- Keep the gentle torque bleed-off (`TORQUE_RELEASE_SECONDS = 8.0`) on every
  exit path, including the autonomous one.

---

## 7. Time budget

| Item | Estimate |
|---|---|
| Closed-loop infrastructure (proprioception in, motor commands out) | **6–10 h — the real cost** |
| Train/obtain ACT checkpoint for the task | 1–2 h, off-robot |
| Pilot (5 trials, shake out) | 1 h |
| Block A (30 trials, continuous) | ~2 h |
| Block B (40 trials + cooldowns) | ~2.5 h |
| Block C (20 trials) | ~1 h |

The infrastructure is the gate, and it is **not wasted effort** — a closed-loop
path from sensing to motor command is exactly what the tactile work needs next.
Build it once, properly.
