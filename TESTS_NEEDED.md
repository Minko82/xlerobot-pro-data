# Test plan — CLOSED 2026-09-02

**No further tests are planned.** The paper (v5, `~/Desktop/XLeRobot-Pro-Paper/v5/`)
carries every measured result and states the rest as limitations. Status of
the items below:

| # | Item | Status |
|---|---|---|
| 1 | Bus B ammeter | **done** 2 Sep — 1.41 A one arm, Table IV P2 filled |
| 2 | Register calibration | not run; one point from #1 (register reads 1.6x the meter). alpha_th stated as unidentified; Table IV P4 uses fitted asymptotes |
| 3–4 | 3–5 tau holds, the 700 g question | not run; 700 g stays a censored bound |
| 5 | n = 3 at 700 / 800 g | not run; stated in Limitations |
| 6 | Unsized failure | not run; Naive column stays a computed counterfactual |
| 7 | Closed-loop policy | **done** 2 Sep — Block A 49/50, Sec. VI-E; transit/place and drive-bus concurrency not run, stated |
| 8 | Second actuator class | not run; stated |

The original plan follows for the record.

---

Rewritten 2026-08-19 against the current draft. Supersedes the earlier version,
which predated the data audit.

Everything recoverable from existing data has been recovered: the unloaded slew
control, the τ collapse, n=3 reproducibility at 366 g, the 4.4 h isolation
aggregate, and the nine-run slew table are all in the paper and all came from
logs that already existed. **Nothing further is salvageable** — every run in the
repository was checked during the fact-check pass.

Four `[PENDING]` markers remain. They are the submission blocker: the manuscript
cannot go to an editor while they are in it.

---

## Tier 0 — same day, and item 2 gates everything else

### 1. Bus B ammeter reading · ~15 min
**Fills:** Table IV, P2 "Measured" cell (currently `[PENDING: ammeter]`).

> **Still required after the 28 August A1 run.** That run measured the compute
> rail and reset behaviour (n = 20, zero resets, 72 mV worst margin) but not bus
> current: `trial_*_buscurrent.csv` reads 0.0 mA in 304 of 340 samples, which is
> the static-hold floor this section already warns about. See `RESULTS_TABLES.md`
> § A1.

Inline ammeter on the Bus B (arm) rail. Log while the arms execute a
worst-case simultaneous multi-axis move at the deployed cap (τ = 450), not a
static hold — the envelope bounds *simultaneous commanded motion*, and a static
hold reads only 0.3–0.5 A.

Report peak, not mean. The prediction to beat is ≤ 4.90 A on a 5 A port.

> Do not substitute the per-servo telemetry sum. It is in uncalibrated register
> units and reads a static-hold floor. This was checked and it does not work.

### 2. Current-register calibration · ~half day ★ highest leverage
**Fills:** nothing directly. **Decides whether P4 is publishable as a predictive
model**, so do it before committing 10 h to long holds.

Log the servo's `current` register against a real ammeter on the same rail
across the load range (say 200 g → 900 g, 5–6 points, steady hold each).

The question it answers: the fitted ΔT∞/I² drifts threefold across the sweep.
Either the register is nonlinear in amperes — in which case converting to real
current collapses the drift and P4 becomes clean I²R — or it does not, and there
is a second dissipation path. **Right now the paper cannot say which**, and that
ambiguity is the single largest technical weakness a reviewer identified.

---

## Tier 1 — the α_th campaign (~10 h, mostly unattended)

### 3. Long holds at 3–5τ · 4 runs × 90–100 min
**Fills:** α_th in Sec. III and Table IV, P4 "Calculated" cell.

τ = 19.6 min, so a 30 min protocol spans only 1.5τ. Every run in the repository
is exactly 30.0 min — this is why α_th is unidentified.

Run 400, 500, 600, 700 g at `--minutes 100`. Keep every invariant from
`A2/STATE.md`: replayed reference pose, thread suspension, paired unloaded
control, cool to 33–35 °C on ID 8 between runs, **battery only, never charging**
(watch for a flat `bat` annotation).

### 4. The 700 g question · folded into #3
The current draft states that ≤ 700 g "completed the full 30 min without
reaching the ceiling, but this is censored by the protocol": at 700 g the joint
was still climbing at +0.4 °C/min when the run ended, having peaked at 60 °C.

The 100 min run at 700 g settles it. Either it plateaus below 65 °C — and you
can state a true continuous-duty payload instead of a censored bound — or it
aborts around 45 min, which is a third endurance point and strengthens the
thermal thesis. **Both outcomes are better than what you have.**

---

## Tier 2 — closes two standing reviewer objections (~5 h)

### 5. n = 3 at 700 and 800 g
**Answers:** "n = 1 per load," raised in every review including all three
external ones.

The existing n=3 is at 366 g *grasped*, a different condition, and the paper
correctly does not conflate them. Repeat the two sweep points that carry the
argument: 700 g (the 41% torque / 92% thermal claim) and 800 g (the 27.8 min
endurance figure). Six runs, ~5.5 h with cooldowns.

### 6. One unsized configuration run to failure · ~1 h
**Earns:** the paper's own framing. The "Naive" column in Table IV is a
*computed counterfactual*, and the draft says so. No unsized build was ever
observed to fail.

Raise the wheel acceleration register well above 20, or the arm cap well above
τ = 450, and record what breaks — brownout, reset, thermal abort, or mechanical
slip. One observed failure converts the framing from arithmetic to evidence.

Log the compute rail and kernel boot id throughout so a reset is unambiguous.

---

## Tier 3 — the capability gap (~1 week, mostly software)

### 7. Closed-loop learned policy
**Fills:** Sec. VI-E `[PENDING]`. **The single largest criticism in every review.**

Protocol is unchanged — see `AUTONOMY_PROTOCOL.md`, which is still current.
Summary:

- **ACT, not SmolVLA.** At 1.4 Hz a SmolVLA failure cannot be distinguished from
  a loop-rate failure.
- The gate is engineering: the inference scripts feed zeroed proprioceptive
  state and never command a motor. Budget most of the time here.
- **N ≥ 20**, one task, fixed start pose, per-stage failure attribution
  (perceive / approach / grasp / transit / place).
- Run trials **back-to-back without cooldown** and report success against joint
  temperature. If the model predicts the trial index where performance degrades,
  that is a task-level validation of P4 and by far the strongest result available.
- Log `LATCH` (overload latch, arm collapses) as its own failure class. Do not
  silently retry it.

---

## Tier 4 — generality (~1 day, ~$50)

### 8. Second actuator class
**Answers:** "single actuator, single platform," raised by two external reviewers.

A Dynamixel XL430 or similar. One calibration hold plus one load sweep is enough
to ask whether P4's *form* transfers with only datasheet constants and a single
fitted τ. This is what converts "STS-3215 parameters" into "a method," and it is
the difference between a characterization paper and a transferable one.

---

## Ordering

1. **Register calibration (#2) first.** Half a day, and it determines whether the
   10 h of long holds produce a clean model or an ambiguous one.
2. **Ammeter (#1)** while you are already on the bench with instruments out.
3. **Long holds (#3/#4)** — launch overnight, unattended.
4. **Unsized failure (#6)** — an hour, earns the title.
5. **n=3 (#5)** — unattended, run alongside.
6. **Policy (#7)** — the week of software work, and the thing that decides
   whether this is a characterization paper or a systems paper.
7. **Second actuator (#8)** — only if aiming beyond RA-L.

## Capture hygiene

- `run_info.json` on **every** run. The six exploratory slew runs have none, and
  their periods had to be recovered from the logs.
- Keep the `_FAILED` / `_SLIP` / `_BUMPED` naming. It preserved the exclusion
  reasons and it is why the duplicate `load_500g_verify/` was caught.
- Back up after every session (`rsync` per `README.md`) — the SD card has
  accumulated ext4 errors from power loss.
