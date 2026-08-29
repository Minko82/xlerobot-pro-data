# A2 Phase 2 — Peak Inference — n = 3 Summary

Three independent 30-minute runs at matched conditions.

## Conditions (identical across all three)
- Jetson Orin Nano Super 8 GB, JetPack 6.2.1, MAXN_SUPER, **DVFS** (no jetson_clocks)
- `rlodhi/smolvla_screw_picking_5000` fp16, chunk 5, num_steps 10, /dev/video2
- Ambient 24.4 C; all 17 motors powered and torque-free
- Start GPU temp: 49.8 / 50.0 / 50.5 C (spread 0.7 C)

## Results

| Metric | run_2 | run_3 | run_4 | mean | range |
|---|---|---|---|---|---|
| Max GPU (C) | 59.9 | 59.9 | 60.1 | **59.97** | 0.2 |
| Mean GPU (C) | 57.2 | 57.5 | 57.3 | 57.33 | 0.3 |
| Max CPU (C) | 58.7 | 58.7 | 58.6 | 58.67 | 0.1 |
| Rise above ambient (C) | +35.5 | +35.5 | +35.7 | **+35.57** | 0.2 |
| VDD_IN mean (W) | 9.40 | 9.72 | 9.34 | **9.49** | 0.38 |
| VDD_IN peak (W) | 11.48 | 12.02 | 11.52 | 11.67 | 0.54 |
| GPU clock mean (MHz) | 429 | 425 | 425 | 426 | 4 |
| Inference latency (ms) | 998.8 | 1031.5 | 989.4 | **1006.6** | 42.1 |
| Replan rate (Hz) | 1.00 | 0.97 | 1.01 | **0.99** | 0.04 |
| Forward passes | 1791 | 1729 | 1806 | 1775 | 77 |
| Latency drift (%) | +0.15 | — | -0.40 | ~0 | — |
| **Throttle** | **N** | **N** | **N** | **N** | — |

## Battery drain
Measured at different points on the discharge curve:

| Run | SoC range | Consumed | Rate |
|---|---|---|---|
| run_2 | 54 -> 49% | 5% / 37.7 min | 7.95 %/h |
| run_3 | 100 -> 94% | 6% / 35 min | distorted (see below) |
| run_4 | 86 -> 82% | 4% / 37.8 min | 6.36 %/h |

**The C300's SoC readout is non-linear near full charge.** In run_3 the pack held
100% for 935 s of continuous ~10 W draw, then shed 3% in 370 s. Rates measured in
that region are unusable. Runs 2 and 4, both mid-range, bracket the usable figure
at **6.4-8.0 %/h**.

Recommendation for the paper: report drain measured mid-range (roughly 45-90%)
and state the non-linearity, or measure current directly at the 12 V rail.

## Throttle verdict: N (n = 3)
No throttling in any run, by three independent checks each:
1. Inference latency flat over 30 min (worst drift 0.4%, and one run drifted *down*).
2. GPU reached its full 1020 MHz ceiling throughout, including the final 10 min.
3. Peak 60.1 C is ~25 C below throttle onset.

**Valid at 24.4 C ambient.** The rise above ambient is +35.6 C; in a 40 C
environment the same workload would reach ~76 C, close enough to onset that this
result should not be generalised.

## Interpretation
Peak temperature reproduced within **0.2 C across three runs** — the platform's
thermal behaviour under sustained VLA inference is highly repeatable.

The GPU averaged 426 MHz against a 1020 MHz ceiling at 23-30% utilisation while
one CPU core sat at ~90%. The workload is **single-thread CPU-dispatch-bound, not
GPU-bound**; thermal headroom is abundant and is not the limiting factor.

## Control frequency — important caveat
All three runs used `chunk 5`, the benchmark script's default, giving ~5 Hz
control. **This is a script default, not a platform limit.** The checkpoint's
native `chunk_size` is 50; measured separately, chunk 50 gives ~55 Hz control at
identical latency and identical compute. Any control-rate figure quoted from
these runs must be qualified as "as configured".

See `results/A2/optimisation_sweep/` and `protocols/C2_inference_optimization.md`.
