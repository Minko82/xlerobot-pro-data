# A2 Phase 2 — Peak Inference — Run 2

**Date:** 2026-08-01 UTC
**Platform:** Jetson Orin Nano Super 8 GB, JetPack 6.2.1 (L4T 36.4.7)
**Power mode:** MAXN_SUPER (nvpmodel ID 2)
**Supply:** Anker C300, 12 V car socket (untethered)
**Ambient:** 24.4 C (76 F), room thermometer
**Motor state:** all 17 motors verified torque-FREE before start (see PRERUN_STATE.txt)
**Duration:** 1800 s inference, 2087 s logging (ramp + steady + cooldown)

## Workload
- Model: `rlodhi/smolvla_screw_picking_5000` (fine-tuned SmolVLA, fp16)
- Runner: `~/vla/smolvla_benchmark_jetson.py`
- Camera: /dev/video2 (RealSense RGB) 640x480
- chunk_size 5, n_action_steps 5, num_steps 10, resize 512x512

## Results
| Metric | Value |
|---|---|
| GPU temp min / mean / max | 49.8 / 57.2 / **59.9 C** |
| Rise above ambient | +35.5 C |
| Max CPU temp | 58.7 C |
| Max tj | 59.9 C |
| VDD_IN mean / peak | 9.40 W / 11.48 W |
| GPU clock min / mean / max | 306 / 429 / **1020 MHz** (ceiling 1020) |
| Samples at clock ceiling | 114 of 2087 (5.5%) |
| Forward passes | 1791 |
| Inference latency | 998.8 ms (978.1-1057.9) = **1.00 Hz** |
| Latency drift | first 200: 997.8 ms; last 200: 999.3 ms (+0.15%) |
| Battery drain | **7.95 %/h** (54% -> 49%, 5% over 2264 s) |
| **Throttle** | **N** |

## Battery series
| Elapsed | SoC |
|---|---|
| t+0 s | 54% |
| t+234 s | 53% |
| t+771 s | 52% |
| t+1515 s | 50% |
| t+2100 s | 49% |

Per-interval rates scatter (6.7-15.4 %/h) because the C300 reports whole
percent; span-based estimates converge (15.4 -> 9.3 -> 9.5 -> 7.95 %/h as the
span grew). The span figure is the defensible one.

## Throttle verdict: N
Three independent checks agree:
1. Latency flat over 30 min (+0.15%, within noise).
2. GPU still reached its full 1020 MHz ceiling throughout; the achievable
   maximum never fell.
3. Peak 59.9 C is ~25 C below throttle onset.

Valid at 24.4 C ambient. At 40 C ambient the same +35.5 C rise would reach
~76 C, close enough to onset that the result should not be generalised.

## Interpretation
The GPU averaged 429 MHz against a 1020 MHz ceiling and sat at the ceiling only
5.5% of the time, while one CPU core ran at ~90%. The workload is
single-thread CPU-bound, not GPU-bound. Thermal headroom is abundant but not
the limiting factor at this operating point; the binding constraint is the
1.00 Hz inference rate.

## Files
- `log/peak-inference_*/samples.csv` - 1 Hz: temps, util, GPU clock + ceiling,
  power rails, RAM, CPU clock
- `log/peak-inference_*/annotations.csv` - battery SoC, ambient
- `log/peak-inference_*/tegrastats_raw.log` - unparsed source
- `bench.log` - per-pass inference latency and actions
- `PRERUN_STATE.txt` - pre-run temps, power mode, torque state

## Known gaps
- Ambient is a single spot reading; RH not measured.
- Whole-percent SoC is coarse; a clamp meter on the 12 V rail would give
  continuous draw.
- n = 1.
