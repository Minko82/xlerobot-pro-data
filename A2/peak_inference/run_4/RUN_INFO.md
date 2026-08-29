# A2 Phase 2 — Peak Inference — Run 4

**Date:** 2026-08-02 UTC
**Platform:** Jetson Orin Nano Super 8 GB, JetPack 6.2.1 (L4T 36.4.7)
**Power mode:** MAXN_SUPER · **Clock policy:** DVFS (jetson_clocks NOT applied)
**Supply:** Anker C300, 12 V car socket (untethered)
**Ambient:** 24.4 C (76 F)
**Motor state:** all 17 motors powered and verified torque-FREE before start
**Duration:** 1800 s inference, 2087 s logging

## Workload
`rlodhi/smolvla_screw_picking_5000` fp16 via `smolvla_benchmark_jetson.py`,
/dev/video2 (RealSense RGB) 640x480, **chunk 5, num_steps 10** — matching
run_2 and run_3 exactly.

## Results
| Metric | Value |
|---|---|
| GPU temp min / mean / max | 50.4 / 57.3 / **60.1 C** |
| Rise above ambient | +35.7 C |
| Max CPU / tj | 58.6 / 60.1 C |
| VDD_IN mean / peak | 9.34 W / 11.52 W |
| GPU clock min / mean / max | 306 / 425 / 1020 MHz |
| Forward passes | 1806 |
| Inference latency | 989.4 ms (968.0-1056.8) = **1.01 Hz replan** |
| Latency drift | first 200: 990.9 ms; last 200: 986.8 ms (**-0.4%**) |
| Battery used | **4% over 37.8 min** (~6.4 %/h) |
| **Throttle** | **N** |

## Battery series
| Elapsed | SoC |
|---|---|
| t+0 s | 86% |
| t+540 s | 85% |
| t+1482 s | 83% |
| t+1802 s | 83% |
| t+2265 s | 82% |

## Throttle verdict: N
Latency drifted **downward** 0.4% over 30 min (990.9 -> 986.8 ms), the GPU still
reached its full 1020 MHz ceiling, and peak temperature stayed ~25 C below
throttle onset. Valid at 24.4 C ambient.

## Note on control frequency
This run used `chunk 5`, the benchmark script's default, giving 5.05 Hz control
(replan x n_action_steps). That is a **script default, not a platform limit** —
the checkpoint's native chunk_size is 50. Measured separately, chunk 50 yields
~55 Hz control at identical latency. See `results/A2/optimisation_sweep/` and
`protocols/C2_inference_optimization.md`.

## Files
- `log/peak-inference_*/samples.csv` - 1 Hz temps, util, GPU clock, power, RAM
- `log/peak-inference_*/annotations.csv` - battery SoC, ambient
- `log/peak-inference_*/tegrastats_raw.log` - unparsed source
- `bench.log` - per-pass latency
- `PRERUN_STATE.txt` - pre-run temps, clocks, torque state

## Known gaps
- Ambient is a single spot reading; RH not measured.
- Whole-percent SoC is coarse.
