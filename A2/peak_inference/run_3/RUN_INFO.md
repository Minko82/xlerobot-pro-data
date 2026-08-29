# A2 Phase 2 — Peak Inference — Run 3

**Date:** 2026-08-01 UTC
**Platform:** Jetson Orin Nano Super 8 GB, JetPack 6.2.1 (L4T 36.4.7)
**Power mode:** MAXN_SUPER · **Clock policy:** DVFS (306–1020 MHz)
**Supply:** Anker C300, 12 V car socket (untethered)
**Ambient:** 24.4 C (76 F)
**Motor state:** all 17 motors powered and verified torque-FREE before start
**Duration:** 1800 s inference, 2087 s logging (ramp + steady + cooldown)

## Workload
`rlodhi/smolvla_screw_picking_5000` fp16 via `smolvla_benchmark_jetson.py`,
/dev/video2 (RealSense RGB) 640x480, **chunk 5, num_steps 10** — matching
run_2 and run_4 exactly.

## Results
| Metric | Value |
|---|---|
| GPU temp min / mean / max | 49.3 / 57.2 / **59.9 C** |
| Rise above ambient | +35.5 C |
| Max CPU | 58.7 C |
| VDD_IN mean / peak | 9.30 W / 12.02 W |
| GPU clock min / mean / max | 306 / 423 / 1020 MHz (ceiling 1020) |
| Forward passes | 1729 |
| Inference latency | 1033.2 ms (1018.3–1102.7) = **0.97 Hz replan** |
| Battery used | 6% (100% → 94%) |
| **Throttle** | **N** |

## Battery series
| Elapsed | SoC |
|---|---|
| t+0 s | 100% |
| t+370 s | 97% |
| t+634 s | 96% |
| t+781 s | 95% |
| — | 94% |

**Do not derive a drain rate from this run.** The pack held 100% through 935 s of
continuous ~10 W draw, then shed 3% in 370 s. The C300's SoC readout is
non-linear near full charge, so rates measured in this window are unusable.
Runs 2 and 4, both measured mid-range, are the defensible figures.
See `../SUMMARY_n3.md`.

## Throttle verdict: N
Latency held flat across the run, the GPU reached its full 1020 MHz ceiling
throughout, and peak temperature stayed ~25 C below throttle onset. Valid at
24.4 C ambient.

## Note on control frequency
This run used `chunk 5`, the benchmark script's default, giving ~4.8 Hz control.
That is a **script default, not a platform limit** — the checkpoint's native
`chunk_size` is 50, which yields ~55 Hz control at identical latency. Any
control-rate figure from this run must be qualified as "as configured".
See `../../optimisation_sweep/` and `protocols/C2_inference_optimization.md`.

## Files
- `log/peak-inference_*/samples.csv` — 1 Hz temps, util, GPU clock, power, RAM
- `log/peak-inference_*/annotations.csv` — battery SoC, ambient
- `log/peak-inference_*/tegrastats_raw.log` — unparsed source
- `bench.log` — per-pass latency
- `PRERUN_STATE.txt` — pre-run temps, clocks, torque state

## Provenance note
This file was generated on 2026-08-02 from the run's own raw data, after the
original write step was missed. All figures are computed from
`samples.csv`, `bench.log` and `annotations.csv` in this directory.

## Known gaps
- Ambient is a single spot reading; RH not measured.
- Battery drain unusable (see above).
