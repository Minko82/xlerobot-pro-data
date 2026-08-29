# A2 Phase 1 — Active Idle — Run 1

**Date:** 2026-08-02 UTC
**Platform:** Jetson Orin Nano Super 8 GB, JetPack 6.2.1 (L4T 36.4.7)
**Power mode:** MAXN_SUPER · **Clock policy:** DVFS (306-1020 MHz)
**Supply:** Anker C300, 12 V car socket (untethered)
**Ambient:** 24.4 C (76 F)
**Motor state:** all 17 motors powered and verified torque-FREE
**Workload:** NONE — no inference, no motor buses held open, no project processes
**Duration:** 3579 s (59.7 min)

## Results
| Metric | Value |
|---|---|
| GPU temp min / mean / max | 48.8 / 49.5 / **51.0 C** |
| Rise above ambient | +26.6 C |
| Max CPU / tj | 49.9 / 51.0 C |
| VDD_IN mean / peak | **5.43 W** / 5.69 W |
| GPU clock mean / max | 306 / 306 MHz (never left minimum) |
| RAM mean | 1458 MB |
| Battery used | **4% over 57.2 min** (~4-5 %/h) |
| **Throttle** | **N** (no load to throttle) |

## Battery series
| Elapsed | SoC |
|---|---|
| t+0 s | 81% |
| t+890 s | 80% |
| t+1494 s | 79% |
| t+2162 s | 78% |
| t+3434 s | 77% |

Interval rates: 4.04, 5.96, 5.39, 2.83 %/h. Span estimate 4.19 %/h over the full
57.2 min. Whole-percent SoC resolution limits precision; **~4-5 %/h** is the
defensible range.

## Interpretation
Thermally flat: **2.2 C total spread over 58 minutes**, GPU clock never left its
306 MHz minimum. This is the platform's idle floor at 24.4 C ambient.

## Comparison against peak inference (n=3)
| Metric | Active idle | Peak inference | Delta |
|---|---|---|---|
| Max GPU | 51.0 C | 59.97 C | +9.0 C |
| Rise above ambient | +26.6 | +35.57 | +9.0 |
| VDD_IN mean | 5.43 W | 9.49 W | **+75%** |
| GPU clock mean | 306 MHz | 426 MHz | +39% |
| Battery drain | ~4-5 %/h | 6.4-8.0 %/h | **+30-60%** |

**The Jetson's own consumption rises 75% under inference, but total battery drain
rises only 30-60%.** The gap is the servo rail and peripherals drawing a roughly
constant load regardless of compute state. On this platform idle actuator draw
costs more than the policy does — releasing torque on unused joints between
manipulations would buy more runtime than inference optimisation.

## Known gaps
- Ambient is a single spot reading; RH not measured.
- Whole-percent SoC is coarse; a clamp meter on the 12 V rail would be better.
- n = 1. Two more runs needed for parity with peak inference.
