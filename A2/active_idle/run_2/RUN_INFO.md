# A2 Phase 1 — Active Idle — Run 2

**Date:** 2026-08-02 UTC
**Platform:** Jetson Orin Nano Super 8 GB, JetPack 6.2.1 (L4T 36.4.7)
**Power mode:** MAXN_SUPER · **Clock policy:** DVFS (306-1020 MHz)
**Supply:** Anker C300, 12 V car socket (untethered)
**Ambient:** 24.4 C (76 F)
**Motor state:** all 17 motors powered and verified torque-FREE
**Workload:** NONE — no inference, no motor buses held open
**Duration:** 3579 s (59.7 min)

## Results
| Metric | Value |
|---|---|
| GPU temp min / mean / max | 48.8 / 49.9 / **51.2 C** |
| Rise above ambient | +26.8 C |
| Max CPU / tj | 50.2 / 51.2 C |
| VDD_IN mean / peak | **5.44 W** / 5.81 W |
| GPU clock mean / max | 306 / 306 MHz (never left minimum) |
| RAM mean | 1462 MB |
| Battery used | **5% over 61.5 min** (4.88 %/h) |
| **Throttle** | **N** (no load to throttle) |

## Battery series
| Elapsed | SoC |
|---|---|
| t+0 s | 74% |
| t+2040 s | 71% |
| t+2911 s | 70% |
| t+3691 s | 69% |

## Note
Measured 74->69%, a different window of the discharge curve than run_1's
81->77%. The two agree closely, which supports treating idle drain as roughly
constant across this part of the pack's range.
