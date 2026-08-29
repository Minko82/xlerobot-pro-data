# A2 Phase 1 — Active Idle — Run 3

**Date:** 2026-08-02 UTC
**Platform:** Jetson Orin Nano Super 8 GB, JetPack 6.2.1 (L4T 36.4.7)
**Power mode:** MAXN_SUPER · **Clock policy:** DVFS (306-1020 MHz)
**Supply:** Anker C300, 12 V car socket (untethered)
**Ambient:** 24.4 C (76 F)
**Motor state:** all 17 motors powered and verified torque-FREE
**Workload:** NONE
**Duration:** 3578 s (59.6 min)

## Results
| Metric | Value |
|---|---|
| GPU temp min / mean / max | 48.8 / 50.0 / **51.1 C** |
| Rise above ambient | +26.7 C |
| Max CPU | 50.0 C |
| VDD_IN mean / peak | **5.44 W** / 5.73 W |
| GPU clock mean / max | 306 / 306 MHz |
| RAM mean | 1458 MB |
| Battery used | **5% over 58.2 min** (5.16 %/h) |
| **Throttle** | **N** |

## Battery series
| Elapsed | SoC |
|---|---|
| t+0 s | 67% |
| t+233 s | 67% |
| t+1652 s | 65% |
| t+2628 s | 64% |
| t+3490 s | 62% |

SoC window 67->62%, distinct from run_1 (81->77%) and run_2 (74->69%).
