# A2 Phase 1 — Active Idle — Summary (n = 3)

## Conditions (identical across all three runs)
- Jetson Orin Nano Super 8 GB, JetPack 6.2.1, MAXN_SUPER, DVFS (306-1020 MHz)
- Ambient 24.4 C; all 17 motors powered and torque-free
- No inference, no motor buses held open, no project processes
- Start GPU temp: 50.6 / 50.8 / 50.7 C (spread 0.2 C)

## Results

| Metric | run_1 | run_2 | run_3 | mean | range |
|---|---|---|---|---|---|
| GPU min (C) | 48.8 | 48.8 | 48.8 | 48.8 | **0.0** |
| GPU mean (C) | 49.5 | 49.9 | 50.0 | **49.8** | 0.5 |
| GPU max (C) | 51.0 | 51.2 | 51.1 | **51.1** | **0.2** |
| Rise above ambient (C) | +26.6 | +26.8 | +26.7 | **+26.7** | 0.2 |
| CPU max (C) | 49.9 | 50.2 | 50.0 | 50.0 | 0.3 |
| VDD_IN mean (W) | 5.43 | 5.44 | 5.44 | **5.437** | **0.01** |
| VDD_IN peak (W) | 5.69 | 5.81 | 5.73 | 5.74 | 0.12 |
| GPU clock (MHz) | 306 | 306 | 306 | 306 | 0 |
| RAM mean (MB) | 1458 | 1462 | 1458 | 1459 | 4 |
| Battery drain (%/h) | 4.19 | 4.88 | 5.16 | **4.74** | 0.97 |
| SoC window | 81->77% | 74->69% | 67->62% | — | — |

**Idle power reproduced within 0.01 W and peak GPU within 0.2 C across three
hour-long runs.** The GPU clock never left its 306 MHz minimum in any run.

Drain was measured in three non-overlapping SoC windows (81-77, 74-69, 67-62%),
so the agreement is not an artefact of sampling the same part of the discharge
curve. Spread (0.97 %/h) is dominated by whole-percent SoC resolution.

## Comparison: idle vs peak inference

| Metric | Active idle (n=3) | Peak inference (n=3) | Delta |
|---|---|---|---|
| Max GPU | 51.1 C | 59.97 C | **+8.9 C** |
| Rise above ambient | +26.7 | +35.57 | +8.9 |
| VDD_IN mean | 5.437 W | 9.49 W | **+75%** |
| GPU clock mean | 306 MHz | 426 MHz | +39% |
| Battery drain | 4.74 %/h | 6.4-8.0 %/h | **+35-69%** |
| Throttle | N | N | — |

## Interpretation

Sustained VLA inference raises the board only **8.9 C above its idle floor**, and
neither condition approaches throttling at 24.4 C ambient. The thermal envelope
is not the constraint on this platform; the ~1 Hz replan rate is.

The Jetson's own draw rises **75%** under inference (5.44 -> 9.49 W) while total
battery drain rises **35-69%**. The gap is servo-rail and peripheral load, which
is roughly constant regardless of compute state. Estimated from the delta, that
constant load is on the order of 14 W — more than the Jetson's entire idle
consumption.

**Implication:** on this platform, idle actuator draw exceeds the cost of running
the policy. Releasing torque on unused joints between manipulations would buy
more runtime than any inference optimisation.

## Known gaps
- Ambient is a single spot reading per run; RH not measured.
- Whole-percent SoC is coarse. Within-run interval rates scattered 2.8-6.0 %/h;
  only span-based estimates are meaningful. A clamp meter on the 12 V rail would
  replace this with a continuous measurement and is the single biggest
  improvement available to this phase.
