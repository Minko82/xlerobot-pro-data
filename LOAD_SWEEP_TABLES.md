### A2 Phase 3 — sustained bimanual hold, load sweep

Load suspended on a thread from the right arm; left arm is the unloaded paired
control. Measured on `right_shoulder_lift`. Ambient 24.4 C. Powerbank on
battery (never charging). 1 count = 0.0879 deg = 0.285 mm at the 18.6 cm moment arm.

| Load (g) | Settle (counts) | Settle (mm) | Current mean | Torque | % of limit | Peak R (C) | Peak L (C) | dT gap (C) | Drift (counts) | Drift (mm) | Slips | Duration (min) | Outcome |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 100 | +18 | 5.1 | 1.5 | 50 | 11% | 37 | 39 | -2 | +0 | 0.0 | 0 | 30.0 | completed 30 min |
| 200 | +26 | 7.4 | 2.9 | 67 | 15% | 39 | 39 | +0 | +1 | 0.3 | 0 | 30.0 | completed 30 min |
| 300 | +35 | 10.0 | 4.5 | 86 | 19% | 41 | 38 | +3 | +7 | 2.0 | 1 | 30.0 | completed 30 min |
| 400 | +43 | 12.3 | 6.4 | 103 | 23% | 43 | 38 | +5 | +2 | 0.6 | 0 | 30.0 | completed 30 min |
| 500 | +52 | 14.8 | 9.9 | 129 | 29% | 48 | 39 | +9 | +8 | 2.3 | 0 | 30.0 | completed 30 min |
| 600 | +79 | 22.5 | 14.0 | 177 | 39% | 52 | 37 | +15 | +4 | 1.1 | 0 | 30.0 | completed 30 min |
| 700 | +80 | 22.8 | 20.2 | 186 | 41% | 60 | 38 | +22 | +10 | 2.8 | 0 | 30.0 | completed 30 min |
| 800 | +91 | 25.9 | 25.6 | 217 | 48% | 65 | 38 | +27 | +36 | 10.3 | 2 | 27.8 | 65 C ceiling |
| 900 | +92 | 26.2 | 29.3 | 254 | 56% | 65 | 35 | +30 | +42 | 12.0 | 2 | 22.1 | 65 C ceiling |
| 1000 (t1) | +101 | 28.8 | 31.8 | 271 | 60% | 61 | 37 | +24 | +707 | 201.5 | 2 | 13.9 | mechanical failure |
| 1000 (t2) | +108 | 30.8 | 33.2 | 247 | 55% | 55 | 35 | +20 | +739 | 210.6 | 1 | 8.8 | mechanical failure |

### Endurance — time from 35 C to the 65 C ceiling

Normalised to a 35 C start so runs beginning at different temperatures are
comparable. Every ceiling run passes through 35 C on the way up.

| Load (g) | Start temp (C) | Time 35->65 C (min) |
|---|---|---|
| 100 | 37 | >30 (ceiling not reached) |
| 200 | 36 | >30 (ceiling not reached) |
| 300 | 37 | >30 (ceiling not reached) |
| 400 | 37 | >30 (ceiling not reached) |
| 500 | 33 | >30 (ceiling not reached) |
| 600 | 36 | >30 (ceiling not reached) |
| 700 | 33 | >30 (ceiling not reached) |
| 800 | 33 | 26.4 |
| 900 | 29 | 19.6 |
| 1000 (t1) | 34 | n/a - fails mechanically |
| 1000 (t2) | 35 | n/a - fails mechanically |

### Discrete slip events

Position jumps greater than 5 counts within one 1 Hz sample, distinguished
from smooth creep. Slip appears only at 800 g and above.

| Load (g) | Total drift (counts) | From slips | From creep | Slip events |
|---|---|---|---|---|
| 100 | +0 | +0 | +0 | none |
| 200 | +1 | +0 | +1 | none |
| 300 | +7 | +6 | +1 | +6 at 28.8 min |
| 400 | +2 | +0 | +2 | none |
| 500 | +8 | +0 | +8 | none |
| 600 | +4 | +0 | +4 | none |
| 700 | +10 | +0 | +10 | none |
| 800 | +36 | +24 | +12 | +7 at 22.4 min; +17 at 25.6 min |
| 900 | +42 | +20 | +22 | +12 at 5.1 min; +8 at 6.1 min |
| 1000 (t1) | +707 | +682 | +25 | +24 at 4.4 min; +658 at 13.7 min |
| 1000 (t2) | +739 | +727 | +12 | +727 at 8.7 min |

### Notes for the writeup

- **Slip vs creep.** Discrete slip (a >5 count jump within one 1 Hz sample) appears
  only at 800 g and above. The single +6 event at 300 g / 28.8 min is marginal and
  sits at the detection threshold; treat it as noise rather than a slip.
- **Separating the two changes the creep conclusion.** Total drift at 800 g is +36
  counts, but only +12 of that is smooth creep. True creep across the sweep is
  +0, +1, +1, +2, +8, +4, +10, +12, +22 counts for 100-900 g -- gradual, not
  runaway. The dramatic drift figures at high load are mechanical slippage.
- **The 1 kg failure is progressive, not a threshold.** Two trials under matched
  conditions failed earlier (13.9 -> 8.8 min), cooler (61 -> 55 C) and at lower
  torque (271 -> 247), with settle degrading +101 -> +108. Torque before each slip
  was only 55-60% of the configured `Torque_Limit` (450), so the limit was not the
  cause; saturation to 450 is a *consequence* of the position error after the drop.
- **Peak temperature is censored at 800 g and above** by the 65 C abort. Use
  time-to-ceiling above that, not peak.
- **Torque headroom vs thermal headroom.** At 700 g the joint is at 41% of its
  torque budget but 92% of the thermal ceiling -- the platform is thermally limited
  through 900 g, then mechanically limited at 1 kg.
- **Excluded from this table:** `load_400g_FAILED_SETUP` (failed ramp),
  `load_800g_CHARGING` (powerbank on charger -- see STATE.md for the comparison),
  and the `load_500g_*` n=3 series (different thread position, so a separate
  condition rather than a tenth sweep point).
