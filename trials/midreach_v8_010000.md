# /home/xle/xlerobot-pro-data/outputs/act_glassbottle_pick_v8/checkpoints/010000/pretrained_model   49 episodes

onset/arrive/grasp frames: median 51 / 199 / 276

## variant `base`

| frame | n | never closes | planned close vs true frames-to-close (median) | signed err pan / lift / elbow / wflex (mean) | abs err lift / elbow (mean) | corr(err_lift, true_lift) | slope err_lift~true_lift |
|---|---|---|---|---|---|---|---|
| start | 49 | 49 | — | -2.1 / -47.2 / +24.8 / +26.3 | 47.2 / 25.8 | -0.38 | -0.29 |
| f0.00 | 49 | 49 | — | -1.1 / -15.5 / +8.9 / +8.3 | 16.6 / 11.5 | +0.03 | +0.02 |
| f0.25 | 49 | 49 | — | -0.8 / -2.9 / +1.5 / +1.3 | 8.1 / 7.8 | +0.46 | +0.20 |
| f0.50 | 49 | 47 | 66 vs 129 | -0.7 / -1.2 / -0.2 / +0.0 | 7.1 / 8.4 | +0.80 | +0.24 |
| f0.75 | 49 | 25 | 71 vs 106 | -0.8 / -1.0 / +0.3 / -0.9 | 6.1 / 8.2 | +0.88 | +0.24 |
| f1.00 | 49 | 5 | 49 vs 76 | -0.5 / -0.8 / -0.1 / -0.5 | 6.0 / 7.5 | +0.93 | +0.24 |
| dwell | 49 | 4 | 46 vs 38 | -0.4 / -0.3 / -0.7 / -0.4 | 5.9 / 7.5 | +0.92 | +0.24 |
| close-5 | 49 | 3 | 47 vs 5 | -0.3 / -0.2 / -0.9 / -0.4 | 5.7 / 7.3 | +0.89 | +0.23 |

  f0.50 near (true lift < median): n=24 signed err pan/lift/elbow/wflex = -1.5 / -7.2 / +5.8 / +5.1; never closes 23
  f0.50 far (true lift >= median): n=25 signed err pan/lift/elbow/wflex = +0.0 / +4.5 / -5.9 / -4.9; never closes 24

## variant `bright`

| frame | n | never closes | planned close vs true frames-to-close (median) | signed err pan / lift / elbow / wflex (mean) | abs err lift / elbow (mean) | corr(err_lift, true_lift) | slope err_lift~true_lift |
|---|---|---|---|---|---|---|---|
| start | 49 | 49 | — | -2.0 / -47.0 / +25.2 / +25.9 | 47.0 / 26.2 | -0.38 | -0.28 |
| f0.00 | 49 | 49 | — | -1.0 / -15.4 / +9.2 / +8.1 | 16.3 / 11.8 | +0.02 | +0.02 |
| f0.25 | 49 | 49 | — | -0.6 / -3.0 / +1.7 / +1.4 | 7.8 / 8.1 | +0.44 | +0.19 |
| f0.50 | 49 | 47 | 66 vs 129 | -0.5 / -1.5 / -0.1 / +0.3 | 6.7 / 8.6 | +0.79 | +0.23 |
| f0.75 | 49 | 26 | 67 vs 106 | -0.6 / -1.3 / +0.4 / -0.6 | 5.7 / 8.4 | +0.88 | +0.22 |
| f1.00 | 49 | 7 | 48 vs 76 | -0.3 / -0.9 / -0.0 / -0.2 | 5.7 / 7.6 | +0.93 | +0.23 |
| dwell | 49 | 5 | 48 vs 38 | -0.2 / -0.5 / -0.7 / -0.1 | 5.6 / 7.6 | +0.91 | +0.22 |
| close-5 | 49 | 4 | 48 vs 5 | -0.2 / -0.3 / -0.9 / -0.1 | 5.4 / 7.4 | +0.89 | +0.22 |

  f0.50 near (true lift < median): n=24 signed err pan/lift/elbow/wflex = -1.3 / -7.3 / +5.8 / +5.0; never closes 23
  f0.50 far (true lift >= median): n=25 signed err pan/lift/elbow/wflex = +0.2 / +4.1 / -5.8 / -4.3; never closes 24

## variant `dark`

| frame | n | never closes | planned close vs true frames-to-close (median) | signed err pan / lift / elbow / wflex (mean) | abs err lift / elbow (mean) | corr(err_lift, true_lift) | slope err_lift~true_lift |
|---|---|---|---|---|---|---|---|
| start | 49 | 49 | — | -2.4 / -47.4 / +25.2 / +26.2 | 47.4 / 26.2 | -0.40 | -0.30 |
| f0.00 | 49 | 49 | — | -1.4 / -15.9 / +9.4 / +8.4 | 16.8 / 11.9 | +0.02 | +0.02 |
| f0.25 | 49 | 49 | — | -1.0 / -3.0 / +1.8 / +1.3 | 8.0 / 8.0 | +0.45 | +0.20 |
| f0.50 | 49 | 47 | 68 vs 129 | -0.8 / -1.4 / +0.1 / +0.1 | 6.9 / 8.5 | +0.80 | +0.24 |
| f0.75 | 49 | 24 | 68 vs 106 | -0.9 / -1.2 / +0.6 / -0.8 | 5.9 / 8.3 | +0.88 | +0.23 |
| f1.00 | 49 | 5 | 49 vs 76 | -0.6 / -0.9 / +0.1 / -0.5 | 5.9 / 7.7 | +0.93 | +0.24 |
| dwell | 49 | 5 | 46 vs 38 | -0.5 / -0.4 / -0.6 / -0.4 | 5.9 / 7.7 | +0.92 | +0.23 |
| close-5 | 49 | 4 | 46 vs 5 | -0.5 / -0.3 / -0.8 / -0.4 | 5.7 / 7.4 | +0.88 | +0.22 |

  f0.50 near (true lift < median): n=24 signed err pan/lift/elbow/wflex = -1.7 / -7.3 / +6.1 / +5.0; never closes 23
  f0.50 far (true lift >= median): n=25 signed err pan/lift/elbow/wflex = -0.0 / +4.3 / -5.7 / -4.7; never closes 24

## live frames `trials/midreach_v8_010000_livestart.json`

| frame | state lift/elbow/grip | base: close, lift/elbow/wflex, max lift | bright: close, lift/elbow/wflex, max lift | dark: close, lift/elbow/wflex, max lift |
|---|---|---|---|---|
| v7t1 frame0 + state s0 | -100/+99/94 | None, -8/+83/-42, -1 | None, -9/+84/-39, -3 | None, -5/+82/-46, +2 |
| v7t1 frame0 + state s90 | -56/+97/58 | None, +38/+58/-70, +40 | None, +35/+58/-66, +36 | None, +43/+54/-73, +44 |
| v7t1 frame0 + state s120 | -11/+90/60 | None, +43/+56/-75, +44 | None, +40/+57/-71, +40 | None, +48/+52/-78, +48 |
| v7t1 frame0 + state s150 | +33/+60/57 | 62, +41/+58/-78, +42 | 64, +38/+58/-74, +40 | 72, +44/+55/-80, +45 |
| v7t1 frame0 + state s180 | +48/+47/57 | 27, +51/+51/-81, +52 | 27, +49/+51/-78, +50 | 27, +53/+49/-83, +54 |
| v7t5 frame0 + state s0 | -100/+99/94 | None, -16/+100/-41, -10 | None, -19/+100/-38, -13 | None, -14/+98/-45, -7 |
| v7t5 frame0 + state s90 | -43/+96/57 | None, +18/+79/-60, +19 | None, +14/+81/-56, +14 | None, +23/+76/-63, +23 |
| v7t5 frame0 + state s120 | -9/+96/58 | None, +15/+82/-62, +16 | None, +10/+85/-57, +10 | None, +20/+78/-65, +20 |
| v7t5 frame0 + state s150 | +5/+87/46 | 0, +20/+93/-72, +20 | 0, +19/+92/-70, +19 | 0, +21/+92/-74, +21 |
| v7t5 frame0 + state s180 | -4/+87/43 | 0, +18/+94/-72, +18 | 0, +18/+93/-69, +18 | 0, +18/+93/-74, +18 |
| v7t6 frame0 + state s0 | -100/+99/94 | None, -4/+85/-52, +3 | None, -7/+88/-50, +0 | None, -5/+88/-53, +3 |
| v7t6 frame0 + state s90 | -38/+94/58 | None, +43/+58/-78, +44 | None, +41/+60/-76, +42 | None, +41/+62/-78, +42 |
| v7t6 frame0 + state s120 | +6/+89/50 | 0, +10/+92/-75, +22 | 0, +12/+91/-76, +21 | 0, +10/+93/-76, +19 |
| v7t6 frame0 + state s150 | +14/+80/27 | 0, +31/+80/-81, +31 | 0, +34/+79/-80, +34 | 0, +31/+81/-80, +31 |
| v7t6 frame0 + state s180 | -13/+83/18 | 0, -16/+90/-54, -16 | 0, -14/+90/-54, -13 | 0, -17/+91/-54, -17 |
| base_t1 frame0 + state s0 | -100/+99/94 | None, +11/+76/-69, +19 | None, +10/+76/-68, +19 | None, +13/+74/-72, +23 |
| base_t1 frame0 + state s100 | -9/+82/61 | None, +67/+34/-95, +67 | None, +66/+34/-93, +66 | None, +68/+32/-95, +68 |
| base_t1 frame0 + state s200 | +13/+64/28 | 0, +26/+67/-79, +26 | 0, +26/+68/-79, +26 | 0, +25/+67/-80, +26 |
| cc_t1 frame0 + state s0 | -100/+100/93 | None, -4/+82/-53, +3 | None, -5/+82/-51, +3 | None, -2/+81/-56, +5 |
| cc_t1 frame0 + state s100 | -32/+87/60 | None, +57/+42/-85, +58 | None, +53/+42/-78, +53 | None, +59/+39/-85, +59 |
| cc_t1 frame0 + state s200 | +19/+78/60 | None, +63/+38/-91, +63 | None, +58/+39/-83, +58 | None, +64/+34/-91, +65 |

