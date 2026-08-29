# XLeRobot-Pro — experimental data

Research on the robot: measurement scripts, raw telemetry, and derived results.

The [`xlerobot-pro`](../xlerobot-pro) repository is the product — the library and
tools for *operating* the robot. This directory is everything that measures it.
The split exists so a user setting up the robot never has to wade through
experiment tooling, and so raw telemetry never lands in the source tree.

The manuscript itself is not kept here.

## Layout

| Path | Contents |
| --- | --- |
| `scripts/` | Measurement tooling for the protocol (A1, A2, B1, C2) |
| `protocols/` | Written protocols for the experiments |
| `A2/` | Thermal endurance and actuator-load telemetry |
| `B1/` | Payload-under-motion telemetry (arm slew and base translation) |
| `thermal/` | `tegrastats` captures — GPU/CPU temperature, clocks, VDD_IN power |
| `calibration/` | Reference poses and camera offsets used by the experiments |
| `*.md` | Protocol notes, results tables and the outstanding test plan |
| `backup.sh` | Pull everything off the robot |

## Running the scripts

They import `xlerobot_pro`, so the main repository must be installed first
(`pip install -e ".[all]"` from there). They also expect the udev symlinks
`/dev/xle_arms` and `/dev/xle_head` — create them with `diagnostics/detect_buses.py`
in the main repo.

| Script | Test | What it does |
| --- | --- | --- |
| `a1_brownout.py` | A1 | Enable every actuator in sequence, log the compute rail at ~760 Hz, then command a worst-case simultaneous pose. Detects brownout resets via kernel boot id. Compute-rail voltage comes from the onboard INA3221, not an oscilloscope, so reported V_min is an upper bound |
| `log_thermal_power.py` | A2 | Wrap `tegrastats`; log GPU/CPU temperature, clocks and VDD_IN power. Runs alongside any other test |
| `hold_pose_thermal.py` | A2 | Hold a bimanual pose under sustained load, logging per-servo telemetry. Replays a saved reference pose so every load in a sweep shares one geometry. Aborts at the servo temperature ceiling |
| `slew_payload_test.py` | B1 | Sweep two arm joints through a sinusoidal trajectory with a grasped payload. Base stationary |
| `b1_base_payload.py` | B1 | Drive the base at a target acceleration while the arm holds a payload. Legs alternate direction so the robot stays near its start |
| `b1_slip_from_video.py` | B1 | Measure slip in millimetres from phone video using two ArUco markers. Needs OpenCV with `aruco` |
| `a1_dynamic.py` | A1 | Drive the base and slew both arms *in phase*, so peak arm lift lands on peak wheel inrush. The static sequence parks goals on present position and measures almost nothing |
| `c1_unsized_failure.py` | C1 | Escalate one limit at a time past the documented envelope until something breaks, and classify what broke (RESET / SAG / LATCH / THERMAL / DARK / OVERCUR). Trips at 98% of each bus's own fuse |
| `record_kinesthetic.py` | C1 | Record demonstrations by posing the arm by hand. Writes a standard `LeRobotDataset`; the action for frame *t* is the state at *t+1* |
| `policy_trials.py` | C1 | Trial harness for closed-loop policy runs: N, success rate, per-stage failure attribution, realized control rate, joint temperature per trial |
| `policy_preflight.py` | C1 | Check dependencies, buses, calibration, camera and disk before a recording session, then print the record/train/trial commands |
| `camera_preview.py` | — | Serve the camera over HTTP through the same RealSense path the recorder uses, so what you frame is what gets written |

## Method notes that took real time to learn

- **Never re-baseline `Goal_Position` to where a joint settled.** A position-mode
  servo holds by applying torque proportional to position error; zeroing that
  error makes it stop resisting gravity and the arm visibly drops.
- **Report realized, not commanded.** The base delivers ~62% of commanded
  acceleration under payload (95% unloaded), and `shoulder_lift` realizes 1% of a
  commanded slew amplitude with 366 g on the gripper. Commanded values are not
  measurements.
- **Fit, do not differentiate.** Recovering acceleration by double-differentiating
  1-count encoder data at 20 Hz overestimated it by 13×. Fit the known trajectory,
  or take Δv over a ramp segment.
- **Separate slip from creep.** A discrete >5-count jump in one 1 Hz sample is
  mechanical slip; smooth motion is creep. Conflating them overstated creep at
  800 g by 3×.
- **Never run on the charger.** Pass-through charging changes the supply the
  servos see: the same 800 g load ran 2 °C cooler, 8% lower current and half the
  drift. The tell is a `bat` annotation that does not move.
- **Battery percentage is coarse.** State-of-charge is whole-percent and
  non-linear near full — the pack held 100% through 935 s of ~10 W draw, then shed
  3% in 370 s. Measure current at the rail where it matters.
