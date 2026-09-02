# Sec. VI-E draft — closed-loop learned grasping on the untethered platform

Numbers below are from `results.csv`, `tegrastats.log` and the per-trial logs in
this directory. Every number is measured, not commanded. Wording is a draft;
the tables are ready to paste.

## Text

### Setup

The learned stage is an Action Chunking Transformer (ACT) [Zhao et al. 2023]
trained with LeRobot from 49 kinesthetic demonstrations of a single task:
grasp a 100 ml glass bottle from a shelf. Observations are one fixed 640×480
RGB frame from the neck camera and the six joint positions of the left arm;
actions are absolute joint targets, predicted in chunks of 100 at 30 Hz and
executed open loop for the full chunk (3.3 s) before the next inference.
Training ran on the robot's own Jetson Orin Nano (10 000 optimisation steps,
batch 4, ResNet-18 backbone, 0.34 s per step). Inference also runs on the
Orin, untethered, on battery, with the firmware envelope of Table III applied
to every joint.

Kinesthetic demonstration places the demonstrator's hand in the camera's view
on every frame of the reach, and a policy trained on the raw video learned to
read the hand rather than the bottle: without a hand present it closed the
gripper at the first joint configuration that resembled a grasp, short for far
bottles and long for near ones (Sec. VI-E.1, ablation in the supplementary
material). We therefore freeze the visual observation at the first frame of
each episode, in training and at deployment: the image carries the bottle's
position and the joint state carries the arm. The demonstrations, the network
and the training recipe are otherwise unchanged.

### Protocol

Fifty consecutive trials at one marked bottle position, no cooldown between
trials, the operator returning the bottle to its mark and the arm to its
reference pose between trials (mean cycle ≈ 2 min including handling). A trial
is a 25 s run of the policy; success is a grasp that lifts the bottle and holds
it until the run ends. Failures are attributed to one stage: perceive,
approach, grasp, transit, place, or a system event (servo overload latch,
compute reset, thermal abort). Servo temperatures are read before and after
every trial; SoC temperature and input power are logged at 0.2 Hz across the
block; realised control rate and inference time are recorded per trial.

### Results

The policy succeeded in 49 of 50 trials (98 %; Wilson 95 % CI 89.5–99.6 %). The
single failure (trial 43) was at the grasp stage: the jaws closed on the bottle
and it slipped out during the lift; the arm reached the same pose as in the
successful trials (Table VI-E-1). No approach, perception, or system failures
occurred. Success did not decay with trial index.

The reach was repeatable: across the 50 trials the joint configuration at the
moment the gripper command was issued had a standard deviation of 1.2 units on
shoulder pan (≈1.4°) and 2.2 units on wrist flex, and the time from start to
close was 9.38 ± 0.16 s. Shoulder lift and elbow varied more (4.2 and 7.0
units) because the demonstrations contain two arm configurations for the same
bottle position and the policy reproduces both.

Sustained closed-loop operation cost the platform little: the realised control
rate was 28.60 ± 0.03 Hz against a 30 Hz target (the loop is bound by the
camera), a full chunk inference took 104.5 ± 3.2 ms (max 111 ms) after a
one-time 0.56 s warm-up per run, and the SoC drew 6.27 W on average (7.80 W
peak) at 46 °C mean, 56 °C maximum. Arm servo temperatures were flat across
the block (shoulder lift 34–36 °C at every trial start); only the gripper
warmed, from 37 to 44 °C, from holding the bottle. At this duty cycle the arm
does not approach the 65 °C ceiling that bounds the loaded holds of Sec. VI-C,
so the thermal limit characterised there is not reached by pick-and-lift
alone; it is the drive bus and sustained payload, tested next, that load it.

### Limitations

One bottle position, one lighting condition (blinds closed, afternoon), one
object. Trials at other positions the same afternoon (n = 6) showed a failure
mode specific to this policy's training images, in which the bottle's body
falls under a masked corner of the frame; it is removed by training on
unmasked first frames, which was not done for the reported block. Base motion
and transit were not exercised because the drive bus adapter was unavailable.

## Tables

Table VI-E-1: Block A, 50 consecutive untethered trials.

```latex
\begin{table}[t]
\centering
\caption{Closed-loop grasping, 50 consecutive untethered trials at one position, no cooldown.}
\label{tab:blocka}
\begin{tabular}{lr}
\toprule
Trials / successes & 50 / 49 (98\%, 95\% CI 89.5--99.6\%) \\
Failure stages (perc./appr./grasp/transit/system) & 0 / 0 / 1 / -- / 0 \\
Time to gripper close & $9.38 \pm 0.16$ s \\
Close pose SD, pan / lift / elbow / wrist (units of 200 per range) & 1.2 / 4.2 / 7.0 / 2.2 \\
Realised control rate & $28.60 \pm 0.03$ Hz (30 Hz target) \\
Chunk inference (100 steps, ResNet-18 ACT, Orin Nano) & $104.5 \pm 3.2$ ms (max 111) \\
First inference per run (warm-up) & $564 \pm 14$ ms \\
Open-loop horizon & 100 steps, 3.3 s \\
Input power, mean / peak & 6.27 / 7.80 W \\
SoC temperature, mean / peak & 46.1 / 56.3 $^\circ$C \\
Shoulder-lift temperature, trial 1 / trial 50 / max & 34 / 35 / 36 $^\circ$C \\
Gripper temperature, trial 1 / trial 50 / max & 37 / 42 / 44 $^\circ$C \\
\bottomrule
\end{tabular}
\end{table}
```

Table VI-E-2 (optional): what changes with the frozen observation, same 49
demonstrations, same network and steps. From `../DIAGNOSIS_2026-09-02.md` and
`../v7_040000/README.md`; the v7 numbers are the morning's daylight trials.

```latex
\begin{tabular}{lcc}
\toprule
 & raw video, hand masked (v7) & first frame only (v8) \\
\midrule
Plan error on held-out reach frames, lift (units) & 3.5 & 7.1 \\
Plans a close from a mid-reach live frame & yes (step 0) & no \\
Hardware grasps, single position & 0 / 6 & 49 / 50 \\
\bottomrule
\end{tabular}
```

## Figure suggestions

1. Trial index (x) against shoulder-lift start temperature and cumulative
   success (two y-axes) — flat line at 98 % with temperature flat at 34–36 °C;
   this is the panel the protocol asked for, and its message is "no decay at
   this duty".
2. Close-pose scatter (pan vs lift) for the 50 trials, coloured by outcome,
   with the two demonstration styles visible as two clusters.
3. One trial's timeline: chunk boundaries at 0/100/200 steps, the planned close
   step at each re-plan (`tNN_plans.csv`), and the executed gripper command.
