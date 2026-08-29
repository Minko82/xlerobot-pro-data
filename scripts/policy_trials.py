#!/usr/bin/env python3
"""Trial harness for closed-loop policy runs.

The paper needs more than "the policy worked": it needs N, a success rate, the
stage each failure happened at, the realized control rate, and joint temperature
against trial index so the thermal-degradation claim can be checked.  This runs
the policy as a subprocess, one trial at a time, and records all of that.

Joint temperature is sampled *between* trials rather than during them.  The
policy process owns the arms serial port while it runs, so a second reader
cannot open it; before/after readings per trial give the temperature-vs-trial
series without fighting for the bus.

    python scripts/policy_trials.py --out A2/policy_trials/act_pick_20260820 \\
        --trials 20 --task "pick red cube, place in bin" \\
        --policy act --note "single arm, fixed start pose" \\
        --cmd "python ../xlerobot-pro/examples/policies/diffusion_policy_control.py run --duration 30"

Per trial you are prompted for the outcome and, on failure, the stage it broke
at.  Everything lands in trials.csv plus run_info.json for provenance.
"""
import argparse
import csv
import json
import shlex
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from xlerobot_pro.config import ARMS_PORT

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex",
          "wrist_flex", "wrist_roll", "gripper"]
RETRY = 5
STAGES = ["perception", "reach", "grasp", "transport", "place"]
FIELDS = ["trial", "started", "condition", "outcome", "failure_stage",
          "seconds", "steps", "realized_hz", "temp_before_c", "temp_after_c",
          "peak_temp_c", "note"]


def arms_bus():
    motors = {f"left_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES)
              for j, i in zip(JOINTS, range(1, 7))}
    motors |= {f"right_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES)
               for j, i in zip(JOINTS, range(7, 13))}
    return FeetechMotorsBus(port=ARMS_PORT, motors=motors)


def read_temps(bus) -> dict:
    """Per-joint case temperature, degrees C. Bus must be free."""
    bus.connect()
    try:
        return dict(bus.sync_read("Present_Temperature", num_retry=RETRY))
    finally:
        bus.disconnect()


def ask(prompt: str, options: list[str], default: str | None = None) -> str:
    opts = "/".join(options)
    while True:
        raw = input(f"{prompt} [{opts}]"
                    f"{f' (default {default})' if default else ''}: ").strip().lower()
        if not raw and default:
            return default
        for o in options:
            if raw == o or raw == o[0]:
                return o
        print(f"  please answer one of: {opts}")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--trials", type=int, default=20)
    p.add_argument("--cmd", required=True,
                   help="Command that runs ONE trial and exits.")
    p.add_argument("--policy", required=True,
                   help="Policy identifier as it will appear in the paper.")
    p.add_argument("--task", required=True)
    p.add_argument("--checkpoint", default="", help="Checkpoint path or hub id.")
    p.add_argument("--concurrent-cmd", default="",
                   help="Run alongside each trial and stop when it ends, e.g. a "
                        "base-motion script. Wheels sit on the head port and the "
                        "arms on their own, so the two do not contend.")
    p.add_argument("--condition", default="static",
                   help="Label for this block of trials, e.g. static / mobile.")
    p.add_argument("--cooldown", type=float, default=0.0,
                   help="Seconds to wait between trials.")
    p.add_argument("--battery", type=float, default=None,
                   help="Powerbank state of charge, percent.")
    p.add_argument("--ambient-c", type=float, default=None)
    p.add_argument("--note", default="")
    p.add_argument("--no-temps", action="store_true",
                   help="Skip the servo readings (no bus access).")
    args = p.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    bus = None if args.no_temps else arms_bus()

    (args.out / "run_info.json").write_text(json.dumps({
        "started": datetime.now(timezone.utc).isoformat(),
        "policy": args.policy, "checkpoint": args.checkpoint,
        "task": args.task, "trials_planned": args.trials,
        "condition": args.condition, "command": args.cmd,
        "concurrent_command": args.concurrent_cmd, "battery_pct": args.battery,
        "ambient_c": args.ambient_c, "note": args.note,
    }, indent=2) + "\n")

    csv_path = args.out / "trials.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        ok = 0
        for n in range(1, args.trials + 1):
            print(f"\n=== trial {n}/{args.trials} — reset the scene, then Enter "
                  f"(or 'q' to stop) ===")
            if input().strip().lower() == "q":
                break

            before = peak = None
            if bus is not None:
                temps = read_temps(bus)
                before = peak = max(temps.values())
                print(f"  joints at {before:.0f} C")

            side = None
            if args.concurrent_cmd:
                side = subprocess.Popen(shlex.split(args.concurrent_cmd))
                print(f"  concurrent load running (pid {side.pid})")

            t0 = time.time()
            try:
                proc = subprocess.run(shlex.split(args.cmd))
                if proc.returncode != 0:
                    print(f"  ! policy exited {proc.returncode}")
            finally:
                seconds = time.time() - t0
                if side is not None and side.poll() is None:
                    # b1_base_payload stops the wheels on SIGTERM, so this is
                    # the safe way down; kill only if it ignores us
                    side.terminate()
                    try:
                        side.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        side.kill()
                        print("  ! concurrent load had to be killed")

            after = None
            if bus is not None:
                temps = read_temps(bus)
                after = max(temps.values())
                peak = max(peak, after)
                print(f"  joints at {after:.0f} C after {seconds:.0f} s")

            outcome = ask("  outcome", ["success", "failure", "void"])
            stage = ""
            if outcome == "failure":
                stage = ask("  failed at", STAGES)
            steps = input("  steps executed (blank if unknown): ").strip()
            note = input("  note (optional): ").strip()
            hz = ""
            if steps.isdigit() and seconds > 0:
                hz = f"{int(steps) / seconds:.2f}"
                print(f"  realized {hz} Hz")
            ok += outcome == "success"

            writer.writerow({
                "trial": n,
                "started": datetime.fromtimestamp(t0, timezone.utc).isoformat(),
                "condition": args.condition, "outcome": outcome, "failure_stage": stage,
                "seconds": f"{seconds:.1f}", "steps": steps, "realized_hz": hz,
                "temp_before_c": before if before is None else f"{before:.0f}",
                "temp_after_c": after if after is None else f"{after:.0f}",
                "peak_temp_c": peak if peak is None else f"{peak:.0f}",
                "note": note,
            })
            handle.flush()
            print(f"  running: {ok}/{n} success")

            if args.cooldown and n < args.trials:
                print(f"  cooling {args.cooldown:.0f} s...")
                time.sleep(args.cooldown)

    print(f"\nwrote {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
