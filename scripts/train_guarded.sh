#!/usr/bin/env bash
# Run lerobot-train with tegrastats logged beside it and a thermal watchdog.
#
# The 31 August run died at step 53 000 of 60 000 on a thermal shutdown with no
# telemetry and a volatile journal; nothing said when or how hot. This wrapper
# writes tegrastats every 5 s to <output_dir>-guard/tegrastats.log and kills the run
# if the SoC junction temperature stays at or above TJ_MAX for three samples.
# A killed run resumes from its last checkpoint with --resume=true; a thermal
# shutdown loses everything since the last checkpoint and possibly the disk.
#
#   bash scripts/train_guarded.sh <output_dir> <job_name> [lerobot-train args...]
#
# Example:
#   bash scripts/train_guarded.sh outputs/act_glassbottle_pick_v7 act_glassbottle_pick_v7 \
#     --dataset.repo_id=local/glassbottle_pick_v7_masked --policy.type=act \
#     --policy.device=cuda --policy.push_to_hub=false --steps=40000 --save_freq=10000 \
#     --batch_size=4 --num_workers=2
set -u
OUT="$1"; JOB="$2"; shift 2
TJ_MAX="${TJ_MAX:-92}"
PY="${PY:-$HOME/.venvs/xlerobot-pro/bin}"
# lerobot-train refuses an output_dir that already exists, so the guard's own
# logs live beside it, not inside it.
GUARD="${OUT%/}-guard"; mkdir -p "$GUARD"
LOG="$GUARD/train.log"; TS="$GUARD/tegrastats.log"

tegrastats --interval 5000 > "$TS" 2>&1 &
TS_PID=$!
echo "tegrastats -> $TS (pid $TS_PID)   tj ceiling ${TJ_MAX} C   started $(date)" | tee -a "$LOG"

"$PY/lerobot-train" --output_dir="$OUT" --job_name="$JOB" "$@" >> "$LOG" 2>&1 &
TRAIN_PID=$!

hot=0
while kill -0 "$TRAIN_PID" 2>/dev/null; do
    sleep 15
    tj=$(tail -n 1 "$TS" | grep -o 'tj@[0-9.]*' | cut -d@ -f2 | cut -d. -f1)
    if [ -n "${tj:-}" ] && [ "$tj" -ge "$TJ_MAX" ]; then
        hot=$((hot+1))
        echo "$(date '+%H:%M:%S') tj=${tj} C  (${hot}/3 over ceiling)" | tee -a "$LOG"
        if [ "$hot" -ge 3 ]; then
            echo "THERMAL GUARD: tj ${tj} C for three samples -- stopping training at $(date)" | tee -a "$LOG"
            kill "$TRAIN_PID"; sleep 5; kill -9 "$TRAIN_PID" 2>/dev/null
            break
        fi
    else
        hot=0
    fi
done
wait "$TRAIN_PID" 2>/dev/null; RC=$?
kill "$TS_PID" 2>/dev/null
echo "training exited $RC at $(date)" | tee -a "$LOG"
exit $RC
