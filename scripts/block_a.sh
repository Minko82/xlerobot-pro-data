#!/usr/bin/env bash
# Block A: back-to-back policy trials at one marked bottle position, everything logged.
#
#   ssh -t xle@192.168.1.14 'cd ~/xlerobot-pro-data && bash scripts/block_a.sh 17'      # start at trial 17
#   ...                                          bash scripts/block_a.sh 17 50   # trials 17..50
#
# Per trial: servo temperatures before and after, start-pose replay, the policy run with
# trajectory / plans / frames logged, the runner's timing summary, then a y/n prompt.
# Rows go to trials/blockA/results.csv (scripts/log_block_result.py). tegrastats for the
# whole block: start it once before the block with
#   nohup tegrastats --interval 5000 > trials/blockA/tegrastats.log 2>&1 &
set -u
N=${1:-1}; END=${2:-50}
LOCATION=${LOCATION:-bottom-right}
CK=${CK:-outputs/act_glassbottle_pick_v8/checkpoints/010000/pretrained_model}
P=~/.venvs/xlerobot-pro/bin/python
D=trials/blockA; mkdir -p $D
RUNNER=~/xlerobot-pro/examples/policies/act_policy_control.py
while [ "$N" -le "$END" ]; do
  TAG=$(printf "t%02d" "$N")
  read -r -p "Trial $N ($TAG): bottle on the mark, hands clear. ENTER to run, q to quit: " k
  [ "$k" = "q" ] && break
  $P scripts/servo_temps.py > $D/${TAG}_temps_start.txt 2>/dev/null; echo "  start temps: $(cat $D/${TAG}_temps_start.txt)"
  $P scripts/goto_start_pose.py --port /dev/xle_head --arm left --seconds 5 2>&1 | grep -v -i warn | tail -1
  timeout 120 $P $RUNNER run --checkpoint "$CK" --port /dev/xle_head --arm left --duration 25 --n-action-steps 100 \
      --freeze-frame --overlay calibration/hand_mask_grey88.png \
      --log-trajectory $D/${TAG}.csv --log-frames $D/${TAG}_frames --log-plans $D/${TAG}_plans.csv \
      --task "pick up the glass bottle from the shelf" 2>&1 \
      | grep -v -i warn | grep -i "inference\|chunk inf\|loop \|realised\|peak joint\|stopped\|Traceback\|Error" | tee $D/${TAG}_summary.txt
  $P scripts/servo_temps.py > $D/${TAG}_temps_end.txt 2>/dev/null; echo "  end temps:   $(cat $D/${TAG}_temps_end.txt)"
  while true; do
    read -r -p "  Outcome: y = grasp, n = fail (then stage PERC/APPR/GRASP/LATCH), optional note, e.g. 'n APPR closed short': " line
    set -- $line; ans=${1:-}; shift || true
    if [ "$ans" = "y" ]; then out=grasp; stage=-; note="$*"; break
    elif [ "$ans" = "n" ]; then out=fail; stage=${1:-APPR}; shift || true; note="$*"; break
    fi
  done
  $P scripts/log_block_result.py "$TAG" "$out" "$stage" "$LOCATION" "$note"
  N=$((N+1))
done
echo "stopped before trial $N. Results: $D/results.csv"
