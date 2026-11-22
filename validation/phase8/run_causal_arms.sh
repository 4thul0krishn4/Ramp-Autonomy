#!/bin/bash
# Causal test of the crest-HALT hypothesis on the FIXED detector (which has no crest HALT). Three arms, 5 runs each:
#   hold    - a 1.75 s pause via the test hook (no stall latch); its unavoidable HALT side effects are logged
#   ceiling - descent_brake's post-HALT restart profile (40% of ceiling, +5%/0.5 s) with NO pause
#   reset   - the MPPI parameter pushes a HALT causes, unchanged values, NO pause
H=${RAMP_WS:?}/validation/phase8
export INJECTOR=1 LOGDIR=$H/curved_unloaded_pause_logs PAUSE_DELAY=0.4 PAUSE_DURATION=1.75
for arm in "hold pause_b_r" "ceiling ceil_r" "reset reset_r"; do
  set -- $arm
  export INJECT_MODE=$1 LABEL_PREFIX=curved_unloaded_normal_$2
  echo "########## ARM $1"
  bash $H/run_cell_repeat_curved_unloaded.sh 5 || { echo "ARM $1 STOPPED"; exit 2; }
done
echo "CAUSAL ARMS COMPLETE"
