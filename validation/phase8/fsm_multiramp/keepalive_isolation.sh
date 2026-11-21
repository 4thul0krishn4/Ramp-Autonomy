#!/bin/bash
# Controlled, reverted-after experiment: does the loaded-climb HALT cadence follow nav2_param_switcher's keepalive interval?
# Only change between arms: env SWITCHER_KEEPALIVE_N (retry ticks of 0.2 s between keepalive re-pushes) read by an EXPERIMENT copy of the switcher installed for the duration.
# 15 = the shipped 3.0 s; 30 = 6.0 s; 7 = 1.4 s; 100000 = keepalive disabled. Baseline repeated at the end (return-to-baseline control). Loaded 100 kg, MPPI-driven ascent legs.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; D=$WS/validation/phase8/fsm_multiramp
INST=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/nav2_param_switcher_node.py
ORIG=$D/nav2_param_switcher_node.ORIGINAL.py; EXP=$D/nav2_param_switcher_node.EXPERIMENT.py
export HG_LOW_RTF=0.85
cp $EXP $INST; echo "experiment switcher installed: $(sha256sum $INST | cut -c1-8)"
for arm in "n15a 15" "n30 30" "n7 7" "off 100000" "n15b 15"; do
  set -- $arm; export SWITCHER_KEEPALIVE_N=$2
  echo "### arm $1 (SWITCHER_KEEPALIVE_N=$2)"
  $D/guarded_run.sh $D/run8_keepalive_isolation $1 100.0 0.1 python3 $D/multiramp_nav2_test.py $D/run8_keepalive_isolation/$1.json --legs ascent
done
cp $ORIG $INST; echo "reverted: installed $(sha256sum $INST | cut -c1-8) (original $(sha256sum $ORIG | cut -c1-8))"
echo ISOLATION DONE
