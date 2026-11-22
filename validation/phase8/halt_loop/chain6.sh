#!/bin/bash
export HG_LOW_RTF=0.85
WS=${RAMP_WS:?}; D=$WS/validation/phase8/halt_loop; I=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
mkdir -p $D/run7_blocked
for arm in pre fix; do
  if [ $arm = pre ]; then cp $D/state_machine_node.PRE_PERSIST.py $I; else cp $WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py $I; fi
  echo "### $arm $(sha256sum $I | cut -c1-8)"
  $D/guarded_run.sh $D/run7_blocked blocked_$arm 0.0 0.0 python3 $D/blocked_wheels_control.py $D/run7_blocked/blocked_$arm.json
done
cp $WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py $I; echo "restored $(sha256sum $I | cut -c1-8)"; echo CHAIN6 DONE
