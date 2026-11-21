#!/bin/bash
# Resume of ramp_mode_ab.sh after a pause: the 6 cells not yet run (same protocol, same alternation rule: odd trial = current first, even = gated first).
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; D=$WS/validation/phase8/fsm_multiramp
INST=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
export HG_LOW_RTF=0.85
for cell in "loaded gated 2" "loaded current 2" "unloaded current 3" "unloaded gated 3" "loaded current 3" "loaded gated 3"; do
  set -- $cell; payload=$1; arm=$2; i=$3
  if [ $payload = loaded ]; then MASS=100.0; CG=0.1; else MASS=0.0; CG=0.0; fi
  if [ $arm = current ]; then cp $D/state_machine_node.MULTIRAMP_ONLY.py $INST; else cp $D/state_machine_node.GATED_CANDIDATE.py $INST; fi
  echo "### ${payload}_${arm}_r$i: installed sha $(sha256sum $INST | cut -c1-8)"
  $D/guarded_run.sh $D/run11_ramp_mode_ab ${payload}_${arm}_r$i $MASS $CG python3 $D/multiramp_nav2_test.py $D/run11_ramp_mode_ab/${payload}_${arm}_r$i.json --legs ascent
done
cp $D/state_machine_node.MULTIRAMP_ONLY.py $INST; echo "restored: $(sha256sum $INST | cut -c1-8)"
echo RAMPMODE AB DONE
