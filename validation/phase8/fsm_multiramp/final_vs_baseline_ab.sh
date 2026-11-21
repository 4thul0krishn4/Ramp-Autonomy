#!/bin/bash
# Interleaved A/B of the WHOLE change set (final FSM 4db40158 + keepalive-verify switcher 3020997a) vs the BASELINE at the start of the multi-ramp work
# (original FSM 1104e6bf + original switcher 654cf299) on sharp / normal friction / UNLOADED - the one matrix cell that showed a HALT_THRASH verdict (10 HALTs) in the final regression.
# 6 fresh-launch trials per arm, order alternating. Installed files are swapped between trials and restored (sha-checked) at the end.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; D=$WS/validation/phase8/fsm_multiramp; I=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav
FIN_FSM=$WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py; FIN_SW=$WS/src/ramp_autonomy_nav/ramp_autonomy_nav/nav2_param_switcher_node.py
export WORLD=sharp MASS=0.0 CG=0.0 GZW=ramp_world_sharp
for i in $(seq ${AB_FROM:-1} ${AB_TO:-6}); do
  if [ $((i % 2)) = 1 ]; then ORDER="final baseline"; else ORDER="baseline final"; fi
  for arm in $ORDER; do
    if [ $arm = final ]; then cp $FIN_FSM $I/state_machine_node.py; cp $FIN_SW $I/nav2_param_switcher_node.py
    else cp $D/state_machine_node.ORIGINAL.py $I/state_machine_node.py; cp $D/nav2_param_switcher_node.ORIGINAL.py $I/nav2_param_switcher_node.py; fi
    echo "### trial $i arm $arm: fsm $(sha256sum $I/state_machine_node.py | cut -c1-8) switcher $(sha256sum $I/nav2_param_switcher_node.py | cut -c1-8)"
    LOGDIR=$D/run13_final_vs_baseline/$arm LABEL_PREFIX=sharp_unloaded_normal_${arm}_r START=$i bash $WS/validation/phase8/crest_freeze/run_cell_repeat_resumable.sh $i
  done
done
cp $FIN_FSM $I/state_machine_node.py; cp $FIN_SW $I/nav2_param_switcher_node.py
echo "restored: $(sha256sum $I/state_machine_node.py $I/nav2_param_switcher_node.py | cut -c1-8 | tr '\n' ' ')"; echo AB DONE
