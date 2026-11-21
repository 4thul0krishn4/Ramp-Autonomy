#!/bin/bash
# E2: raw slip-metric behaviour with the slip HALT DISABLED (FSM_SLIP_HALT=1e9; margin and stall HALTs stay): how long do the onset / decel / turn artefacts last when nothing intervenes?
export HG_LOW_RTF=0.85
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; D=$WS/validation/phase8/halt_loop; INST=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
cp $D/state_machine_node.EXP_SLIPENV.py $INST; echo "experiment FSM installed $(sha256sum $INST | cut -c1-8)"
export FSM_SLIP_HALT=1e9
mkdir -p $D/run2_noslip
$D/guarded_run.sh $D/run2_noslip noslip_loaded 100.0 0.1 python3 $D/start_stop_repro.py $D/run2_noslip/noslip_loaded.json 24
$D/guarded_run.sh $D/run2_noslip noslip_unloaded 0.0 0.0 python3 $D/start_stop_repro.py $D/run2_noslip/noslip_unloaded.json 24
cp $WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py $INST; echo "restored $(sha256sum $INST | cut -c1-8)"
echo CHAIN2 DONE
