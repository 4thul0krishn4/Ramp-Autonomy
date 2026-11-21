#!/bin/bash
# E3b: REAL slip positive control: sharp world with ramp friction 0.25 (< tan 16 deg: the ramp cannot be climbed, wheels genuinely slip), unloaded, slip HALT disabled, raw slip recorded. 2 trials.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; D=$WS/validation/phase8/halt_loop; INST=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
cp $D/state_machine_node.EXP_SLIPENV.py $INST; export FSM_SLIP_HALT=1e9 MASS=0.0 CG=0.0 WORLD=sharp_mu025_TESTONLY GZW=ramp_world_sharp
LOGDIR=$D/run3_realslip LABEL_PREFIX=sharp_mu025_unloaded_r bash $D/cell_with_recorder.sh 2
cp $WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py $INST; echo "restored $(sha256sum $INST | cut -c1-8)"; echo CHAIN3B DONE
