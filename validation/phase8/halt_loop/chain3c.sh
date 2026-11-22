#!/bin/bash
# E3c: REAL slip positive control at ramp friction 0.12 (the phase-6 live test setting: slip ratios up to 2.08 observed there), unloaded, slip HALT disabled, raw slip recorded. 2 trials.
WS=${RAMP_WS:?}; D=$WS/validation/phase8/halt_loop; INST=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
cp $D/state_machine_node.EXP_SLIPENV.py $INST; export FSM_SLIP_HALT=1e9 MASS=0.0 CG=0.0 WORLD=sharp_mu012_TESTONLY GZW=ramp_world_sharp
LOGDIR=$D/run3_realslip LABEL_PREFIX=sharp_mu012_unloaded_r bash $D/cell_with_recorder.sh 2
cp $WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py $INST; echo "restored $(sha256sum $INST | cut -c1-8)"; echo CHAIN3B DONE
