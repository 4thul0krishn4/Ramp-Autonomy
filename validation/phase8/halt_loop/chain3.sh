#!/bin/bash
# E3: REAL slip populations: loaded descents on low-friction (mu 0.45) sharp and curved worlds and on normal-friction curved, slip HALT DISABLED (FSM_SLIP_HALT=1e9) so nothing truncates the episodes,
# raw slip recorded at message rate by slip_recorder.py next to the standard validation cell. 3 fresh-launch trials per cell.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; D=$WS/validation/phase8/halt_loop; INST=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
cp $D/state_machine_node.EXP_SLIPENV.py $INST; echo "experiment FSM installed $(sha256sum $INST | cut -c1-8)"
export FSM_SLIP_HALT=1e9 MASS=100.0 CG=0.1
for cell in "sharp_lowfriction_TESTONLY ramp_world_sharp sharp_mu045" "curved_mu045_TESTONLY ramp_world_curved curved_mu045" "curved ramp_world_curved curved_normal"; do
  set -- $cell; export WORLD=$1 GZW=$2
  LOGDIR=$D/run3_realslip LABEL_PREFIX=${3}_loaded_r bash $D/cell_with_recorder.sh 3
done
cp $WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py $INST; echo "restored $(sha256sum $INST | cut -c1-8)"
echo CHAIN3 DONE
