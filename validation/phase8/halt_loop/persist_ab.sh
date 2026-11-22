#!/bin/bash
# Interleaved A/B on one matrix cell: PRE (state machine without the slip persistence gate, sha 4db40158) vs FIX (persistence-gated slip HALT, src). Only the FSM file differs.
# env: WORLD MASS CG GZW PREFIX OUTDIR AB_FROM AB_TO. Installed file swapped per trial and restored to the FIX file at the end.
WS=${RAMP_WS:?}; D=$WS/validation/phase8/halt_loop; I=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
FIX=$WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py; PRE=$D/state_machine_node.PRE_PERSIST.py
export WORLD MASS CG GZW
for i in $(seq ${AB_FROM:-1} ${AB_TO:-6}); do
  if [ $((i % 2)) = 1 ]; then ORDER="fix pre"; else ORDER="pre fix"; fi
  for arm in $ORDER; do
    if [ $arm = fix ]; then cp $FIX $I; else cp $PRE $I; fi
    echo "### $PREFIX trial $i arm $arm: fsm $(sha256sum $I | cut -c1-8)"
    LOGDIR=$OUTDIR/$arm LABEL_PREFIX=${PREFIX}_${arm}_r START=$i bash $D/cell_with_recorder.sh $i
  done
done
cp $FIX $I; echo "restored: $(sha256sum $I | cut -c1-8)"; echo AB DONE
