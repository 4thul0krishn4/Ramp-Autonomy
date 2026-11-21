#!/bin/bash
# Interleaved A/B on the one cell whose HALT-count verdict moved in the regression matrix (sharp world, mu 0.45, loaded 100 kg): NEW vs ORIGINAL state machine, 6 fresh-launch trials each,
# alternating new/old per trial so slow host drift cannot favour an arm. The installed state_machine_node.py is swapped between trials and restored (sha-checked) at the end.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; INST=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
NEW=$WS/src/ramp_autonomy_nav/ramp_autonomy_nav/state_machine_node.py; OLD=$WS/validation/phase8/fsm_multiramp/state_machine_node.ORIGINAL.py
export WORLD=sharp_lowfriction_TESTONLY MASS=100.0 CG=0.1 GZW=ramp_world_sharp
for i in 1 2 3 4 5 6; do
  for arm in new old; do
    if [ $arm = new ]; then cp $NEW $INST; else cp $OLD $INST; fi
    echo "### trial $i arm $arm: installed sha $(sha256sum $INST | cut -c1-8)"
    LOGDIR=$WS/validation/phase8/fsm_multiramp/run5_sharp_ab/$arm LABEL_PREFIX=sharp_loaded_mu045_${arm}_r START=$i bash $WS/validation/phase8/crest_freeze/run_cell_repeat_resumable.sh $i
  done
done
cp $NEW $INST; echo "restored: $(sha256sum $INST $NEW | cut -c1-8 | tr '\n' ' ')"
echo AB DONE
