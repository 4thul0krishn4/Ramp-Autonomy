#!/bin/bash
# Interleaved A/B of the ASCENT->TRANSITION trigger on real Nav2/MPPI climbs (both ramps in every run): CURRENT (multi-ramp FSM, sha 067e9d6e: TRANSITION for the whole climb)
# vs GATED (crest-only debounced pre-emption: ramp_mode governs the sustained climb, transition_mode the last ~2.7 m). Fixed switcher installed for both arms.
# Unloaded and loaded (100 kg), 3 fresh-launch trials per cell per arm, order alternating so host drift cannot favour an arm.
WS=${RAMP_WS:?}; D=$WS/validation/phase8/fsm_multiramp
INST=$WS/install/ramp_autonomy_nav/lib/python3.12/site-packages/ramp_autonomy_nav/state_machine_node.py
export HG_LOW_RTF=0.85
for i in 1 2 3; do
  for payload in unloaded loaded; do
    if [ $payload = loaded ]; then MASS=100.0; CG=0.1; else MASS=0.0; CG=0.0; fi
    if [ $((i % 2)) = 1 ]; then ORDER="current gated"; else ORDER="gated current"; fi
    for arm in $ORDER; do
      if [ $arm = current ]; then cp $D/state_machine_node.MULTIRAMP_ONLY.py $INST; else cp $D/state_machine_node.GATED_CANDIDATE.py $INST; fi
      echo "### ${payload}_${arm}_r$i: installed sha $(sha256sum $INST | cut -c1-8)"
      $D/guarded_run.sh $D/run11_ramp_mode_ab ${payload}_${arm}_r$i $MASS $CG python3 $D/multiramp_nav2_test.py $D/run11_ramp_mode_ab/${payload}_${arm}_r$i.json --legs ascent
    done
  done
done
cp $D/state_machine_node.MULTIRAMP_ONLY.py $INST; echo "restored: $(sha256sum $INST | cut -c1-8)"
echo RAMPMODE AB DONE
