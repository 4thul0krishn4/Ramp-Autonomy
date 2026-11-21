#!/bin/bash
# Final validation under the CORRECTED default turn procedure and the detector chosen by the round-2 decision (whatever is installed;
# its sha is logged first). Guarded throughout; any guard failure stops the run.
#   1) positive control (6 normal + 6 freeze sessions, construction site)      -> stall_detector_control/data/final_*.npz (only if RUN_PC=1)
#   2) full 7-cell matrix                                                       -> matrix_final_logs/
#   3) both construction-site round trips                                       -> construction_site_descent/roundtrip_final_*
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; P8=$WS/validation/phase8; H=$P8/path_structural
GUARD="python3 $P8/harness/harness_guard.py"
INST=$WS/install/ramp_autonomy_control/lib/python3.12/site-packages/ramp_autonomy_control/motion_stall_detector_node.py
echo "detector installed: $(sha256sum $INST | cut -c1-8)   nav2_params: $(sha256sum $WS/install/ramp_autonomy_nav/share/ramp_autonomy_nav/config/nav2_params.yaml | cut -c1-12)"
unset INJECTOR INJECT_MODE PAUSE_DELAY PAUSE_DURATION CLOSED_LOOP_TURN PLATFORM_TURN_M PATH_SPACING
export GT_TRACE=1
if [ "$RUN_PC" = "1" ]; then
  echo "########## 1/3 positive control (12 sessions)"
  S=$P8/stall_detector_control
  for spec in "normal loaded 51" "freeze loaded 61" "normal unloaded 52" "freeze unloaded 62" \
              "normal loaded 53" "freeze loaded 63" "normal unloaded 54" "freeze unloaded 64" \
              "normal loaded 55" "freeze loaded 65" "normal unloaded 56" "freeze unloaded 66"; do
    set -- $spec
    bash $S/run_session.sh final $1 $2 $3 || { echo "POSITIVE CONTROL STOPPED at: $spec"; exit 2; }
  done
  echo "positive control complete"
fi
echo "########## 2/3 full 7-cell matrix (corrected turn procedure)"
LOGDIR=$H/matrix_final_logs bash $P8/run_matrix_regression_cleanhost.sh > $H/matrix_final_full.log 2>&1 || { echo "MATRIX STOPPED"; exit 2; }
echo "matrix complete"
echo "########## 3/3 construction-site round trips"
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
for cfg in "loaded 100.0 0.1" "unloaded 0.0 0.0"; do
  set -- $cfg
  $GUARD cleanup && $GUARD preflight || exit 2
  L=$P8/construction_site_descent/roundtrip_final_$1_launch.log
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=$2 payload_cg_offset:=$3 headless:=true > $L 2>&1 < /dev/null &
  disown
  for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
  grep -q "Managed nodes are active" $L || { echo "round trip $1 never became active"; $GUARD cleanup; exit 2; }
  sleep 5
  ( cd $P8/construction_site_descent && python3 arc_turn_followpath_test_ekf.py $PWD/roundtrip_final_$1.csv > roundtrip_final_$1.log 2>&1 )
  echo "round trip $1 exit=$?"
  $GUARD cleanup
done
echo "FINAL VALIDATION COMPLETE"
