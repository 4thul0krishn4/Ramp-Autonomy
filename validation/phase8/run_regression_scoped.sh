#!/bin/bash
# Final validation with the STATE-SCOPED stall detector live and the test hook removed:
#   1) curved / normal / unloaded x6 (the cell the scoping is meant to fix), no injector
#   2) the full 7-cell matrix
#   3) both construction-site round trips
# All guarded (verified cleanup, preflight, one instance, RTF >= 0.95); any guard failure stops the run.
WS=${RAMP_WS:?}
P8=$WS/validation/phase8
GUARD="python3 $P8/harness/harness_guard.py"
unset INJECTOR INJECT_MODE PAUSE_DELAY PAUSE_DURATION
echo "########## 1/3 curved/normal/unloaded x6 (scoped detector)"
LOGDIR=$P8/curved_unloaded_scoped_logs LABEL_PREFIX=curved_unloaded_normal_scoped_r bash $P8/run_cell_repeat_curved_unloaded.sh 6 || { echo "REPEATS STOPPED"; exit 2; }
echo "########## 2/3 full 7-cell matrix"
LOGDIR=$P8/matrix_regression_scoped_logs bash $P8/run_matrix_regression_cleanhost.sh > $P8/matrix_regression_scoped_full.log 2>&1 || { echo "MATRIX STOPPED"; exit 2; }
echo "########## 3/3 construction-site round trips"
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
for cfg in "loaded 100.0 0.1" "unloaded 0.0 0.0"; do
  set -- $cfg
  $GUARD cleanup && $GUARD preflight || exit 2
  L=$P8/construction_site_descent/roundtrip_scoped_$1_launch.log
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=$2 payload_cg_offset:=$3 headless:=true > $L 2>&1 < /dev/null &
  disown
  for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
  grep -q "Managed nodes are active" $L || { echo "round trip $1 never became active"; $GUARD cleanup; exit 2; }
  sleep 5
  ( cd $P8/construction_site_descent && python3 arc_turn_followpath_test_ekf.py $PWD/roundtrip_scoped_$1.csv > roundtrip_scoped_$1.log 2>&1 )
  echo "round trip $1 exit=$?"
  $GUARD cleanup
done
echo "SCOPED VALIDATION COMPLETE"
