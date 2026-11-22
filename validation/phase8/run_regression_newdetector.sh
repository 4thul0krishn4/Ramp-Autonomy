#!/bin/bash
# Regression pass on the FIXED stall detector: the 7-cell matrix, then the loaded and unloaded
# construction-site round trips, each guarded (verified cleanup, preflight, one instance, RTF >= 0.95).
WS=${RAMP_WS:?}
P8=$WS/validation/phase8
GUARD="python3 $P8/harness/harness_guard.py"
export LOGDIR=$P8/matrix_regression_newdetector_logs
bash $P8/run_matrix_regression_cleanhost.sh > $P8/matrix_regression_newdetector_full.log 2>&1 || { echo "MATRIX STOPPED"; exit 2; }
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
for cfg in "loaded 100.0 0.1" "unloaded 0.0 0.0"; do
  set -- $cfg
  $GUARD cleanup && $GUARD preflight || exit 2
  L=$P8/construction_site_descent/roundtrip_newdet_$1_launch.log
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=$2 payload_cg_offset:=$3 headless:=true > $L 2>&1 < /dev/null &
  disown
  for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
  grep -q "Managed nodes are active" $L || { echo "round trip $1 never became active"; $GUARD cleanup; exit 2; }
  sleep 5
  ( cd $P8/construction_site_descent && python3 arc_turn_followpath_test_ekf.py $PWD/roundtrip_newdet_$1.csv > roundtrip_newdet_$1.log 2>&1 )
  echo "round trip $1 exit=$?"
  $GUARD cleanup
done
echo "REGRESSION new-detector COMPLETE"
