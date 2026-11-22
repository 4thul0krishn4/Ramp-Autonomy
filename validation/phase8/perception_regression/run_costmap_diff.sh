#!/bin/bash
WS=${RAMP_WS:?}; P8=$WS/validation/phase8; H=$P8/perception_regression; GUARD="python3 $P8/harness/harness_guard.py"
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
run() { local world=$1 out=$2
  $GUARD cleanup && $GUARD preflight || exit 2
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=$world payload_mass:=0.0 payload_cg_offset:=0.0 headless:=true > $H/launch_cm_$world.log 2>&1 < /dev/null &
  disown; sleep 25
  [[ "$(ros2 lifecycle get /controller_server 2>&1)" == *active* ]] || { $GUARD cleanup; exit 2; }
  $GUARD verify >/dev/null || { $GUARD cleanup; exit 2; }
  python3 $H/obstacle_lidar_check.py $out > /dev/null; $GUARD cleanup; }
run sharp_obstacles_TESTONLY $H/cm_corrected.json
run sharp_obstacles_origgeom_TESTONLY $H/cm_original.json
echo "CM COMPLETE"
