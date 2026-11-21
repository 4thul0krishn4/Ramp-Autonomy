#!/bin/bash
# A/B on the ORIGINAL (unfixed) geometry with today's stack: same scripts, control worlds.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; P8=$WS/validation/phase8; H=$P8/perception_regression; GUARD="python3 $P8/harness/harness_guard.py"
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
run() { local world=$1; shift
  $GUARD cleanup && $GUARD preflight || exit 2
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=$world payload_mass:=0.0 payload_cg_offset:=0.0 headless:=true > $H/launch_ab_$world.log 2>&1 < /dev/null &
  disown; sleep 25
  [[ "$(ros2 lifecycle get /controller_server 2>&1)" == *active* ]] || { echo "not active $world"; $GUARD cleanup; exit 2; }
  $GUARD verify | tee $H/verify_ab_$world.txt; [ ${PIPESTATUS[0]} -eq 0 ] || { $GUARD cleanup; exit 2; }
  "$@"; echo "check exit=$?"; $GUARD cleanup; }
run sharp_origgeom_TESTONLY python3 $H/breakpoint_accuracy.py sharp $H/ab_breakpoint_sharp_origgeom.json
run sharp_obstacles_origgeom_TESTONLY python3 $H/obstacle_lidar_check.py $H/ab_obstacle_lidar_origgeom.json
echo "AB COMPLETE"
