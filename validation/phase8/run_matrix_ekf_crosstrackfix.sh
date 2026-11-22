#!/bin/bash
set -o pipefail
WS=${RAMP_WS:?}
SCRATCH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR=$WS/validation/phase8/matrix_ekf_crosstrackfix_logs
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
cd "$WS"
source install/setup.bash

kill_stale() {
  ps aux | grep -E "gz sim|nav2_|ramp_autonomy|robot_state_publisher|parameter_bridge|ekf_node" | grep -v grep | awk '{print $2}' | xargs -r kill -9
  sleep 2
}

run_cell() {
  local label=$1 world=$2 payload_mass=$3 payload_cg=$4 gz_world_name=$5
  echo "=================================================="
  echo "CELL: $label  world=$world  payload_mass=$payload_mass  payload_cg=$payload_cg  gz_world_name=$gz_world_name"
  echo "=================================================="
  kill_stale
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
      world:="$world" payload_mass:="$payload_mass" payload_cg_offset:="$payload_cg" headless:=true \
      > "$LOGDIR/${label}_launch.log" 2>&1 < /dev/null &
  disown
  sleep 25
  local status
  status=$(ros2 lifecycle get /controller_server 2>&1)
  echo "controller_server: $status"
  if [[ "$status" != *"active"* ]]; then
    echo "LAUNCH FAILED for $label - retrying once after daemon restart"
    kill_stale
    ros2 daemon stop >/dev/null 2>&1
    ros2 daemon start >/dev/null 2>&1
    sleep 2
    setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
        world:="$world" payload_mass:="$payload_mass" payload_cg_offset:="$payload_cg" headless:=true \
        > "$LOGDIR/${label}_launch_retry.log" 2>&1 < /dev/null &
    disown
    sleep 25
    status=$(ros2 lifecycle get /controller_server 2>&1)
    echo "controller_server (retry): $status"
    if [[ "$status" != *"active"* ]]; then
      echo "CELL $label ABORTED - controller_server never came up"
      return 1
    fi
  fi
  timeout 220 python3 "${RAMP_WS:?}/validation/phase8/validation_test_ekf_groundtruth.py" "$label" "$gz_world_name" > "$LOGDIR/${label}_result.log" 2>&1
  echo "exit code: $?"
  tail -n 30 "$LOGDIR/${label}_result.log"
  kill_stale
}

# Same 7 cells as validation/phase8/run_matrix.sh's "Full matrix" section,
# for a direct before/after comparison. gz_world_name is the actual
# /world/<name>/... topic name gz-sim uses at runtime - this is the SDF's
# OWN internal <world name="..."> element, NOT the launch 'world:=' arg
# string: confirmed by inspection that every friction TESTONLY variant
# reuses its base world's internal name (e.g. ramp_world_sharp_lowfriction
# _TESTONLY.sdf still declares <world name="ramp_world_sharp">), so it's
# always exactly "ramp_world_sharp" or "ramp_world_curved" here regardless
# of which friction variant is actually loaded.
run_cell sharp_unloaded_normal sharp 0.0 0.0 ramp_world_sharp
run_cell sharp_loaded_normal sharp 100.0 0.1 ramp_world_sharp
run_cell sharp_loaded_mu045 sharp_lowfriction_TESTONLY 100.0 0.1 ramp_world_sharp
run_cell curved_unloaded_normal curved 0.0 0.0 ramp_world_curved
run_cell curved_unloaded_mu045 curved_mu045_TESTONLY 0.0 0.0 ramp_world_curved
run_cell curved_loaded_normal curved 100.0 0.1 ramp_world_curved
run_cell curved_loaded_mu045 curved_mu045_TESTONLY 100.0 0.1 ramp_world_curved

echo "=================================================="
echo "EKF + GROUND-TRUTH MATRIX RUN COMPLETE"
echo "=================================================="
