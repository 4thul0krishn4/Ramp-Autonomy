#!/bin/bash
set -o pipefail
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}
SCRATCH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGDIR=$SCRATCH/matrix_logs
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
cd "$WS"
source install/setup.bash

kill_stale() {
  ps aux | grep -E "gz sim|nav2_|ramp_autonomy|robot_state_publisher|parameter_bridge" | grep -v grep | awk '{print $2}' | xargs -r kill -9
  sleep 2
}

run_cell() {
  local label=$1 world=$2 payload_mass=$3 payload_cg=$4
  echo "=================================================="
  echo "CELL: $label  world=$world  payload_mass=$payload_mass  payload_cg=$payload_cg"
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
  timeout 220 python3 "$SCRATCH/validation_test.py" "$label" > "$LOGDIR/${label}_result.log" 2>&1
  echo "exit code: $?"
  tail -n 25 "$LOGDIR/${label}_result.log"
  kill_stale
}

# --- Friction-floor characterization (sharp, unloaded) ---
run_cell floor_mu025 sharp_mu025_TESTONLY 0.0 0.0
run_cell floor_mu030 sharp_mu030_TESTONLY 0.0 0.0

# --- Full matrix: 2 worlds x 2 payloads x {normal friction, mu=0.45} ---
# sharp/unloaded/mu045 already covered by this session's halt_recovery_test.py run - not repeated.
run_cell sharp_unloaded_normal sharp 0.0 0.0
run_cell sharp_loaded_normal sharp 100.0 0.1
run_cell sharp_loaded_mu045 sharp_lowfriction_TESTONLY 100.0 0.1
run_cell curved_unloaded_normal curved 0.0 0.0
run_cell curved_unloaded_mu045 curved_mu045_TESTONLY 0.0 0.0
run_cell curved_loaded_normal curved 100.0 0.1
run_cell curved_loaded_mu045 curved_mu045_TESTONLY 100.0 0.1

echo "=================================================="
echo "MATRIX RUN COMPLETE"
echo "=================================================="
