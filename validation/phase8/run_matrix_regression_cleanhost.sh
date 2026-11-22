#!/bin/bash
# Hardened re-run of the run_matrix_ekf_crosstrackfix.sh cells (SAME 7 cells, SAME
# validation_test_ekf_groundtruth.py, SAME per-cell 220 s cap - the instrument under
# test is unchanged) with the harness failures that produced the false "50% stall
# rate" designed out:
#   * cleanup is by PID across every sim/ROS process family and is VERIFIED
#     (the old kill_stale was name-based, unverified, and missed launch parents);
#   * before every cell: preflight (zero stale processes, host actually idle);
#   * after every launch, before any motion: verify (exactly one sim instance, no
#     duplicated nodes, load not starved, simulator real-time factor ~1.0);
#   * during every cell: an RTF/load monitor, and a starved cell aborts the run.
# Any guard failure STOPS the whole run (exit 2/3) instead of recording a starved trial.
# New LOGDIR so the historical baseline logs (matrix_ekf_crosstrackfix_logs) are not overwritten.
set -o pipefail
WS=${RAMP_WS:?}
GUARD="python3 $WS/validation/phase8/harness/harness_guard.py"
LOGDIR=${LOGDIR:-$WS/validation/phase8/matrix_regression_cleanhost_logs}
mkdir -p "$LOGDIR"

source /opt/ros/jazzy/setup.bash
cd "$WS"
source install/setup.bash

abort() {
  echo "=================================================="
  echo "!! RUN STOPPED BY HARNESS GUARD: $1"
  echo "=================================================="
  $GUARD cleanup >/dev/null 2>&1
  exit "${2:-2}"
}

launch() {  # $1 world  $2 payload_mass  $3 payload_cg  $4 logfile
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
      world:="$1" payload_mass:="$2" payload_cg_offset:="$3" headless:=true \
      > "$4" 2>&1 < /dev/null &
  disown
}

run_cell() {
  local label=$1 world=$2 payload_mass=$3 payload_cg=$4 gz_world_name=$5
  echo "=================================================="
  echo "CELL: $label  world=$world  payload_mass=$payload_mass  payload_cg=$payload_cg  gz_world_name=$gz_world_name"
  echo "=================================================="
  $GUARD cleanup   || abort "cleanup could not clear stale processes before $label"
  $GUARD preflight || abort "preflight failed before $label"
  launch "$world" "$payload_mass" "$payload_cg" "$LOGDIR/${label}_launch.log"
  sleep 25
  local status
  status=$(ros2 lifecycle get /controller_server 2>&1)
  echo "controller_server: $status"
  if [[ "$status" != *"active"* ]]; then
    echo "LAUNCH FAILED for $label - one retry after a verified cleanup and daemon restart"
    $GUARD cleanup   || abort "cleanup failed during retry of $label"
    ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
    $GUARD preflight || abort "preflight failed on retry of $label"
    launch "$world" "$payload_mass" "$payload_cg" "$LOGDIR/${label}_launch_retry.log"
    sleep 25
    status=$(ros2 lifecycle get /controller_server 2>&1)
    echo "controller_server (retry): $status"
    [[ "$status" == *"active"* ]] || abort "controller_server never came up for $label"
  fi
  $GUARD verify | tee "$LOGDIR/${label}_verify.txt"
  [[ ${PIPESTATUS[0]} -eq 0 ]] || abort "post-launch verify failed for $label (starved host or duplicate instance)"

  $GUARD monitor "$LOGDIR/${label}_health.json" 900 &
  local MON=$!
  timeout 220 python3 "$WS/validation/phase8/validation_test_ekf_groundtruth.py" "$label" "$gz_world_name" > "$LOGDIR/${label}_result.log" 2>&1
  echo "exit code: $?"
  kill "$MON" 2>/dev/null; wait "$MON" 2>/dev/null
  tail -n 30 "$LOGDIR/${label}_result.log"
  $GUARD summarize "$LOGDIR/${label}_health.json"
  [[ $? -eq 3 ]] && abort "cell $label ran while the host was STARVED - result invalid" 3
  $GUARD cleanup   || abort "cleanup failed after $label"
}

run_cell sharp_unloaded_normal sharp 0.0 0.0 ramp_world_sharp
run_cell sharp_loaded_normal sharp 100.0 0.1 ramp_world_sharp
run_cell sharp_loaded_mu045 sharp_lowfriction_TESTONLY 100.0 0.1 ramp_world_sharp
run_cell curved_unloaded_normal curved 0.0 0.0 ramp_world_curved
run_cell curved_unloaded_mu045 curved_mu045_TESTONLY 0.0 0.0 ramp_world_curved
run_cell curved_loaded_normal curved 100.0 0.1 ramp_world_curved
run_cell curved_loaded_mu045 curved_mu045_TESTONLY 100.0 0.1 ramp_world_curved

echo "=================================================="
echo "CLEAN-HOST REGRESSION MATRIX COMPLETE (all 7 cells passed the harness guard)"
echo "=================================================="
