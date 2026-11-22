#!/bin/bash
# DIAGNOSTIC variant (same steps, the unmodified test is run under a passive recorder, crest_freeze/crest_diag.py).
# Repeats ONE matrix cell (curved / normal friction / unloaded) N times with the identical instrument and steps as
# run_matrix_regression_cleanhost.sh: verified cleanup, preflight, fresh launch, readiness check, verify (one instance,
# RTF >= 0.95), RTF/load monitor, `validation_test_ekf_groundtruth.py`, 220 s cap. Any guard failure stops the run.
# Characterises the cell's pass/fail rate; changes no code under test.
set -o pipefail
WS=${RAMP_WS:?}
GUARD="python3 $WS/validation/phase8/harness/harness_guard.py"
LOGDIR=${LOGDIR:-$WS/validation/phase8/curved_unloaded_repeat_logs}
N=${1:-5}
mkdir -p "$LOGDIR"
source /opt/ros/jazzy/setup.bash; cd "$WS"; source install/setup.bash
abort() { echo "!! RUN STOPPED BY HARNESS GUARD: $1"; $GUARD cleanup >/dev/null 2>&1; exit "${2:-2}"; }
launch() { setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=${WORLD:-curved} payload_mass:=${MASS:-0.0} payload_cg_offset:=${CG:-0.0} headless:=true > "$1" 2>&1 < /dev/null & disown; }
for i in $(seq 1 $N); do
  label=${LABEL_PREFIX:-curved_unloaded_normal_r}$i
  echo "=== REPEAT $i/$N: $label"
  $GUARD cleanup   || abort "cleanup before $label"
  $GUARD preflight || abort "preflight before $label"
  launch "$LOGDIR/${label}_launch.log"; sleep 25
  status=$(ros2 lifecycle get /controller_server 2>&1); echo "controller_server: $status"
  if [[ "$status" != *"active"* ]]; then
    echo "launch check failed - one verified retry after a daemon restart"
    $GUARD cleanup || abort "cleanup during retry of $label"
    ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 2
    $GUARD preflight || abort "preflight during retry of $label"
    launch "$LOGDIR/${label}_launch_retry.log"; sleep 25
    status=$(ros2 lifecycle get /controller_server 2>&1); echo "controller_server (retry): $status"
    [[ "$status" == *"active"* ]] || abort "controller_server never came up for $label"
  fi
  $GUARD verify | tee "$LOGDIR/${label}_verify.txt"; [[ ${PIPESTATUS[0]} -eq 0 ]] || abort "verify failed for $label"
  $GUARD monitor "$LOGDIR/${label}_health.json" 900 & MON=$!
  if [ -n "$INJECTOR" ]; then python3 "$WS/validation/phase8/stall_detector_control/crest_pause_injector.py" "$LOGDIR/${label}_pause.json" ${PAUSE_DELAY:-0.4} ${PAUSE_DURATION:-1.75} ${INJECT_MODE:-hold} > "$LOGDIR/${label}_injector.log" 2>&1 & INJ=$!; fi
  python3 "$WS/validation/phase8/crest_freeze/crest_diag.py" "$label" ${GZW:-ramp_world_curved} "$LOGDIR/${label}_crest.npz" > "$LOGDIR/${label}_result.log" 2>&1
  echo "exit code: $?"
  kill "$MON" 2>/dev/null; wait "$MON" 2>/dev/null
  if [ -n "$INJECTOR" ]; then kill -TERM "$INJ" 2>/dev/null; wait "$INJ" 2>/dev/null; fi
  $GUARD summarize "$LOGDIR/${label}_health.json"; [[ $? -eq 3 ]] && abort "$label ran while STARVED - invalid" 3
  grep -E "^overall_verdict|^ground_truth_verdict|^halt_count" "$LOGDIR/${label}_result.log"
  $GUARD cleanup || abort "cleanup after $label"
done
echo "REPEAT BATCH COMPLETE"
