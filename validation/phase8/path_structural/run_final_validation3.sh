#!/bin/bash
# After the platform-drive fix: 5 repeats of curved / 0.45 / loaded (recorded), then the full matrix again. Reuses the quiet-host wrapper for the matrix.
WS=${RAMP_WS:?}; P8=$WS/validation/phase8; H=$P8/path_structural
unset INJECTOR INJECT_MODE PAUSE_DELAY PAUSE_DURATION CLOSED_LOOP_TURN PLATFORM_TURN_M PATH_SPACING
export GT_TRACE=1
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
echo "detector installed: $(sha256sum $WS/install/ramp_autonomy_control/lib/python3.12/site-packages/ramp_autonomy_control/motion_stall_detector_node.py | cut -c1-8)"
echo "########## curved / 0.45 / loaded x5 (distance-based platform drive)"
WORLD=curved_mu045_TESTONLY MASS=100.0 CG=0.1 LOGDIR=$H/curved_loaded_mu045_fix_logs LABEL_PREFIX=curved_loaded_mu045_fix_r bash $P8/run_cell_diag.sh 5 2>/dev/null || WORLD=curved_mu045_TESTONLY MASS=100.0 CG=0.1 LOGDIR=$H/curved_loaded_mu045_fix_logs LABEL_PREFIX=curved_loaded_mu045_fix_r bash $H/run_cell_diag.sh 5 || { echo "STOPPED loaded-0.45 repeats"; exit 2; }
echo "########## full matrix"
export LOGDIR=$H/matrix_final2_logs
for attempt in $(seq 1 6); do
  bash $H/run_matrix_resumable.sh >> $H/matrix_final2_full.log 2>&1 && break
  echo "matrix stopped by guard (attempt $attempt) - retry"; python3 $P8/harness/harness_guard.py cleanup >/dev/null 2>&1; sleep 120
done
grep -q "CLEAN-HOST REGRESSION MATRIX COMPLETE" $H/matrix_final2_full.log || { echo "MATRIX NOT COMPLETED"; exit 2; }
echo "FINAL2 COMPLETE"
