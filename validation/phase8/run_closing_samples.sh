#!/bin/bash
# Closing samples on the SCOPED detector (guarded clean host, one sim at a time, unchanged instrument):
#   1) curved / normal / unloaded x8 more (adds to the 6 already run)
#   2) curved / mu 0.45 / unloaded x4 (matrix cell whose HALT verdict went MARGINAL -> FAIL at 13 HALTs)
#   3) curved / mu 0.45 / loaded   x4 (matrix cell whose HALT verdict went MARGINAL -> FAIL at 8 HALTs)
H=${RAMP_WS:?}/validation/phase8
unset INJECTOR INJECT_MODE PAUSE_DELAY PAUSE_DURATION
echo "########## 1/3 curved/normal/unloaded x8"
LOGDIR=$H/curved_unloaded_scoped2_logs LABEL_PREFIX=curved_unloaded_normal_scoped2_r bash $H/run_cell_repeat_curved_unloaded.sh 8 || { echo "STOPPED 1"; exit 2; }
echo "########## 2/3 curved / 0.45 / unloaded x4"
WORLD=curved_mu045_TESTONLY MASS=0.0 CG=0.0 LOGDIR=$H/cell_curved_mu045_unloaded_scoped_logs LABEL_PREFIX=curved_unloaded_mu045_scoped_r bash $H/run_cell_repeat_curved_unloaded.sh 4 || { echo "STOPPED 2"; exit 2; }
echo "########## 3/3 curved / 0.45 / loaded x4"
WORLD=curved_mu045_TESTONLY MASS=100.0 CG=0.1 LOGDIR=$H/cell_curved_mu045_loaded_scoped_logs LABEL_PREFIX=curved_loaded_mu045_scoped_r bash $H/run_cell_repeat_curved_unloaded.sh 4 || { echo "STOPPED 3"; exit 2; }
echo "CLOSING SAMPLES COMPLETE"
