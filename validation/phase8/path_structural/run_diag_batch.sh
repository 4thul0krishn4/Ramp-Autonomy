#!/bin/bash
H=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}/validation/phase8/path_structural
unset INJECTOR INJECT_MODE PAUSE_DELAY PAUSE_DURATION
echo "## baseline spacing 0.4 x3"
LOGDIR=$H/diag_logs LABEL_PREFIX=diag_base04_r bash $H/run_cell_diag.sh 3 || exit 2
echo "## dense 0.05 x2"
PATH_SPACING=0.05 LOGDIR=$H/diag_logs LABEL_PREFIX=diag_dense005_r bash $H/run_cell_diag.sh 2 || exit 2
echo "## reference mu0.45 unloaded x1"
WORLD=curved_mu045_TESTONLY LOGDIR=$H/diag_logs LABEL_PREFIX=diag_mu045_r bash $H/run_cell_diag.sh 1 || exit 2
echo "DIAG BATCH DONE"
