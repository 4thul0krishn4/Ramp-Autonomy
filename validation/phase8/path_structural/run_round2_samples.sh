#!/bin/bash
# Round 2 larger samples, corrected default turn procedure (no env switches), guarded by the harness:
#   Arm P  : scoped detector (ea1bcee5) x12   ->  armP2_logs
#   swap to the unscoped fixed detector (48625b76), rebuild, verify
#   Arm P-fixed: x12                          ->  armPfixed2_logs        (the FIXED detector is LEFT INSTALLED at the end)
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; P8=$WS/validation/phase8; H=$P8/path_structural
D=$P8/stall_detector_control/data
SRC=$WS/src/ramp_autonomy_control/ramp_autonomy_control/motion_stall_detector_node.py
INST=$WS/install/ramp_autonomy_control/lib/python3.12/site-packages/ramp_autonomy_control/motion_stall_detector_node.py
sha() { sha256sum "$1" | cut -c1-8; }
unset INJECTOR INJECT_MODE PAUSE_DELAY PAUSE_DURATION CLOSED_LOOP_TURN PLATFORM_TURN_M PATH_SPACING
export GT_TRACE=1
[ "$(sha $INST)" = "ea1bcee5" ] || { echo "ABORT: installed detector is $(sha $INST), expected ea1bcee5"; exit 2; }
echo "########## Arm P (scoped ea1bcee5) x12"
LOGDIR=$H/armP2_logs LABEL_PREFIX=curved_unloaded_normal_armP2_r bash $P8/run_cell_repeat_curved_unloaded.sh 12 || { echo "STOPPED in Arm P"; exit 2; }
echo "########## swapping to the unscoped fixed detector"
cp $D/motion_stall_detector_node.FIXED.py $SRC
source /opt/ros/jazzy/setup.bash; ( cd $WS && colcon build --packages-select ramp_autonomy_control 2>&1 | tail -1 )
[ "$(sha $INST)" = "48625b76" ] || { echo "ABORT: swap failed, installed $(sha $INST)"; exit 2; }
echo "installed detector now $(sha $INST) (unscoped fixed)"
echo "########## Arm P-fixed (unscoped 48625b76) x12"
LOGDIR=$H/armPfixed2_logs LABEL_PREFIX=curved_unloaded_normal_armPfixed2_r bash $P8/run_cell_repeat_curved_unloaded.sh 12 || { echo "STOPPED in Arm P-fixed"; exit 2; }
echo "ROUND2 SAMPLES COMPLETE (unscoped fixed detector left installed: $(sha $INST))"
