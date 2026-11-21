#!/bin/bash
# One fully guarded session: verified cleanup -> preflight -> fresh launch -> readiness wait -> session -> cleanup.
# Usage: run_session.sh <label> <normal|freeze> <unloaded|loaded> <id> [clearance f,b,s] [events]
# Any guard failure stops the session (exit 2) instead of producing a starved/contaminated recording.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}
HERE=$WS/validation/phase8/stall_detector_control
GUARD="python3 $WS/validation/phase8/harness/harness_guard.py"
LABEL=$1; MODE=$2; PAYLOAD=$3; ID=$4; CLEAR=${5:-0.06,0.08,0.04}; EVENTS=${6:-straight_raw,arc,taper}
MASS=0.0; CG=0.0; [ "$PAYLOAD" = "loaded" ] && { MASS=100.0; CG=0.1; }
TAG=${LABEL}_${MODE}_${PAYLOAD}_${ID}
LOG=$HERE/data/launch_${TAG}.log
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash

$GUARD cleanup   || exit 2
$GUARD preflight || exit 2
setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
    world:=construction_site payload_mass:=$MASS payload_cg_offset:=$CG headless:=true > "$LOG" 2>&1 < /dev/null &
disown
for i in $(seq 1 60); do
  grep -q "Managed nodes are active" "$LOG" 2>/dev/null && break
  grep -q "Failed to bring up" "$LOG" 2>/dev/null && { echo "!! launch failed for $TAG"; $GUARD cleanup; exit 2; }
  sleep 3
done
grep -q "Managed nodes are active" "$LOG" || { echo "!! $TAG never became active"; $GUARD cleanup; exit 2; }
sleep 5
cd $HERE
python3 freeze_session.py $MODE $PAYLOAD $ID $HERE/data/$TAG $CLEAR 0.30 20 $EVENTS > data/session_${TAG}.log 2>&1
RC=$?
echo "session $TAG exit=$RC"
$GUARD cleanup
exit $RC
