#!/bin/bash
# Retry wrapper: the harness guard (correctly) refuses to run trials on a contended host. This waits for a quiet host (busy CPU < 12% for
# 3 consecutive 20 s samples, i.e. no sim running), then runs the resumable matrix, then the round trips with the same verify/monitor
# guards; on any guard stop it waits and retries (resuming), up to a limit. Nothing is lowered: the guard thresholds are unchanged.
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; P8=$WS/validation/phase8; H=$P8/path_structural
GUARD="python3 $P8/harness/harness_guard.py"
unset INJECTOR INJECT_MODE PAUSE_DELAY PAUSE_DURATION CLOSED_LOOP_TURN PLATFORM_TURN_M PATH_SPACING
export GT_TRACE=1 LOGDIR=$H/matrix_final_logs
INST=$WS/install/ramp_autonomy_control/lib/python3.12/site-packages/ramp_autonomy_control/motion_stall_detector_node.py
echo "detector installed: $(sha256sum $INST | cut -c1-8)   nav2_params: $(sha256sum $WS/install/ramp_autonomy_nav/share/ramp_autonomy_nav/config/nav2_params.yaml | cut -c1-12)"
busy() { python3 - <<'PY'
import time
def r():
    v=[int(x) for x in open('/proc/stat').readline().split()[1:]]; return sum(v), v[3]+v[4]
a=r(); time.sleep(20); b=r(); print(int(100*(1-(b[1]-a[1])/max(1,b[0]-a[0]))))
PY
}
wait_quiet() { local ok=0 waited=0
  while [ $ok -lt 3 ]; do b=$(busy); if [ "$b" -lt 12 ]; then ok=$((ok+1)); else ok=0; fi; waited=$((waited+20)); [ $waited -gt 10800 ] && return 1; done; echo "host quiet (waited ${waited}s)"; return 0; }
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
for attempt in $(seq 1 12); do
  wait_quiet || { echo "host never quiet"; exit 3; }
  echo "########## matrix attempt $attempt"
  bash $H/run_matrix_resumable.sh >> $H/matrix_final_full.log 2>&1 && { echo "matrix complete"; break; }
  echo "matrix stopped by guard on attempt $attempt - retrying after a wait"; $GUARD cleanup >/dev/null 2>&1; sleep 90
done
grep -q "CLEAN-HOST REGRESSION MATRIX COMPLETE" $H/matrix_final_full.log || { echo "MATRIX NOT COMPLETED"; exit 2; }
for cfg in "loaded 100.0 0.1" "unloaded 0.0 0.0"; do
  set -- $cfg; done_ok=0
  for attempt in 1 2 3 4 5 6; do
    wait_quiet || exit 3
    $GUARD cleanup && $GUARD preflight || { sleep 60; continue; }
    L=$P8/construction_site_descent/roundtrip_final_$1_launch.log
    setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=$2 payload_cg_offset:=$3 headless:=true > $L 2>&1 < /dev/null &
    disown
    for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
    grep -q "Managed nodes are active" $L || { echo "round trip $1 never became active (attempt $attempt)"; $GUARD cleanup; sleep 60; continue; }
    sleep 5
    $GUARD verify > $P8/construction_site_descent/roundtrip_final_$1_verify.txt 2>&1 || { echo "round trip $1 verify failed (attempt $attempt)"; $GUARD cleanup; sleep 90; continue; }
    $GUARD monitor $P8/construction_site_descent/roundtrip_final_$1_health.json 900 & MON=$!
    ( cd $P8/construction_site_descent && python3 arc_turn_followpath_test_ekf.py $PWD/roundtrip_final_$1.csv > roundtrip_final_$1.log 2>&1 )
    echo "round trip $1 exit=$? (attempt $attempt)"
    kill $MON 2>/dev/null; wait $MON 2>/dev/null
    $GUARD summarize $P8/construction_site_descent/roundtrip_final_$1_health.json; rc=$?
    $GUARD cleanup
    [ $rc -eq 3 ] && { echo "round trip $1 ran STARVED - discarding, retrying"; mkdir -p $P8/construction_site_descent/roundtrip_final_starved_discarded; mv $P8/construction_site_descent/roundtrip_final_$1* $P8/construction_site_descent/roundtrip_final_starved_discarded/ 2>/dev/null; sleep 90; continue; }
    done_ok=1; break
  done
  [ $done_ok -eq 1 ] || { echo "ROUND TRIP $1 NOT COMPLETED"; exit 2; }
done
echo "FINAL VALIDATION COMPLETE"
