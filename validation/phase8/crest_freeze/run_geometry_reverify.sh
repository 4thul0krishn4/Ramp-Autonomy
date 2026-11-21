#!/bin/bash
# Reverification after the ramp-geometry fix, corrected turn procedure (defaults), unscoped fixed detector, ground-truth ascent gate.
#   12 runs each of the 4 loaded cells, 6 each of the 2 unloaded regression cells, then the full 7-cell matrix and both round trips.
# Guard-safe: waits for a quiet host, runs each trial individually, retries a trial after a guard stop (up to 4 times).
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; P8=$WS/validation/phase8; H=$P8/crest_freeze; GUARD="python3 $P8/harness/harness_guard.py"
unset INJECTOR INJECT_MODE PAUSE_DELAY PAUSE_DURATION CLOSED_LOOP_TURN PLATFORM_TURN_M PATH_SPACING
export GT_TRACE=1
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
INST=$WS/install/ramp_autonomy_control/lib/python3.12/site-packages/ramp_autonomy_control/motion_stall_detector_node.py
echo "detector $(sha256sum $INST | cut -c1-8)  params $(sha256sum $WS/install/ramp_autonomy_nav/share/ramp_autonomy_nav/config/nav2_params.yaml | cut -c1-12)  curved world $(sha256sum $WS/install/ramp_autonomy_gazebo/share/ramp_autonomy_gazebo/worlds/ramp_world_curved.sdf | cut -c1-8)"
busy() { python3 - <<'PY'
import time
def r():
    v=[int(x) for x in open('/proc/stat').readline().split()[1:]]; return sum(v), v[3]+v[4]
a=r(); time.sleep(15); b=r(); print(int(100*(1-(b[1]-a[1])/max(1,b[0]-a[0]))))
PY
}
wait_quiet() { local ok=0 waited=0; while [ $ok -lt 2 ]; do b=$(busy); if [ "$b" -lt 12 ]; then ok=$((ok+1)); else ok=0; fi; waited=$((waited+15)); [ $waited -gt 7200 ] && return 1; done; return 0; }
valid() { grep -q "^overall_verdict:" "$1_result.log" 2>/dev/null && python3 -c "import json,sys;sys.exit(1 if json.load(open('$1_health.json'))['starved'] else 0)" 2>/dev/null; }
cell() {   # name world mass cg prefix n
  local name=$1 world=$2 mass=$3 cg=$4 prefix=$5 n=$6 dir=$H/reverify_${1}_logs
  mkdir -p $dir; echo "########## $name x$n"
  for i in $(seq 1 $n); do
    for attempt in 1 2 3 4; do
      valid $dir/${prefix}$i && break
      wait_quiet || { echo "host never quiet"; exit 3; }
      WORLD=$world MASS=$mass CG=$cg GZW=ramp_world_${world%%_*} START=$i LOGDIR=$dir LABEL_PREFIX=$prefix bash $H/run_cell_repeat_resumable.sh $i >> $dir/driver.log 2>&1
      valid $dir/${prefix}$i && break
      echo "  $name run $i attempt $attempt did not complete cleanly - retrying"; $GUARD cleanup >/dev/null 2>&1; sleep 60
    done
    valid $dir/${prefix}$i || { echo "  $name run $i FAILED to obtain a valid result"; }
  done
}
cell curved_loaded_mu045   curved_mu045_TESTONLY 100.0 0.1 curved_loaded_mu045_rv_r 12
cell curved_loaded_normal  curved                100.0 0.1 curved_loaded_normal_rv_r 12
cell sharp_loaded_normal   sharp                 100.0 0.1 sharp_loaded_normal_rv_r 12
cell sharp_loaded_mu045    sharp_lowfriction_TESTONLY 100.0 0.1 sharp_loaded_mu045_rv_r 12
cell curved_unloaded_normal curved               0.0 0.0 curved_unloaded_normal_rv_r 6
cell sharp_unloaded_normal sharp                 0.0 0.0 sharp_unloaded_normal_rv_r 6
echo "########## full matrix"
export LOGDIR=$H/matrix_geometry_logs
for attempt in $(seq 1 6); do
  wait_quiet || exit 3
  bash $P8/path_structural/run_matrix_resumable.sh >> $H/matrix_geometry_full.log 2>&1 && break
  echo "matrix stopped (attempt $attempt)"; $GUARD cleanup >/dev/null 2>&1; sleep 90
done
grep -q "CLEAN-HOST REGRESSION MATRIX COMPLETE" $H/matrix_geometry_full.log || { echo "MATRIX NOT COMPLETED"; exit 2; }
echo "########## round trips"
for cfg in "loaded 100.0 0.1" "unloaded 0.0 0.0"; do
  set -- $cfg; done_ok=0
  for attempt in 1 2 3 4 5 6; do
    wait_quiet || exit 3
    $GUARD cleanup && $GUARD preflight || { sleep 60; continue; }
    L=$P8/construction_site_descent/roundtrip_geometry_$1_launch.log
    setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=$2 payload_cg_offset:=$3 headless:=true > $L 2>&1 < /dev/null &
    disown
    for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
    grep -q "Managed nodes are active" $L || { $GUARD cleanup; sleep 60; continue; }
    sleep 5
    $GUARD verify > $P8/construction_site_descent/roundtrip_geometry_$1_verify.txt 2>&1 || { $GUARD cleanup; sleep 90; continue; }
    $GUARD monitor $P8/construction_site_descent/roundtrip_geometry_$1_health.json 900 & MON=$!
    ( cd $P8/construction_site_descent && python3 arc_turn_followpath_test_ekf.py $PWD/roundtrip_geometry_$1.csv > roundtrip_geometry_$1.log 2>&1 )
    echo "round trip $1 exit=$? (attempt $attempt)"
    kill $MON 2>/dev/null; wait $MON 2>/dev/null
    $GUARD summarize $P8/construction_site_descent/roundtrip_geometry_$1_health.json; rc=$?
    $GUARD cleanup
    [ $rc -eq 3 ] && { echo "round trip $1 STARVED - discarding"; mkdir -p $P8/construction_site_descent/roundtrip_geometry_starved_discarded; mv $P8/construction_site_descent/roundtrip_geometry_$1* $P8/construction_site_descent/roundtrip_geometry_starved_discarded/ 2>/dev/null; sleep 90; continue; }
    done_ok=1; break
  done
  [ $done_ok -eq 1 ] || { echo "ROUND TRIP $1 NOT COMPLETED"; exit 2; }
done
echo "GEOMETRY REVERIFY COMPLETE"
