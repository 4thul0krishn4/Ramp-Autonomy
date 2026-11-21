#!/bin/bash
# Verification of the Floor-2 world (headless, guarded, quiet-host waits, one sim at a time):
#   1) Floor-1-only round trip, LOADED    (existing driver arc_turn_followpath_test_ekf.py, unchanged)
#   2) Floor-1-only round trip, UNLOADED
#   3) ramp-2 traversal to Floor 2, empty payload
# Each: verified cleanup -> preflight -> launch -> readiness -> verify -> monitor -> test -> cleanup. Retry on a guard stop (up to 3 attempts).
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; P8=$WS/validation/phase8; H=$P8/floor2; CS=$P8/construction_site_descent; GUARD="python3 $P8/harness/harness_guard.py"
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
echo "detector $(sha256sum $WS/install/ramp_autonomy_control/lib/python3.12/site-packages/ramp_autonomy_control/motion_stall_detector_node.py | cut -c1-8)  world $(sha256sum $WS/install/ramp_autonomy_gazebo/share/ramp_autonomy_gazebo/worlds/construction_site.sdf | cut -c1-8)"
busy() { python3 - <<'PY'
import time
def r():
    v=[int(x) for x in open('/proc/stat').readline().split()[1:]]; return sum(v), v[3]+v[4]
a=r(); time.sleep(15); b=r(); print(int(100*(1-(b[1]-a[1])/max(1,b[0]-a[0]))))
PY
}
wait_quiet() { local ok=0 w=0; while [ $ok -lt 2 ]; do b=$(busy); if [ "$b" -lt 12 ]; then ok=$((ok+1)); else ok=0; fi; w=$((w+15)); [ $w -gt 7200 ] && return 1; done; return 0; }
run() {   # name payload_mass payload_cg cmd...
  local name=$1 mass=$2 cg=$3; shift 3
  for attempt in 1 2 3; do
    wait_quiet || { echo "host never quiet"; exit 3; }
    $GUARD cleanup && $GUARD preflight || { sleep 60; continue; }
    L=$H/${name}_launch.log
    setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=$mass payload_cg_offset:=$cg headless:=true > $L 2>&1 < /dev/null &
    disown
    for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
    grep -q "Managed nodes are active" $L || { echo "$name never active (attempt $attempt)"; $GUARD cleanup; sleep 60; continue; }
    sleep 5
    $GUARD verify > $H/${name}_verify.txt 2>&1 || { echo "$name verify failed (attempt $attempt)"; $GUARD cleanup; sleep 90; continue; }
    $GUARD monitor $H/${name}_health.json 900 & MON=$!
    "$@" > $H/${name}.log 2>&1; echo "$name exit=$? (attempt $attempt)"
    kill $MON 2>/dev/null; wait $MON 2>/dev/null
    $GUARD summarize $H/${name}_health.json; rc=$?
    $GUARD cleanup
    [ $rc -eq 3 ] && { echo "$name STARVED - discarding, retry"; mv $H/${name}.log $H/${name}.STARVED.log; sleep 90; continue; }
    return 0
  done
  echo "$name NOT COMPLETED"; return 1
}
cd $CS
run roundtrip_floor2world_loaded   100.0 0.1 python3 arc_turn_followpath_test_ekf.py $H/roundtrip_floor2world_loaded.csv
run roundtrip_floor2world_unloaded 0.0   0.0 python3 arc_turn_followpath_test_ekf.py $H/roundtrip_floor2world_unloaded.csv
cd $H
run ramp2_traversal                0.0   0.0 python3 ramp2_traversal_test.py $H/ramp2_traversal.csv
echo "FLOOR2 VERIFICATION COMPLETE"
