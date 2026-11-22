#!/bin/bash
# usage: guarded_run.sh <outdir> <name> <payload_mass> <payload_cg> <cmd...>
# Guarded clean-host trial (same protocol as floor2/run_floor2_verification.sh): quiet-host wait -> verified by-PID cleanup -> preflight -> headless launch of the
# full stack on construction_site -> readiness -> guard verify (RTF gate) -> health monitor -> command -> summarize -> cleanup. Up to 3 attempts.
WS=${RAMP_WS:?}; P8=$WS/validation/phase8; GUARD="python3 $P8/harness/harness_guard.py"
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
OUTDIR=$1; NAME=$2; MASS=$3; CG=$4; shift 4; mkdir -p $OUTDIR
busy() { python3 - <<'PY'
import time
def r():
    v=[int(x) for x in open('/proc/stat').readline().split()[1:]]; return sum(v), v[3]+v[4]
a=r(); time.sleep(15); b=r(); print(int(100*(1-(b[1]-a[1])/max(1,b[0]-a[0]))))
PY
}
wait_quiet() { local ok=0 w=0; while [ $ok -lt 2 ]; do b=$(busy); if [ "$b" -lt 12 ]; then ok=$((ok+1)); else ok=0; fi; w=$((w+15)); [ $w -gt 3600 ] && return 1; done; return 0; }
for attempt in 1 2 3; do
  wait_quiet || { echo "host never quiet"; exit 3; }
  $GUARD cleanup && $GUARD preflight || { sleep 60; continue; }
  L=$OUTDIR/${NAME}_launch.log
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=$MASS payload_cg_offset:=$CG detachable_payload:=${DETACHABLE:-false} spawn_x:=${SPAWN_X:-1.0} spawn_y:=${SPAWN_Y:-0.0} spawn_z:=${SPAWN_Z:-0.23} headless:=true > $L 2>&1 < /dev/null &
  disown
  for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
  grep -q "Managed nodes are active" $L || { echo "$NAME never active (attempt $attempt)"; $GUARD cleanup; sleep 60; continue; }
  sleep 5
  $GUARD verify > $OUTDIR/${NAME}_verify.txt 2>&1 || { echo "$NAME verify failed (attempt $attempt): $(tail -1 $OUTDIR/${NAME}_verify.txt)"; $GUARD cleanup; sleep 90; continue; }
  $GUARD monitor $OUTDIR/${NAME}_health.json 900 & MON=$!
  "$@" > $OUTDIR/${NAME}.log 2>&1; echo "$NAME exit=$? (attempt $attempt)"
  kill $MON 2>/dev/null; wait $MON 2>/dev/null
  $GUARD summarize $OUTDIR/${NAME}_health.json; rc=$?
  $GUARD cleanup
  [ $rc -eq 3 ] && { echo "$NAME STARVED - discarding, retry"; mv $OUTDIR/${NAME}.log $OUTDIR/${NAME}.STARVED.log; sleep 90; continue; }
  exit 0
done
echo "$NAME NOT COMPLETED"; exit 1
