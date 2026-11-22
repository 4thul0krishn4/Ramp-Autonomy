#!/bin/bash
WS=${RAMP_WS:?}; P8=$WS/validation/phase8; GUARD="python3 $P8/harness/harness_guard.py"
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
OUTDIR=$1; NAME=$2; mkdir -p $OUTDIR
for attempt in 1 2 3; do
  $GUARD cleanup && $GUARD preflight || { sleep 30; continue; }
  L=$OUTDIR/${NAME}_launch.log
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=100.0 payload_cg_offset:=0.1 \
      cargo_mode:=loose contact_diag:=true spawn_x:=47.9702 spawn_y:=0.0 spawn_z:=3.48 headless:=true > $L 2>&1 < /dev/null &
  disown
  for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
  grep -q "Managed nodes are active" $L || { echo "never active (attempt $attempt)"; $GUARD cleanup; sleep 30; continue; }
  sleep 5
  for i in $(seq 1 10); do $GUARD verify > $OUTDIR/${NAME}_verify.txt 2>&1 && break; sleep 5; done
  $GUARD verify >> $OUTDIR/${NAME}_verify.txt 2>&1 || { echo "verify failed (attempt $attempt): $(tail -1 $OUTDIR/${NAME}_verify.txt)"; $GUARD cleanup; sleep 30; continue; }
  $GUARD monitor $OUTDIR/${NAME}_health.json 300 & MON=$!
  cd ${RAMP_WS:?}/validation/phase8/mesh_swap/cargo_demo
  python3 contact_diag_run.py $OUTDIR/${NAME}.csv $OUTDIR/${NAME}_payload.csv $OUTDIR/${NAME}_contact.csv --from-top --no-rtf-gate > $OUTDIR/${NAME}.log 2>&1
  RC=$?
  echo "exit=$RC (attempt $attempt)"
  kill $MON 2>/dev/null; wait $MON 2>/dev/null
  $GUARD summarize $OUTDIR/${NAME}_health.json
  $GUARD cleanup
  if [ $RC -ne 0 ]; then echo "python script failed, retrying"; sleep 30; continue; fi
  echo "DONE"
  exit 0
done
echo "NOT COMPLETED"
