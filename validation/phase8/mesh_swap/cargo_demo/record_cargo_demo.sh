#!/bin/bash
# Records ONE cargo_mode demo run (secured or loose) with the live GUI. Spawns directly at the top of the elevated platform (same point the headless
# secured/loose comparison used - see headless_test/{secured,loose}_fromtop* results) and runs ONLY the demo leg: arc-turn on the flat elevated deck ->
# taper onto ramp centreline -> DESCEND ramp 1, loaded (100 kg). Both modes get the identical spawn point/path/conditions, isolating the descent leg
# itself (the "most dynamically interesting leg" per the task) rather than risking the recording ending during an earlier ascent HALT-storm before ever
# reaching the ramp (loose cargo can already destabilize the robot during ascent from the gate - see headless_test/loose_headless* - a separate, already
# documented finding, not what this recording is for).
# cargo_mode:=loose is an opt-in DEMO capability (see robot.urdf.xacro / docs/construction_site_design.md "Unsecured cargo demo mode") - isolated from every
# validated result: this script is the ONLY place in the project that ever passes cargo_mode:=loose.
# Usage: record_cargo_demo.sh <secured|loose> <out_dir>
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; MODE=$1; OUT=$2; D=$OUT/data
SPAWN_X=47.9702; SPAWN_Y=0.0; SPAWN_Z=3.48
V=$WS/visualization; GUARD="python3 $WS/validation/phase8/harness/harness_guard.py"
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
powerprofilesctl set performance 2>/dev/null; trap 'powerprofilesctl set balanced 2>/dev/null' EXIT
export DISPLAY=${DISPLAY:-:0}
rm -rf $OUT; mkdir -p $D
$GUARD cleanup && $GUARD preflight || exit 2
L=$D/launch.log
setsid nohup env -u GTK_PATH -u GTK_EXE_PREFIX ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
    world:=construction_site payload_mass:=100.0 payload_cg_offset:=0.1 cargo_mode:=$MODE headless:=false \
    spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_z:=$SPAWN_Z > $L 2>&1 < /dev/null &
disown
for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
grep -q "Managed nodes are active" $L || { echo "stack never became active - see $L"; $GUARD cleanup; exit 2; }
sleep 5
python3 $V/x11_window.py resize 1280 720 200 100
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "ramp_robot"' > /dev/null 2>&1
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 --req 'x: -5.0 y: 0.0 z: 3.5' > /dev/null 2>&1
# NOTE: the live GUI client alone (before any recording/encoding) already measures RTF ~0.66-0.69 on this host - the strict RTF>=0.95 headless-validation
# gate ($GUARD verify's hard MIN_RTF, harness_guard.py) is essentially impossible to pass with the GUI open here, but a low RTF only means wall-clock lags
# sim-clock (gz-sim's physics step is fixed-size and sim-clock-driven) - it does not affect the physics/contact results this demo is showing. So: log
# verify's output for the record, don't abort on it. Still guards against a genuinely broken launch (0 or >1 gz-sim servers, duplicate nodes) via preflight
# above + the "Managed nodes are active" check; cargo_descent_demo.py is invoked below with --no-rtf-gate for the same reason.
$GUARD verify | tee $D/verify.txt
XID=$(python3 $V/x11_window.py id)
$GUARD monitor $D/health.json 900 & MON=$!
sleep 2
VSTART=$(date +%s.%N)
nice -n 10 timeout --signal=INT 900 gst-launch-1.0 -e ximagesrc xid=$XID use-damage=false show-pointer=false ! video/x-raw,framerate=20/1 ! videoconvert \
    ! x264enc speed-preset=ultrafast bitrate=3000 key-int-max=40 ! h264parse ! mp4mux ! filesink location=$OUT/cargo_${MODE}_raw.mp4 > $D/gst_record.log 2>&1 & GST=$!
sleep 1
echo "RECORDING cargo demo (cargo_mode=$MODE): video start epoch $VSTART"
cd $WS/validation/phase8/mesh_swap/cargo_demo
python3 cargo_descent_demo.py $D/demo.csv $D/payload_gt.csv --from-top --no-rtf-gate 2>&1 | tee $D/demo.log | grep -E "leg [0-9]|CARGO DEMO RESULT|ascent_verified|leg_check|descent leg ok|HALT events|margin|slip|pitch|lateral|HARNESS|NO_RTF_GATE"
sleep 4
kill -INT $GST; sleep 8; kill $MON 2>/dev/null; wait $MON 2>/dev/null
$GUARD summarize $D/health.json | tee -a $D/verify.txt
echo $VSTART > $D/video_start_epoch.txt
$GUARD cleanup
echo "DONE: $OUT"
