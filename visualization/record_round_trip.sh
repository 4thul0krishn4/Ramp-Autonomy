#!/bin/bash
# Run ONE full construction-site round trip with the complete current stack, the Gazebo GUI live on screen, and RECORD it:
#   - <name>.mp4            H.264 screen capture of the Gazebo GUI window (1280x720, 20 fps, ~3 Mbit/s), the real simulation running
#   - <name>_data/          round-trip CSV + log, ground-truth pose CSV (20 Hz), ROS bag (odometry, IMU, joints, slip, margin, stall/verified-progress, FSM state, perception, both LiDARs, commands),
#                           launch log, guard verify/health, events.json (leg timestamps as offsets into the video)
# Usage: visualization/record_round_trip.sh <loaded|unloaded>          Purely visualization/recording tooling - the autonomy code is not touched.
WS=${RAMP_WS:?}; PAYLOAD=${1:-loaded}
MASS=100.0; CG=0.1; [ "$PAYLOAD" = "unloaded" ] && { MASS=0.0; CG=0.0; }
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
# Recording adds load (encoder + bag + GUI): run the CPU in the performance profile for the duration, restore afterwards.
powerprofilesctl set performance 2>/dev/null; trap 'powerprofilesctl set balanced 2>/dev/null' EXIT
export DISPLAY=${DISPLAY:-:0}
GUARD="python3 $WS/validation/phase8/harness/harness_guard.py"; V=$WS/visualization
OUT=$WS/recordings/round_trip_${PAYLOAD}; D=$OUT/data; rm -rf $OUT; mkdir -p $D
$GUARD cleanup && $GUARD preflight || exit 2
L=$D/launch.log
setsid nohup env -u GTK_PATH -u GTK_EXE_PREFIX ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
    world:=construction_site payload_mass:=$MASS payload_cg_offset:=$CG headless:=false > $L 2>&1 < /dev/null &
disown
for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
grep -q "Managed nodes are active" $L || { echo "stack never became active - see $L"; $GUARD cleanup; exit 2; }
sleep 5
python3 $V/x11_window.py resize 1280 720 200 100
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "ramp_robot"' > /dev/null 2>&1
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 --req 'x: -5.0 y: 0.0 z: 3.5' > /dev/null 2>&1
for i in $(seq 1 24); do $GUARD verify > /dev/null 2>&1 && break; sleep 5; done
$GUARD verify | tee $D/verify.txt || { $GUARD cleanup; exit 2; }
XID=$(python3 $V/x11_window.py id)
$GUARD monitor $D/health.json 1500 & MON=$!
python3 $V/gt_pose_recorder.py $D/ground_truth_pose.csv construction_site 2>/dev/null & GT=$!
# (gz world name of construction_site.sdf is "construction_site")
nice -n 10 timeout --signal=INT 3000 ros2 bag record -o $D/rosbag --compression-mode file --compression-format zstd --topics /odometry/filtered /odom /imu /joint_states /wheel_slip /stability_margin /motion_stalled /verified_progress_m /raw_wheel_distance_m \
    /ramp_state /ramp_perception /scan /scan_360 /cmd_vel_nav /model/ramp_robot/cmd_vel /clock /tf > $D/rosbag_record.log 2>&1 & BAG=$!
sleep 2
VSTART=$(date +%s.%N)
nice -n 10 timeout --signal=INT 3000 gst-launch-1.0 -e ximagesrc xid=$XID use-damage=false show-pointer=false ! video/x-raw,framerate=20/1 ! videoconvert ! video/x-raw,format=I420 \
    ! x264enc speed-preset=ultrafast bitrate=3000 key-int-max=40 ! h264parse ! mp4mux ! filesink location=$OUT/round_trip_${PAYLOAD}_raw.mp4 > $D/gst_record.log 2>&1 & GST=$!
sleep 1
echo "RECORDING round trip ($PAYLOAD): video start epoch $VSTART"
cd $WS/validation/phase8/construction_site_descent
python3 arc_turn_followpath_test_ekf.py $D/roundtrip.csv 2>&1 | tee $D/roundtrip.log | grep -E "leg [0-9]|goal-verification|TRAVERSAL RESULT|aborted|HALT events|final gz|HARNESS"
sleep 6
kill -INT $GST; kill -INT $BAG; kill -INT $GT; sleep 10; kill $MON 2>/dev/null; wait $MON 2>/dev/null
$GUARD summarize $D/health.json | tee -a $D/verify.txt
python3 - "$VSTART" "$D" "$PAYLOAD" <<'PY'
import json, re, sys
vstart, D, payload = float(sys.argv[1]), sys.argv[2], sys.argv[3]
ev = []
for l in open(f'{D}/roundtrip.log'):
    m = re.match(r'\[INFO\] \[(\d+\.\d+)\].*?\]: (leg \d[^:]*: .*|leg \d.*done.*|leg goal-verification.*)', l)
    if m: ev.append(dict(video_time_s=round(float(m.group(1)) - vstart, 1), text=m.group(2)[:200]))
json.dump(dict(payload=payload, video_start_epoch=vstart, events=ev), open(f'{D}/events.json', 'w'), indent=1)
print(f'{len(ev)} events written')
PY
$GUARD cleanup
echo "DONE: $OUT"
