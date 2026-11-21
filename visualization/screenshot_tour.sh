#!/bin/bash
# Launch the full stack with the GUI (unloaded by default), then capture screenshots of the Gazebo window from a list of camera poses.
# Usage: visualization/screenshot_tour.sh <out_dir> <loaded|unloaded> "name:x:y:z:lookx:looky:lookz" ...
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}; OUT=$1; PAYLOAD=$2; shift 2
MASS=0.0; CG=0.0; [ "$PAYLOAD" = "loaded" ] && { MASS=100.0; CG=0.1; }
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash; export DISPLAY=${DISPLAY:-:0}
GUARD="python3 $WS/validation/phase8/harness/harness_guard.py"
$GUARD cleanup && $GUARD preflight || exit 2
mkdir -p $OUT; L=$OUT/tour_launch.log
setsid nohup env -u GTK_PATH -u GTK_EXE_PREFIX ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=$MASS payload_cg_offset:=$CG headless:=false > $L 2>&1 < /dev/null &
disown
for i in $(seq 1 90); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
grep -q "Managed nodes are active" $L || { echo "never active"; $GUARD cleanup; exit 2; }
sleep 8; python3 $WS/visualization/x11_window.py resize 1600 900 100 60; sleep 3
XID=$(python3 $WS/visualization/x11_window.py id)
for spec in "$@"; do
  IFS=: read name x y z lx ly lz <<< "$spec"
  python3 - "$x" "$y" "$z" "$lx" "$ly" "$lz" > /tmp/camreq.txt <<'PY'
import sys, math
x, y, z, lx, ly, lz = map(float, sys.argv[1:7])
yaw = math.atan2(ly - y, lx - x); pitch = math.atan2(-(lz - z), math.hypot(lx - x, ly - y))
cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
qw, qx, qy, qz = cy * cp, -sy * sp * 0, sp * cy, sy * cp        # roll 0
print(f'pose: {{position: {{x: {x} y: {y} z: {z}}} orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}')
PY
  gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req "$(cat /tmp/camreq.txt)" > /dev/null 2>&1
  sleep 3; import -window $XID $OUT/$name.png; echo "captured $name"
done
$GUARD cleanup
