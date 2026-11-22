#!/bin/bash
# Live GUI screenshots of the new Husky-mesh chassis: launches the full stack (loaded) with the GUI, drives the SAME round-trip driver used for the regression
# check in the background, and grabs close-in camera shots of the robot on ramp 1 and on the populated elevated site once it gets there.
WS=${RAMP_WS:?}
OUT=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash; export DISPLAY=${DISPLAY:-:0}
GUARD="python3 $WS/validation/phase8/harness/harness_guard.py"
$GUARD cleanup && $GUARD preflight || exit 2
mkdir -p $OUT; L=$OUT/tour_launch.log
setsid nohup env -u GTK_PATH -u GTK_EXE_PREFIX ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=construction_site payload_mass:=100.0 payload_cg_offset:=0.1 headless:=false > $L 2>&1 < /dev/null &
disown
for i in $(seq 1 90); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
grep -q "Managed nodes are active" $L || { echo "never active"; $GUARD cleanup; exit 2; }
sleep 8; python3 $WS/visualization/x11_window.py resize 1600 900 100 60; sleep 3
XID=$(python3 $WS/visualization/x11_window.py id)

shoot() {  # name x y z lookx looky lookz
  name=$1; x=$2; y=$3; z=$4; lx=$5; ly=$6; lz=$7
  python3 - "$x" "$y" "$z" "$lx" "$ly" "$lz" > /tmp/camreq_mesh.txt <<'PY'
import sys, math
x, y, z, lx, ly, lz = map(float, sys.argv[1:7])
yaw = math.atan2(ly - y, lx - x); pitch = math.atan2(-(lz - z), math.hypot(lx - x, ly - y))
cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
qw, qx, qy, qz = cy * cp, -sy * sp * 0, sp * cy, sy * cp
print(f'pose: {{position: {{x: {x} y: {y} z: {z}}} orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}')
PY
  gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req "$(cat /tmp/camreq_mesh.txt)" > /dev/null 2>&1
  sleep 3; import -window $XID $OUT/$name.png; echo "captured $name"
}
gt() {  # prints "x y z" of ramp_robot from ground truth
  timeout 3 gz topic -e -t /world/construction_site/dynamic_pose/info -n 1 2>/dev/null | python3 -c "
import sys,re
t=sys.stdin.read(); i=t.find('name: \"ramp_robot\"')
if i<0: print('0 0 0')
else:
    b=t[i:i+500]; p=b.find('position {'); pb=b[p:p+150]
    f=lambda k: re.search(k+r':\s*([\-0-9.]+)',pb).group(1)
    print(f('x'), f('y'), f('z'))
"
}

# close-up on the stationary loaded robot at the gate, before it moves - shows the chassis clearly with the site behind it
shoot gate_closeup -1.5 -3.0 1.3 1.0 0.0 0.4

# Photo-op driving only (NOT the quantitative regression check, which already ran headless/gated and passed separately - see round_trip/). With the GUI open and
# rendering the new mesh, RTF drops well under the 0.95 gate every quantitative driver in this project enforces (real-time-factor here observed ~0.6-0.7, not a
# regression - the guard is measuring GUI-rendering cost, which no quantitative trial in this project ever runs under), so a gated FollowPath driver would
# correctly refuse to run here. Raw cmd_vel forward drive instead (bypasses Nav2/MPPI, not the safety stack - state machine/stability monitor/stall
# detector/watchdog are all still live), ONE long-running command spanning gate -> laydown -> up ramp 1 -> onto the elevated site, sim-time budgeted (wall-clock
# duration divided by the observed ~0.65 RTF) so it reliably reaches both checkpoints even at reduced real-time factor.
cd $WS/validation/phase8/mesh_swap
python3 simple_drive.py 0.35 300 > $OUT/rt_driver.log 2>&1 &
DRIVER=$!

# wait until it's partway up ramp 1 (gz z rises once it's climbing; ramp foot ~28.3, top ~41.0)
for i in $(seq 1 280); do
  read gx gy gz_ <<< "$(gt)"
  awk -v z="$gz_" 'BEGIN{exit !(z+0>1.5)}' && break
  sleep 1
done
read gx gy gz_ <<< "$(gt)"
echo "on the ramp at gz=($gx,$gy,$gz_)"
shoot ramp_climb $(python3 -c "print($gx-3.0)") $(python3 -c "print($gy+3.5)") $(python3 -c "print($gz_+1.8)") $gx $gy $gz_

# wait until it's reached the elevated level (leg 2 target ~ gz z 3.3+, x around 47) - the SAME drive process is still running
for i in $(seq 1 280); do
  read gx gy gz_ <<< "$(gt)"
  awk -v z="$gz_" 'BEGIN{exit !(z+0>3.2)}' && break
  sleep 1
done
read gx gy gz_ <<< "$(gt)"
echo "on the elevated site at gz=($gx,$gy,$gz_)"
shoot elevated_site $(python3 -c "print($gx-4.0)") $(python3 -c "print($gy-4.0)") $(python3 -c "print($gz_+2.2)") $gx $gy $gz_
shoot elevated_site_close $(python3 -c "print($gx-2.0)") $(python3 -c "print($gy-1.5)") $(python3 -c "print($gz_+1.2)") $gx $gy $gz_

kill $DRIVER 2>/dev/null
wait $DRIVER 2>/dev/null
$GUARD cleanup
echo TOUR DONE
