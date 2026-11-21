#!/bin/bash
# Launch the FULL current stack on construction_site with the Gazebo GUI on your screen, follow the robot with the camera, and (optionally) run the round trip.
# Usage:  visualization/run_live_gui.sh [loaded|unloaded] [--no-trip]
#   loaded   = 100 kg payload, +0.1 m CG (default)      unloaded = no payload
#   --no-trip = just bring the world up and leave it running (drive it yourself / watch)
# Purely visualization tooling: it does not touch the autonomy code, only the environment the GUI process inherits (see visualization/README.md).
# To stop everything afterwards:  python3 validation/phase8/harness/harness_guard.py cleanup
WS=${RAMP_WS:?set RAMP_WS to the ramp_autonomy_ws workspace root}
PAYLOAD=${1:-loaded}; TRIP=1; [ "$2" = "--no-trip" ] && TRIP=0
MASS=100.0; CG=0.1; [ "$PAYLOAD" = "unloaded" ] && { MASS=0.0; CG=0.0; }
source /opt/ros/jazzy/setup.bash; source $WS/install/setup.bash
export DISPLAY=${DISPLAY:-:0}
GUARD="python3 $WS/validation/phase8/harness/harness_guard.py"
$GUARD cleanup && $GUARD preflight || exit 2
mkdir -p $WS/visualization/live_run; L=$WS/visualization/live_run/launch_${PAYLOAD}_gui.log
# THE FIX: the VS Code snap terminal exports GTK_PATH / GTK_EXE_PREFIX pointing into /snap/code/...; GTK then loads the snap's
# libcanberra-gtk-module.so, whose RPATH drags /snap/core20's old libpthread into the GUI process and it dies with
# "symbol lookup error ... __libc_pthread_init, version GLIBC_PRIVATE". Unsetting the two variables for the launch is enough.
setsid nohup env -u GTK_PATH -u GTK_EXE_PREFIX ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py \
    world:=construction_site payload_mass:=$MASS payload_cg_offset:=$CG headless:=false > $L 2>&1 < /dev/null &
disown
for i in $(seq 1 60); do grep -q "Managed nodes are active" $L 2>/dev/null && break; sleep 3; done
grep -q "Managed nodes are active" $L || { echo "stack never became active - see $L"; exit 2; }
sleep 5
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: "ramp_robot"' > /dev/null 2>&1
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 --req 'x: -5.0 y: 0.0 z: 3.5' > /dev/null 2>&1
# The GUI's first seconds (shader compile, scene load) depress the real-time factor; wait for it to settle. Same guard threshold - nothing is lowered.
for i in $(seq 1 24); do $GUARD verify > /dev/null 2>&1 && break; sleep 5; done
$GUARD verify || exit 2
echo "Gazebo GUI is up (camera follows ramp_robot). Payload: $PAYLOAD."
if [ $TRIP -eq 1 ]; then
  cd $WS/validation/phase8/construction_site_descent
  python3 arc_turn_followpath_test_ekf.py $PWD/roundtrip_live_gui_${PAYLOAD}.csv 2>&1 | tee roundtrip_live_gui_${PAYLOAD}.log | grep -E "leg [0-9]|goal-verification|TRAVERSAL RESULT|aborted|HALT events|final gz|HARNESS"
else
  echo "World left running. Stop it with: python3 $WS/validation/phase8/harness/harness_guard.py cleanup"
fi
