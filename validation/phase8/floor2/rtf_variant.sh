#!/bin/bash
# usage: rtf_variant.sh <variant_file>   -> installs it as ramp_world_f2var.sdf, launches headless, prints median RTF over 40 s after a 25 s settle
V=$1; W=${RAMP_WS:?}/install/ramp_autonomy_gazebo/share/ramp_autonomy_gazebo/worlds
source /opt/ros/jazzy/setup.bash; source ${RAMP_WS:?}/install/setup.bash
python3 ${RAMP_WS:?}/validation/phase8/harness/harness_guard.py cleanup >/dev/null
cp $V $W/ramp_world_f2var.sdf
setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=f2var payload_mass:=0.0 payload_cg_offset:=0.0 headless:=true > /tmp/f2var_launch.log 2>&1 < /dev/null &
disown
for i in $(seq 1 60); do grep -q "Managed nodes are active" /tmp/f2var_launch.log 2>/dev/null && break; sleep 3; done
sleep 25
vals=""
for k in $(seq 1 8); do r=$(timeout 8 gz topic -e -t /world/construction_site/stats -n 1 2>/dev/null | grep real_time_factor | awk '{print $2}'); vals="$vals $r"; sleep 5; done
python3 - "$vals" <<'PY'
import sys, statistics
v=[float(x) for x in sys.argv[1].split()]
print('   RTF samples:', ' '.join(f'{x:.2f}' for x in v), '| median %.2f' % statistics.median(v))
PY
python3 ${RAMP_WS:?}/validation/phase8/harness/harness_guard.py cleanup >/dev/null
rm -f $W/ramp_world_f2var.sdf
