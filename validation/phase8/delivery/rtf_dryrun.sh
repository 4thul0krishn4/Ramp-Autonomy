#!/bin/bash
# RTF dry-run of the (no-op) detachable payload plugin: sim cost measured at MAX SPEED (physics real_time_factor 100 => the achievable RTF = 1 / per-step cost), full stack headless on
# construction_site, robot idle at the gate, payload 100 kg / cg 0.1. Variants (payload_mass / detachable flag / detach-at-start):
#   A  loaded, lumped payload (today's validated configuration)
#   B  loaded, detachable payload ATTACHED and never used (the no-op: plugin declared + separate payload model + fixed joint)
#   D  loaded, detachable, DETACHED once at start (payload a free 100 kg body resting on the chassis - the post-delivery state)
#   C  unloaded (reference)
# usage: rtf_dryrun.sh <out.txt> <order like "A B D C A B D C">
W=${RAMP_WS:?}/install/ramp_autonomy_gazebo/share/ramp_autonomy_gazebo/worlds
source /opt/ros/jazzy/setup.bash; source ${RAMP_WS:?}/install/setup.bash
G="python3 ${RAMP_WS:?}/validation/phase8/harness/harness_guard.py"
OUT=$1; shift
sed 's#<real_time_factor>1.0</real_time_factor>#<real_time_factor>100.0</real_time_factor>#' $W/construction_site.sdf > $W/ramp_world_rtfdry.sdf
echo "# RTF dry-run, max-speed, 12 samples x 5 s per run after 30 s settle; order: $*" > $OUT
for v in "$@"; do
  case $v in A) M=100.0; C=0.1; DET=false; DT=0;; B) M=100.0; C=0.1; DET=true; DT=0;; D) M=100.0; C=0.1; DET=true; DT=1;; C) M=0.0; C=0.0; DET=false; DT=0;; esac
  $G cleanup >/dev/null 2>&1
  setsid nohup ros2 launch ramp_autonomy_bringup ramp_autonomy_bringup.launch.py world:=rtfdry payload_mass:=$M payload_cg_offset:=$C detachable_payload:=$DET headless:=true > /tmp/rtfdry_launch.log 2>&1 < /dev/null &
  disown
  for i in $(seq 1 60); do grep -q "Managed nodes are active" /tmp/rtfdry_launch.log 2>/dev/null && break; sleep 3; done
  sleep 5
  [ $DT = 1 ] && { gz topic -t /payload/detach -m gz.msgs.Empty -p ' ' >/dev/null 2>&1; }
  sleep 25
  vals=""
  for k in $(seq 1 12); do r=$(timeout 8 gz topic -e -t /world/construction_site/stats -n 1 2>/dev/null | grep real_time_factor | awk '{print $2}'); vals="$vals $r"; sleep 5; done
  python3 - "$v" "$vals" >> $OUT <<'PY'
import sys, statistics as st
v=[float(x) for x in sys.argv[2].split()]
print(f'{sys.argv[1]}: median {st.median(v):.3f}  mean {st.mean(v):.3f}  min {min(v):.3f}  max {max(v):.3f}  n={len(v)}  samples: ' + ' '.join(f'{x:.2f}' for x in v))
PY
  tail -1 $OUT
done
$G cleanup >/dev/null 2>&1; rm -f $W/ramp_world_rtfdry.sdf; echo "RTF DRYRUN DONE" >> $OUT
