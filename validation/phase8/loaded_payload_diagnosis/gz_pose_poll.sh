#!/bin/bash
# Polls the real Gazebo world-frame pose of ramp_robot every ~2s while
# the loaded-payload drive test runs, so real X/Z can be compared
# directly against odometry and IMU pitch on the same rough timeline.
source /opt/ros/jazzy/setup.bash
OUT=$1
DURATION=$2
END=$((SECONDS + DURATION))
echo "t_wall,x,y,z" > "$OUT"
while [ $SECONDS -lt $END ]; do
  LINE=$(timeout 2 gz topic -e -t /world/ramp_world_sharp/dynamic_pose/info -n 1 2>/dev/null | \
    grep -A 4 'name: "ramp_robot"' | grep -E "x:|y:|z:" | awk '{print $2}' | tr '\n' ',')
  echo "${SECONDS},${LINE}" >> "$OUT"
  sleep 2
done
