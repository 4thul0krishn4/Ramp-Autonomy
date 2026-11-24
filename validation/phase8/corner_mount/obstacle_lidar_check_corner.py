#!/usr/bin/env python3
"""Obstacle-LiDAR re-check for the corner-mounted obstacle_lidar_link (x=-0.30, y=-0.22, z=0.5 rel. base_link - see
../README.md). The original bearing/true_m constants (behind: 180 deg/0.65 m, side: -90 deg/0.85 m - see
../perception_regression/obstacle_lidar_check.py) were derived for the OLD centred mount (0,0,0.5) and do not apply
here: moving the sensor's (x,y) changes both the apparent bearing AND the range to each fixed-world obstacle, even
though the sensor's own angular convention is unchanged (obstacle_lidar_joint has rpy=0 0 0, so its local axes stay
parallel to base_link's). This script's near()/true_m windows are re-derived analytically from the known obstacle box
positions (ramp_world_sharp_obstacles_TESTONLY.sdf: test_obstacle_behind center (0.2,0,0.6) half-extent 0.15 both x/y;
test_obstacle_side center (1.0,-1.0,0.6) half-extent 0.15; robot spawns at (1.0,0.0,yaw=0)) and the NEW sensor's world
offset at spawn (1.0-0.30, 0.0-0.22) = (0.70,-0.22) - nearest-point-on-box-to-sensor geometry, not simply re-used from
the old mount. Predicted here, then checked against the live /scan_360 below (verify, don't just trust the math):
  behind: nearest point (0.35,-0.15) -> range 0.357 m, bearing atan2(0.07,-0.35) = 168.7 deg
  side:   nearest point (0.85,-0.85) -> range 0.648 m, bearing atan2(-0.63,0.15) = -76.6 deg
Usage: obstacle_lidar_check_corner.py <out.json>"""
import json, math, statistics, sys, time
if len(sys.argv) < 2:
    raise SystemExit('usage: obstacle_lidar_check_corner.py <out.json>')
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from ramp_autonomy_msgs.msg import RampPercept

OUT = sys.argv[1]


class N(Node):
    def __init__(self):
        super().__init__('obstacle_lidar_check_corner')
        self.scans, self.grids, self.per = [], [], []
        self.create_subscription(LaserScan, '/scan_360', lambda m: self.scans.append(m), 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: self.grids.append(m), 5)
        self.create_subscription(RampPercept, '/ramp_perception', lambda m: self.per.append((m.breakpoint_distance_m, m.slope_deg, m.confidence)), 10)


rclpy.init(); n = N(); end = time.time() + 12
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
res = {}
S = n.scans[-1]
ang = S.angle_min + np.arange(len(S.ranges)) * S.angle_increment
rng = np.array(S.ranges, dtype=float)
deg = np.degrees(ang)
def near(center, half):
    d = (deg - center + 180) % 360 - 180
    m = (np.abs(d) <= half) & np.isfinite(rng) & (rng > S.range_min) & (rng < 3.0)
    return m
# Predicted bearings (168.7, -76.6) with generous windows (matching the original script's own margin style) to absorb
# small spawn-pose/yaw tolerance - see module docstring for the derivation.
mb = near(168.7, 20); ms = near(-76.6, 25)
if mb.any():
    i = np.argmin(np.where(mb, rng, np.inf)); res['behind'] = dict(bearing_deg=round(float(deg[i]), 1), range_m=round(float(rng[i]), 3), predicted_m=0.357, predicted_bearing_deg=168.7)
else:
    res['behind'] = dict(FOUND=False)
if ms.any():
    i = np.argmin(np.where(ms, rng, np.inf)); rmin = rng[i]
    span = ms & (rng < rmin + 0.25)
    res['side'] = dict(bearing_deg=round(float(deg[i]), 1), range_m=round(float(rmin), 3), predicted_m=0.648, predicted_bearing_deg=-76.6, span_deg=round(float(deg[span].max() - deg[span].min()), 1))
else:
    res['side'] = dict(FOUND=False)
# Full-ring self-occlusion scan: any unexpectedly short return (< 0.3 m) outside the two known-obstacle windows would
# indicate the sensor is seeing its own mast/chassis/mount structure, not the test obstacles.
all_valid = np.isfinite(rng) & (rng > S.range_min) & (rng < 12.0)
suspicious = all_valid & ~mb & ~ms & (rng < 0.30)
res['self_occlusion_scan'] = dict(n_total=int(len(rng)), n_valid=int(all_valid.sum()),
                                   n_suspicious_short_outside_known_obstacles=int(suspicious.sum()),
                                   suspicious_bearings=[round(float(x), 1) for x in deg[suspicious][:20]])
G = n.grids[-1]; a = np.array(G.data, dtype=int)
res['local_costmap'] = dict(occupied_ge90=int((a >= 90).sum()), total=int(a.size))
if n.per:
    v = [p for p in n.per[-30:] if p[0] >= 0] or n.per[-30:]
    res['ramp_perception_at_rest'] = dict(breakpoint_m=round(statistics.median(p[0] for p in v), 3), slope_deg=round(statistics.median(p[1] for p in v), 2), confidence=round(statistics.median(p[2] for p in v), 2))
np.savez_compressed(OUT.replace('.json', '_costmap.npz'), data=a.reshape(G.info.height, G.info.width), res=G.info.resolution, ox=G.info.origin.position.x, oy=G.info.origin.position.y)
json.dump(res, open(OUT, 'w'), indent=1); print(json.dumps(res, indent=1))
rclpy.shutdown()
