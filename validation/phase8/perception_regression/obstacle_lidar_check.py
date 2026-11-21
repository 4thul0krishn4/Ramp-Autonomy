#!/usr/bin/env python3
"""Re-run of the obstacle-LiDAR verification (validation/phase8/obstacle_lidar/README.txt) in world:=sharp_obstacles_TESTONLY on the corrected geometry.
Robot at rest at the spawn pose; reads /scan_360, /local_costmap/costmap and /ramp_perception. Original standard: behind obstacle bearing ~180 deg range 0.64 m
(true 0.65 m); side obstacle bearing ~-90 deg range 0.85 m (true 0.85 m), spanning ~16 deg; local costmap 631 of 14400 cells occupied (>=90); /ramp_perception at rest
slope 17.68 deg, breakpoint 2.55 m, confidence 0.97 (2.61-2.63 m in the breakpoint table). Usage: obstacle_lidar_check.py <out.json>"""
import json, math, statistics, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from ramp_autonomy_msgs.msg import RampPercept

OUT = sys.argv[1]


class N(Node):
    def __init__(self):
        super().__init__('obstacle_lidar_check')
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
mb = near(180, 25); ms = near(-90, 40)
if mb.any():
    i = np.argmin(np.where(mb, rng, np.inf)); res['behind'] = dict(bearing_deg=round(float(deg[i]), 1), range_m=round(float(rng[i]), 3), true_m=0.65)
if ms.any():
    i = np.argmin(np.where(ms, rng, np.inf)); rmin = rng[i]
    span = ms & (rng < rmin + 0.25)
    res['side'] = dict(bearing_deg=round(float(deg[i]), 1), range_m=round(float(rmin), 3), true_m=0.85, span_deg=round(float(deg[span].max() - deg[span].min()), 1))
G = n.grids[-1]; a = np.array(G.data, dtype=int)
res['local_costmap'] = dict(occupied_ge90=int((a >= 90).sum()), total=int(a.size), original_631_of_14400=True)
if n.per:
    v = [p for p in n.per[-30:] if p[0] >= 0] or n.per[-30:]
    res['ramp_perception_at_rest'] = dict(breakpoint_m=round(statistics.median(p[0] for p in v), 3), slope_deg=round(statistics.median(p[1] for p in v), 2), confidence=round(statistics.median(p[2] for p in v), 2))
np.savez_compressed(OUT.replace('.json', '_costmap.npz'), data=a.reshape(G.info.height, G.info.width), res=G.info.resolution, ox=G.info.origin.position.x, oy=G.info.origin.position.y)
json.dump(res, open(OUT, 'w'), indent=1); print(json.dumps(res, indent=1))
rclpy.shutdown()
