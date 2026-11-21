#!/usr/bin/env python3
"""Raw self-occlusion sample for both corner-relocated LiDARs (see ../README.md for the design and the predicted-vs-
empirical distinction). Robot stationary on construction_site (flat, open ground - no ramp/obstacle structure nearby,
so every short/anomalous return has to come from the robot's own body). Samples /scan (rolled ramp sensor) and
/scan_360 (level obstacle sensor) and reports the full range/bearing histogram so an anomaly can be judged against the
analytically-predicted pattern (module docstring in obstacle_lidar_check_corner.py; ramp sensor expected to self-
return almost immediately on its own deck/backward cone, same as the pre-relocation design) rather than assumed either
way. Usage: self_occlusion_raw.py <out.json>"""
import json, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

OUT = sys.argv[1]


class N(Node):
    def __init__(self):
        super().__init__('self_occlusion_raw')
        self.ramp, self.obs = [], []
        self.create_subscription(LaserScan, '/scan', lambda m: self.ramp.append(m), 10)
        self.create_subscription(LaserScan, '/scan_360', lambda m: self.obs.append(m), 10)


def summarize(S):
    ang = np.degrees(S.angle_min + np.arange(len(S.ranges)) * S.angle_increment)
    rng = np.array(S.ranges, dtype=float)
    valid = np.isfinite(rng) & (rng > S.range_min) & (rng < S.range_max)
    out = dict(n_total=int(len(rng)), n_valid=int(valid.sum()),
               range_min_observed=round(float(rng[valid].min()), 3) if valid.any() else None,
               range_max_observed=round(float(rng[valid].max()), 3) if valid.any() else None)
    # Forward cone actually used by ramp_classifier_node.py (-80..45 deg) - what matters for ground-profile accuracy.
    fwd = valid & (ang >= -80) & (ang <= 45)
    out['forward_cone_-80_45'] = dict(n_valid=int(fwd.sum()),
                                       ranges_sample=[round(float(x), 3) for x in rng[fwd][:15]])
    # Histogram of short returns (<0.15 m) by bearing bucket, to localize any self-structure hit.
    short = valid & (rng < 0.15)
    out['short_returns_lt_0.15m'] = dict(n=int(short.sum()), bearings=[round(float(x), 1) for x in ang[short][:30]])
    return out


rclpy.init(); n = N(); end = time.time() + 10
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
res = dict(ramp_lidar_scan=summarize(n.ramp[-1]) if n.ramp else None,
           obstacle_lidar_scan_360=summarize(n.obs[-1]) if n.obs else None)
json.dump(res, open(OUT, 'w'), indent=1); print(json.dumps(res, indent=1))
rclpy.shutdown()
