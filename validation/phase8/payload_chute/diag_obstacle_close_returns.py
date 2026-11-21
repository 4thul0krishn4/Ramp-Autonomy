#!/usr/bin/env python3
"""Diagnose the new 0.35-0.6 m returns seen in the obstacle LiDAR's forward cone after adding the payload chute -
prints exact bearing for every valid return under 1.0 m, to pinpoint what structure is being hit (chute? payload?
something pre-existing in construction_site unrelated to this change?)."""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

rclpy.init()


class N(Node):
    def __init__(self):
        super().__init__('diag_obstacle')
        self.scans = []
        self.create_subscription(LaserScan, '/scan_360', lambda m: self.scans.append(m), 10)


n = N(); end = time.time() + 8
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
S = n.scans[-1]
ang = np.degrees(S.angle_min + np.arange(len(S.ranges)) * S.angle_increment)
rng = np.array(S.ranges, dtype=float)
valid = np.isfinite(rng) & (rng > S.range_min) & (rng < 1.0)
idx = np.where(valid)[0]
for i in idx:
    print(f'bearing={ang[i]:7.1f}deg  range={rng[i]:.3f}m')
rclpy.shutdown()
