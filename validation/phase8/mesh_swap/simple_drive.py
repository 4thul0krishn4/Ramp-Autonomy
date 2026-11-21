#!/usr/bin/env python3
"""Minimal raw-cmd_vel forward drive for the screenshot tour ONLY - not a quantitative trial (no harness_guard gate: with the GUI open and rendering the new
mesh, RTF drops well below the 0.95 gate every quantitative test in this project uses, which is expected and fine for a photo op, not for a measurement -
the actual regression check ran headless, gated, and passed separately). Drives forward at a fixed speed for a fixed duration.
Usage: simple_drive.py <vx> <duration_s>"""
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
vx, dur = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); n = Node('simple_drive'); pub = n.create_publisher(Twist, '/cmd_vel_nav', 10)
end = time.time() + dur
while time.time() < end:
    t = Twist(); t.linear.x = vx; pub.publish(t); rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(10): pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
rclpy.shutdown()
