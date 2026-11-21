#!/usr/bin/env python3
"""Smoke test of the no-op / basic DetachableJoint wiring: payload model exists, is held to the chassis while the robot moves, and stays behind after /payload/detach.
Prints the payload pose relative to the robot at each stage. Usage: detach_smoke.py <out.json>"""
import json, subprocess, sys, time, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
OUT = sys.argv[1]
def poses():
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/construction_site/dynamic_pose/info', '-n', '1'], capture_output=True, text=True, timeout=5).stdout
    res = {}
    for nm in ('ramp_robot', 'payload_box'):
        i = out.find(f'name: "{nm}"')
        if i < 0: continue
        blk = out[i:i + 700]; p = blk.find('position {'); pb = blk[p:p + 200]
        def f(k):
            a = pb.find(k + ':'); b = pb.find('\n', a); return float(pb[a + len(k) + 1:b].strip())
        res[nm] = (f('x'), f('y'), f('z'))
    return res
rclpy.init(); n = Node('detach_smoke'); pub = n.create_publisher(Twist, '/cmd_vel_nav', 10)
def spin(s):
    e = time.time() + s
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.02)
def drive(dist, v=0.3):
    e = time.time() + dist / v
    while time.time() < e:
        t = Twist(); t.linear.x = v; pub.publish(t); spin(0.03)
    for _ in range(10): pub.publish(Twist()); spin(0.05)
    spin(1.0)
def rel(p):
    if 'ramp_robot' not in p or 'payload_box' not in p: return None
    a, b = p['ramp_robot'], p['payload_box']; return tuple(round(b[i] - a[i], 3) for i in range(3))
spin(5.0); log = {}
p0 = poses(); log['t0_settled'] = dict(poses=p0, rel=rel(p0)); print('settled: payload rel robot', rel(p0), p0, flush=True)
drive(1.5); p1 = poses(); log['after_drive_attached'] = dict(poses=p1, rel=rel(p1)); print('after 1.5 m drive (attached): rel', rel(p1), flush=True)
r = subprocess.run(['gz', 'topic', '-t', '/payload/detach', '-m', 'gz.msgs.Empty', '-p', ' '], capture_output=True, text=True); log['detach_cmd'] = (r.returncode, r.stdout[-100:], r.stderr[-100:]); spin(2.0)
drive(1.5); p2 = poses(); log['after_detach_drive'] = dict(poses=p2, rel=rel(p2)); print('after detach + 1.5 m drive: rel', rel(p2), 'payload pose', p2.get('payload_box'), flush=True)
json.dump(log, open(OUT, 'w'), indent=1)
import os; os._exit(0)
