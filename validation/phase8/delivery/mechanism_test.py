#!/usr/bin/env python3
"""Detach-geometry test at the delivery point: robot spawned on the Floor 2 pad with the payload attached; release, let the unloader push the payload onto the dock, then drive the robot away and
check where everything ends up (ground-truth poses). Usage: mechanism_test.py <out.json>"""
import json, math, os, subprocess, sys, time
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
        res[nm] = tuple(round(f(k), 3) for k in ('x', 'y', 'z'))
    return res
rclpy.init(); n = Node('mechanism_test'); pub = n.create_publisher(Twist, '/cmd_vel_nav', 10)
def spin(s):
    e = time.time() + s
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.02)
def drive(dist, v):
    e = time.time() + abs(dist / v)
    while time.time() < e:
        t = Twist(); t.linear.x = v; pub.publish(t); spin(0.03)
    for _ in range(10): pub.publish(Twist()); spin(0.05)
    spin(1.0)
log = {}
unl = subprocess.Popen(['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'payload_unloader.py'), '--log', OUT + '.unloader.log'] + os.environ.get('UNLOADER_ARGS', '').split(), stdout=open(OUT + '.unloader.out', 'w'), stderr=subprocess.STDOUT)
spin(10.0); log['settled'] = poses(); print('settled', log['settled'], flush=True)
subprocess.run(['gz', 'topic', '-t', '/payload/detach', '-m', 'gz.msgs.Empty', '-p', ' '], capture_output=True)
spin(8.0); log['after_release'] = poses(); print('after release + unload', log['after_release'], flush=True)
drive(-1.5, 0.25); log['after_robot_reverse'] = poses(); print('robot reversed 1.5 m', log['after_robot_reverse'], flush=True)
drive(3.0, 0.25); log['after_robot_forward'] = poses(); print('robot forward 3 m', log['after_robot_forward'], flush=True)
try: unl.wait(timeout=8)
except Exception: unl.kill()
json.dump(log, open(OUT, 'w'), indent=1); os._exit(0)
