#!/usr/bin/env python3
"""Smoke test of payload_delivery_node.py end to end: robot spawned loaded, already inside the delivery zone and stopped -> expect the node's own DWELL_S + gz release +
mass-zero, all without any test-script command. Just watches /delivery/status and /payload/state, reads back stability_monitor's payload_mass_kg, and checks ground truth.
Usage: trigger_smoke.py <out.json>"""
import json, os, subprocess, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
OUT = sys.argv[1]
# with SPAWN_X on the pad, odom (0,0,0) is right where we want the trigger zone, so pass a matching zone override via ros2 param set once the node is up
def poses():
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/construction_site/dynamic_pose/info', '-n', '1'], capture_output=True, text=True, timeout=5).stdout; res = {}
    for nm in ('ramp_robot', 'payload_box'):
        i = out.find(f'name: "{nm}"')
        if i < 0: continue
        blk = out[i:i + 700]; p = blk.find('position {'); pb = blk[p:p + 200]
        def f(k):
            a = pb.find(k + ':'); b = pb.find('\n', a); return float(pb[a + len(k) + 1:b].strip())
        res[nm] = tuple(round(f(k), 3) for k in ('x', 'y', 'z'))
    return res
rclpy.init(); n = Node('trigger_smoke'); log = []
n.create_subscription(String, '/delivery/status', lambda m: log.append((round(time.time(), 2), 'status', m.data)), 10)
n.create_subscription(String, '/payload/state', lambda m: log.append((round(time.time(), 2), 'payload_state', m.data)), 10)
def spin(s):
    e = time.time() + s
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.02)
spin(3.0)
# SPAWN_X put the robot's odom origin right on the pad (odom (0,0,0) = wherever it spawned); override the zone to match for this isolated smoke test
subprocess.run(['ros2', 'param', 'set', '/payload_delivery', 'zone_x_min', '-2.0'], capture_output=True)
subprocess.run(['ros2', 'param', 'set', '/payload_delivery', 'zone_x_max', '2.0'], capture_output=True)
subprocess.run(['ros2', 'param', 'set', '/payload_delivery', 'zone_y_max', '2.0'], capture_output=True)
spin(15.0)
gp = n.create_client(GetParameters, '/stability_monitor/get_parameters'); gp.wait_for_service(timeout_sec=5.0)
req = GetParameters.Request(names=['payload_mass_kg']); fut = gp.call_async(req)
end = time.time() + 5
while not fut.done() and time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
mass = fut.result().values[0].double_value if fut.done() and fut.result() else None
result = dict(log=log, final_mass_param=mass, poses=poses())
print('final delivery status/payload_state log:', log[-6:], flush=True); print('stability_monitor payload_mass_kg after:', mass, flush=True); print('poses', result['poses'], flush=True)
with open(OUT, 'w') as f: json.dump(result, f, indent=1); f.flush(); os.fsync(f.fileno())
sys.stdout.flush(); os._exit(0)
