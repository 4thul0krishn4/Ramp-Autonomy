#!/usr/bin/env python3
"""Sim-side payload unloader (the 'roller-top AMR docking at a receiving table' transfer): when the DetachableJoint reports `detached`, push the released payload sideways (+y, toward the
delivery dock) with a persistent link wrench (gz ApplyLinkWrench world system) until its centre is over the dock edge, then clear the wrench. The payload's low friction (0.3) lets it slide off the
deck; the dock (level with the deck, 5 mm lower) receives it and friction stops it. This is an environment/actuator emulator, not part of the robot's autonomy stack.
Usage: payload_unloader.py [--world construction_site] [--force 400] [--y-stop 0.65]"""
import argparse, sys, threading, time
from gz.transport13 import Node
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.boolean_pb2 import Boolean
ap = argparse.ArgumentParser(); ap.add_argument('--world', default='construction_site'); ap.add_argument('--force', type=float, default=450.0)
ap.add_argument('--y-stop', type=float, default=0.40, help='clear the push when the payload centre is this far (+y) from where it was released')
ap.add_argument('--force2', type=float, default=450.0, help='second-phase force once the payload centre has passed the chassis edge (roughly balances the friction on the dock)')
ap.add_argument('--y-phase2', type=float, default=0.30)
ap.add_argument('--timeout', type=float, default=5.0); ap.add_argument('--log', default='')
a = ap.parse_args()
node = Node(); state = {'detached': False, 'pose': None, 'y0': None, 't0': None, 'pushing': False, 'done': False}; lock = threading.Lock(); log = []; traj = []
LINK = 'payload_box::payload_link'
def say(m):
    line = f'[{time.time():.3f}] {m}'; print(line, flush=True); log.append(line)
def wrench_msg(fy):
    m = EntityWrench(); m.entity.name = LINK; m.entity.type = Entity.LINK; m.wrench.force.y = fy; return m
def on_pose(msg):
    for p in msg.pose:
        if p.name == 'payload_box':
            with lock:
                state['pose'] = (p.position.x, p.position.y, p.position.z)
                if state.get('pushing'): traj.append((round(time.time() - state['tp'], 3), round(p.position.y, 4), round(p.position.z, 4)))
def on_state(msg):
    with lock:
        if msg.data == 'detached' and not state['detached']:
            state['detached'] = True; state['t0'] = time.time(); state['y0'] = state['pose'][1] if state['pose'] else None
            say(f'payload detached; released at y = {state["y0"]}')
pub_persist = node.advertise(f'/world/{a.world}/wrench/persistent', EntityWrench)
pub_clear = node.advertise(f'/world/{a.world}/wrench/clear', Entity)
node.subscribe(Pose_V, f'/world/{a.world}/dynamic_pose/info', on_pose)
node.subscribe(StringMsg, '/payload/state', on_state)
say('payload unloader running')
while not state['done']:
    time.sleep(0.02)
    with lock:
        det, pose, y0, t0, pushing = state['detached'], state['pose'], state['y0'], state['t0'], state['pushing']
    if det and not pushing and t0 is not None and time.time() - t0 > 0.3 and pose is not None:
        state['pushing'] = True
        y0 = pose[1]; state['y0'] = y0; state['tp'] = time.time()
        ok = pub_persist.publish(wrench_msg(a.force)); say(f'pushing: {a.force} N +y from y = {y0:.3f} (publish ok={ok})')
    if pushing and pose is not None:
        moved = pose[1] - state['y0']
        if not state.get('phase2') and moved >= a.y_phase2:
            state['phase2'] = True; pub_persist.publish(wrench_msg(a.force2)); say(f'phase 2: {a.force2} N at moved = {moved:.3f}')
        if moved >= a.y_stop or time.time() - state['tp'] > a.timeout:
            ok = pub_clear.publish(Entity(name=LINK, type=Entity.LINK))
            say(f'push cleared after moving {moved:.3f} m in {time.time() - state["tp"]:.2f} s'); state['done'] = True
time.sleep(4.0)
with lock: say(f'final payload pose: {state["pose"]}')
if a.log:
    open(a.log, 'w').write('\n'.join(log)); import json; json.dump(traj, open(a.log + '.traj.json', 'w'))
