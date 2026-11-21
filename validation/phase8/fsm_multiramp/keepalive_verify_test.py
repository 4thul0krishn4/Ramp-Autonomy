#!/usr/bin/env python3
"""Live test of the keepalive change (verify-by-read-back instead of blind re-push), run against the full stack at the gate (robot idle, flat_mode profile):
 (a) IDLE: count /parameter_events sets on /controller_server over 40 s - the old keepalive produced one burst every 3 s, the fixed one must produce none;
 (b) SILENT-RESET DETECTION (what the keepalive exists for): corrupt controller_server's parameters behind the switcher's back (what a crash + respawn would do:
     back to defaults) and time how long until they are restored: single double, single integer, and all six at once. Must be restored within ~one keepalive period (3.0 s + 0.2 s).
Usage: keepalive_verify_test.py <out.json>"""
import json, sys, time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, ParameterEvent
from rcl_interfaces.srv import GetParameters, SetParameters
NAMES = ['FollowPath.vx_max', 'FollowPath.vx_min', 'FollowPath.wz_max', 'FollowPath.PathAngleCritic.max_angle_to_furthest', 'FollowPath.PathAlignCritic.cost_weight',
         'FollowPath.PathAlignCritic.cost_power', 'FollowPath.PathAlignCritic.offset_from_furthest']
def dbl(n, v): return Parameter(name=n, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
def integer(n, v): return Parameter(name=n, value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(v)))
class T(Node):
    def __init__(self):
        super().__init__('keepalive_verify_test')
        self.g = self.create_client(GetParameters, '/controller_server/get_parameters'); self.s = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.events = []
        self.create_subscription(ParameterEvent, '/parameter_events', lambda m: self.events.append((time.time(), m.node, [p.name for p in m.changed_parameters])) if m.node == '/controller_server' else None, 50)
    def spin(self, s):
        end = time.time() + s
        while time.time() < end: rclpy.spin_once(self, timeout_sec=0.02)
    def call(self, cli, req, timeout=3.0):
        f = cli.call_async(req); end = time.time() + timeout
        while not f.done() and time.time() < end: rclpy.spin_once(self, timeout_sec=0.01)
        return f.result()
    def read(self):
        r = self.call(self.g, GetParameters.Request(names=NAMES)); return [v.double_value if v.type == 3 else v.integer_value for v in r.values]
    def set(self, params): return self.call(self.s, SetParameters.Request(parameters=params))
def main():
    rclpy.init(); t = T(); t.g.wait_for_service(10); t.s.wait_for_service(10); t.spin(3.0)
    base = t.read(); out = dict(baseline=base)
    print('baseline (flat_mode expected: vx 0.8, -0.8, wz 1.2, 0.2618, w 14, p 1, off 3):', [round(x, 4) for x in base])
    # (a) idle
    n0 = len(t.events); t0 = time.time(); t.spin(40.0); idle = [e for e in t.events[n0:]]
    out['idle_param_events_40s'] = len(idle); print(f'(a) idle 40 s: {len(idle)} parameter-set events on /controller_server  {[ (round(e[0]-t0,1), e[2][:3]) for e in idle[:6]]}')
    # (b) corruption -> restore timing
    cases = {'one double (vx_max 0.1)': [dbl(NAMES[0], 0.1)], 'one integer (align offset 99)': [integer(NAMES[6], 99)],
             'all six back to unrelated defaults': [dbl(NAMES[0], 0.5), dbl(NAMES[1], -0.5), dbl(NAMES[2], 1.9), dbl(NAMES[3], 0.9), dbl(NAMES[4], 10.0), integer(NAMES[6], 20)]}
    res = {}
    for name, ps in cases.items():
        t.spin(1.0); ok = t.set(ps); tset = time.time(); restored = None
        while time.time() - tset < 12.0:
            cur = t.read()
            if all(abs(a - b) < 1e-6 for a, b in zip(cur, base)): restored = time.time() - tset; break
            t.spin(0.05)
        res[name] = None if restored is None else round(restored, 2); print(f'(b) {name}: restored after {res[name]} s (set accepted: {all(r.successful for r in ok.results)})')
    out['restore_s'] = res; json.dump(out, open(sys.argv[1], 'w'), indent=1)
    good = out['idle_param_events_40s'] == 0 and all(v is not None and v <= 3.6 for v in res.values())
    print('KEEPALIVE_VERIFY_RESULT:', 'PASS' if good else 'FAIL'); sys.stdout.flush()
    import os; os._exit(0 if good else 1)
main()
