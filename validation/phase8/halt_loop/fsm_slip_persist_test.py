#!/usr/bin/env python3
"""Decision-function test of the persistence-gated slip HALT (fake clock, no simulator): the old code (state_machine_node.PRE_PERSIST.py) HALTs on a single sample above 0.5,
the new code only after the metric has stayed above 0.5 for SLIP_HALT_PERSIST_S. Usage: fsm_slip_persist_test.py <old.py> <new.py>"""
import importlib.util, os, sys
from types import SimpleNamespace
os.environ.setdefault('ROS_DOMAIN_ID', '97')
import rclpy
def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m
def run(mod, slip_series, dt=0.05, imu_dt=0.01):
    """slip_series: list of slip values, one per /wheel_slip message (20 Hz); the IMU/decision tick runs at 100 Hz between messages. Returns (list of (t,state) transitions, max HALT dwell)."""
    rclpy.init(); n = mod.RampStateMachine(); n.get_logger().set_level(50)
    clock = [0.0]; n._now_s = lambda: clock[0]
    out = []; last = n.state
    for v in slip_series:
        n._on_wheel_slip(SimpleNamespace(data=v))
        for _ in range(int(round(dt / imu_dt))):
            clock[0] += imu_dt; n._update_state(0.0, 0.0)
            if n.state != last: out.append((round(clock[0], 2), n.state)); last = n.state
    n.destroy_node(); rclpy.shutdown(); return out
old, new = load(sys.argv[1], 'old'), load(sys.argv[2], 'new')
P = 0.25
S = {
 'single 0.05 s spike to 1.0 (onset artefact)':          [0.0] * 5 + [1.0] + [0.0] * 10,
 '0.10 s spike (decel artefact, longest seen)':          [0.0] * 5 + [1.4, 1.4] + [0.0] * 10,
 '20 onset artefacts, 0.05 s on / 0.10 s off (loop feed)': ([1.0, 0.0, 0.0] * 20),
 'sustained 0.30 s above threshold':                      [0.0] * 5 + [0.9] * 6 + [0.0] * 10,
 'sustained 2 s (real tracking failure)':                 [0.0] * 5 + [0.8] * 40 + [0.0] * 10,
 'just under the persistence (0.20 s)':                   [0.0] * 5 + [0.9] * 4 + [0.0] * 10,
}
EXPECT_NEW_HALT = {'single 0.05 s spike to 1.0 (onset artefact)': False, '0.10 s spike (decel artefact, longest seen)': False, '20 onset artefacts, 0.05 s on / 0.10 s off (loop feed)': False,
                   'sustained 0.30 s above threshold': True, 'sustained 2 s (real tracking failure)': True, 'just under the persistence (0.20 s)': False}
bad = 0
for k, ser in S.items():
    o, n_ = run(old, ser), run(new, ser)
    halted = any(s == 'HALT' for _, s in n_); ok = halted == EXPECT_NEW_HALT[k]; bad += (not ok)
    first = next((t for t, s in n_ if s == 'HALT'), None)
    print(f'{k}\n    old: {"HALT at " + str(next(t for t,s in o if s=="HALT")) + " s" if any(s=="HALT" for _,s in o) else "no HALT"}   new: {"HALT at " + str(first) + " s" if halted else "no HALT"}   {"as expected" if ok else "UNEXPECTED"}')
    if first is not None and k.startswith('sustained'):
        onset = 0.05 * 5  # slip series starts above threshold at message index 5 -> t = 0.25 s
        print(f'        HALT latency after the metric first exceeded 0.5: {first - onset:.2f} s (persistence {P} s)')
# margin and stall stay immediate
rclpy.init(); n = new.RampStateMachine(); n.get_logger().set_level(50); clock = [0.0]; n._now_s = lambda: clock[0]
n._stability_margin = 0.05; n._update_state(0.0, 0.0); ok1 = n.state == 'HALT'
n2 = new.RampStateMachine(); n2.get_logger().set_level(50); n2._motion_stalled = True; n2._update_state(0.0, 0.0); ok2 = n2.state == 'HALT'
n.destroy_node(); n2.destroy_node(); rclpy.shutdown()
print(f'margin HALT still immediate: {ok1}; stall HALT still immediate: {ok2}'); bad += (not ok1) + (not ok2)
print('ALL AS EXPECTED' if not bad else f'{bad} UNEXPECTED'); sys.exit(1 if bad else 0)
