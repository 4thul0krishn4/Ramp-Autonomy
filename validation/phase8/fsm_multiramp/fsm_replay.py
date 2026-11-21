#!/usr/bin/env python3
"""Offline REPLAY of a live run's recorded FSM inputs through a state_machine_node.py (old or new): same decision function, same inputs, no simulator.
Usage: fsm_replay.py <run.json> <state_machine_node.py> [--quiet]
Fidelity check: replaying the ORIGINAL code on a run recorded with the ORIGINAL code must reproduce the live /ramp_state transition sequence."""
import importlib.util, json, os, sys
os.environ.setdefault('ROS_DOMAIN_ID', '97')          # isolated: the replay node must not talk to a running sim
import rclpy

def load(path):
    spec = importlib.util.spec_from_file_location('fsm_under_test', path); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m

def replay(run, path):
    m = load(path); rclpy.init(); node = m.RampStateMachine(); node.get_logger().set_level(50)
    seq, prev_p = [], 0.0
    from types import SimpleNamespace
    pc = run.get('percept') or []; k = 0        # newer runs carry the raw perception stream (with slope_deg): feed it through the node's own callback
    for (t, pitch, margin, slip, stall, conf, dist, vx, live) in run['imu']:
        node._stability_margin, node._wheel_slip, node._motion_stalled = margin, slip, bool(stall)
        if pc:
            while k < len(pc) and pc[k][0] <= t:
                node._on_percept(SimpleNamespace(breakpoint_distance_m=pc[k][1], slope_deg=pc[k][2], confidence=pc[k][3])); k += 1
        else:
            node._percept_confidence, node._percept_breakpoint_distance = conf, dist
        if hasattr(node, '_body_vx'): node._body_vx = vx
        before = node.state; node._update_state(pitch, pitch - prev_p); prev_p = pitch
        if node.state != before: seq.append((t, before, node.state, pitch, vx))
    node.destroy_node(); rclpy.shutdown(); return seq

if __name__ == '__main__':
    run = json.load(open(sys.argv[1])); seq = replay(run, sys.argv[2])
    live = [(t['t'], t['frm'], t['to']) for t in run['transitions']]
    print(f'replay of {os.path.basename(sys.argv[2])}: {len(seq)} transitions (live run recorded {len(live)})')
    if '--quiet' not in sys.argv:
        for r in seq: print(f'   t={r[0]:7.2f}  {r[1]:>10} -> {r[2]:<10} pitch={r[3]:7.2f}  vx={r[4]:.3f}')
    def collapse(states):
        out = []
        for x in states:
            if x != 'HALT' and (not out or out[-1] != x): out.append(x)
        return out
    # The live probe samples /ramp_state at 10 Hz and the replay steps at IMU rate, so one-tick HALT blips exist only in the replay: compare the non-HALT state sequences.
    # states shorter than one /ramp_state publish period (0.1 s) can never appear in a live 10 Hz log (e.g. ASCENT lasts ONE IMU tick at the ramp foot, see README)
    timeline = [('APPROACH', 0.0)] + [(x[2], x[0]) for x in seq]
    visible = [st for i, (st, t0) in enumerate(timeline) if i + 1 == len(timeline) or timeline[i + 1][1] - t0 >= 0.09]
    a = collapse(visible)
    live_states = collapse(['APPROACH'] + [t['to'] for t in run['transitions']])
    print('non-HALT state sequence  replay:', ' > '.join(a)); print('non-HALT state sequence  live:  ', ' > '.join(live_states))
    print('IDENTICAL non-HALT sequence to the live run' if a == live_states else 'DIFFERS from the live non-HALT sequence')
