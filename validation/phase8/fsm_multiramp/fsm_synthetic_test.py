#!/usr/bin/env python3
"""Synthetic scenario test of the FSM decision function (no simulator): old (ORIGINAL) vs current source. Scenarios use the pitch/velocity signatures measured live
(nose-up climb = -16 deg, nose-down descent = +16 deg, ramp entry/exit over ~0.5 s). Usage: fsm_synthetic_test.py <old.py> <new.py>"""
import importlib.util, os, sys
os.environ.setdefault('ROS_DOMAIN_ID', '97')
import rclpy

def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m

def ramp_profile(sign, hold=3.0, dt=0.02, vx=0.35):
    """flat -> +-16 deg over 0.5 s -> hold -> back to flat over 0.12 s -> 1 s flat; returns [(pitch, vx)]"""
    out = [(0.0, vx)] * 25
    out += [(sign * 16.0 * i / 25, vx) for i in range(1, 26)]
    out += [(sign * 16.0, vx)] * int(hold / dt)
    out += [(sign * 16.0 * (1 - i / 6), vx) for i in range(1, 7)]      # crest: 2.7 deg per 20 ms tick (> TRANSITION_WINDOW_DEG)
    out += [(0.0, vx)] * 50
    return out

def run(mod, name, seq, halt_at=None):
    rclpy.init(); n = mod.RampStateMachine(); n.get_logger().set_level(50)
    labels, prev = [], 0.0
    for i, (p, vx) in enumerate(seq):
        if hasattr(n, '_body_vx'): n._body_vx = vx
        n._stability_margin = 0.05 if (halt_at is not None and halt_at[0] <= i < halt_at[1]) else 1.0
        n._update_state(p, p - prev); prev = p; labels.append(n.state)
    n.destroy_node(); rclpy.shutdown()
    seqs = [labels[0]] + [b for a, b in zip(labels[:-1], labels[1:]) if a != b]
    return seqs

old, new = load(sys.argv[1], 'old'), load(sys.argv[2], 'new')
S = {
 'A one ramp: up, (flat), turn, down':               ramp_profile(-1) + ramp_profile(+1),
 'B TWO ramps: up ramp1, up ramp2, down ramp2, down ramp1': ramp_profile(-1) + ramp_profile(-1) + ramp_profile(+1) + ramp_profile(+1),
 'C up, down, up again (second climb after a descent)': ramp_profile(-1) + ramp_profile(+1) + ramp_profile(-1),
 'D reversing DOWN a ramp facing uphill (pitch -16, vx -0.3) from flat':  [(0.0, -0.3)] * 25 + [(-16.0 * i / 25, -0.3) for i in range(1, 26)] + [(-16.0, -0.3)] * 100,
 'E reversing UP a ramp facing downhill (pitch +16, vx -0.3) from flat':  [(0.0, -0.3)] * 25 + [(16.0 * i / 25, -0.3) for i in range(1, 26)] + [(16.0, -0.3)] * 100,
 'F no odometry yet (vx None): historical forward assumption, pitch -16 from flat': [(0.0, None)] * 25 + [(-16.0 * i / 25, None) for i in range(1, 26)] + [(-16.0, None)] * 100,
}
EXPECT_NEW = {
 'A one ramp: up, (flat), turn, down': ['APPROACH', 'ASCENT', 'TRANSITION', 'EXIT', 'DESCENT', 'EXIT'],
 'B TWO ramps: up ramp1, up ramp2, down ramp2, down ramp1': ['APPROACH', 'ASCENT', 'TRANSITION', 'EXIT', 'ASCENT', 'TRANSITION', 'EXIT', 'DESCENT', 'EXIT', 'DESCENT', 'EXIT'],
 'C up, down, up again (second climb after a descent)': ['APPROACH', 'ASCENT', 'TRANSITION', 'EXIT', 'DESCENT', 'EXIT', 'ASCENT', 'TRANSITION', 'EXIT'],
 'D reversing DOWN a ramp facing uphill (pitch -16, vx -0.3) from flat': ['APPROACH', 'DESCENT'],
 'E reversing UP a ramp facing downhill (pitch +16, vx -0.3) from flat': ['APPROACH', 'ASCENT'],
 'F no odometry yet (vx None): historical forward assumption, pitch -16 from flat': ['APPROACH', 'ASCENT'],
}
bad = 0
for k, seq in S.items():
    o, n_ = run(old, 'o', seq), run(new, 'n', seq)
    ok = n_ == EXPECT_NEW[k]; bad += (not ok)
    print(f'{k}\n    old: {" > ".join(o)}\n    new: {" > ".join(n_)}   {"as expected" if ok else "UNEXPECTED (expected " + " > ".join(EXPECT_NEW[k]) + ")"}')
# HALT mid-ramp recovers to the interrupted state (unchanged behaviour), on the SECOND ramp
seq = ramp_profile(-1) + ramp_profile(-1, hold=6.0)
h0 = len(ramp_profile(-1)) + 25 + 25 + 150     # ~mid-hold of ramp 2
n_ = run(new, 'n', seq, halt_at=(h0, h0 + 20))
ok = n_ == ['APPROACH', 'ASCENT', 'TRANSITION', 'EXIT', 'ASCENT', 'HALT', 'ASCENT', 'TRANSITION', 'EXIT']
bad += (not ok); print(f'G HALT in the middle of the SECOND ramp climb recovers to ASCENT\n    new: {" > ".join(n_)}   {"as expected" if ok else "UNEXPECTED"}')
print('ALL SCENARIOS AS EXPECTED' if not bad else f'{bad} SCENARIO(S) UNEXPECTED'); sys.exit(1 if bad else 0)
