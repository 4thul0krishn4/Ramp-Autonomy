#!/usr/bin/env python3
"""Unit test of the state-scoped rotation check in motion_stall_detector_node.py (no ROS node, no sim).
Run: source /opt/ros/jazzy/setup.bash && python3 test_state_scope.py   -> exits 1 on any failing boundary case."""
import sys, importlib.util
from pathlib import Path
SRC = Path(__file__).resolve().parents[3] / 'src/ramp_autonomy_control/ramp_autonomy_control/motion_stall_detector_node.py'
spec = importlib.util.spec_from_file_location('m', SRC); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


class F(m.MotionStallDetector):
    def __init__(self):
        self._ramp_state = None; self._pre_halt_state = None


def applies(seq):
    f = F()
    for s in seq:
        x = type('M', (), {})(); x.data = s; f._on_ramp_state(x)
    return f._rotation_check_applies()


CASES = [([], False, 'unknown state -> original logic'), (['APPROACH'], True, 'APPROACH'), (['APPROACH', 'ASCENT'], True, 'ASCENT'),
         (['ASCENT', 'TRANSITION'], True, 'TRANSITION'), (['TRANSITION', 'EXIT'], True, 'EXIT'),
         (['EXIT', 'DESCENT'], False, 'DESCENT -> original logic'), (['EXIT', 'DESCENT', 'HALT'], False, 'HALT that interrupted DESCENT -> original'),
         (['TRANSITION', 'EXIT', 'HALT'], True, 'HALT that interrupted EXIT -> rotation check'),
         (['EXIT', 'DESCENT', 'HALT', 'DESCENT', 'EXIT'], True, 'back to EXIT after the descent'), (['HALT'], False, 'HALT with no prior state -> original'),
         (['DESCENT', 'HALT', 'HALT', 'EXIT', 'HALT'], True, 'HALT interrupting EXIT after an earlier DESCENT')]
bad = [(s, e, d) for s, e, d in CASES if applies(s) != e]
for s, e, d in CASES:
    print(('ok ' if applies(s) == e else 'BAD'), f'applies={applies(s)!s:5s} expected={e!s:5s} {d}')
print('ALL BOUNDARY CASES PASS' if not bad else f'{len(bad)} FAILED'); sys.exit(1 if bad else 0)
