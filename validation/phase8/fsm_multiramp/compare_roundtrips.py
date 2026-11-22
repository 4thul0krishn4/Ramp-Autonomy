#!/usr/bin/env python3
"""Compare Floor-1-only round trips: OLD-FSM runs (floor2/ final-world runs + run1_before_fence_fix) vs NEW-FSM runs (run3_regression): driver metrics + the /ramp_state sequence (from the CSVs' state column)."""
RAMP_WS = os.environ['RAMP_WS']
import csv, re, sys, os
B = os.path.join(RAMP_WS, 'validation/phase8')
def seq(csvp):
    out = []
    for r in csv.DictReader(open(csvp)):
        s = r['state']
        if s != 'HALT' and (not out or out[-1] != s): out.append(s)
    return out
def metrics(log):
    t = open(log).read(); g = lambda p: (re.search(p, t) or [None, None])[1]
    return dict(leg3=g(r'\((\d+)%, need'), halts=g(r'total HALT events: (\d+)'), margin=g(r'min stability_margin observed \(overall\): ([\d.]+)'), final=g(r'final gz pose: \(([^)]*)\)'),
                slip=g(r'max wheel_slip observed \(overall\): ([\d.]+)'), legs=re.findall(r"'leg': '([^']+)'.*?'ok': (True|False)", t), descent_lat=g(r'descent max \|ground-truth lateral Y\|: ([\d.]+)'))
for cfg in ('loaded', 'unloaded'):
    runs = [('OLD FSM (floor2 verification)', f'{B}/floor2/roundtrip_floor2world_{cfg}'), ('NEW FSM (this session)', f'{B}/fsm_multiramp/run3_regression/rt_{cfg}')]
    print(f'=== {cfg} ===')
    for name, base in runs:
        m = metrics(base + '.log'); s = seq(base + '.csv')
        print(f'  {name:32s} leg-3 verification {m["leg3"]}%  HALTs {m["halts"]}  min margin {m["margin"]}  max slip {m["slip"]}  descent max|GT y| {m["descent_lat"]} m  final GT ({m["final"]})')
        print(f'  {"":32s} legs ok: {[(a, b) for a, b in m["legs"]]}')
        print(f'  {"":32s} non-HALT /ramp_state sequence (1 Hz CSV): {" > ".join(s)}')
