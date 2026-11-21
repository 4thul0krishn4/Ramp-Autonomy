#!/usr/bin/env python3
"""Causal-test analysis, curved / normal / unloaded. Groups (ground-truth max |y|, panel half-width 1.5 m):
  original (n=6) | fixed (n=10) | HOLD pause (n=6: pilot + 5) | CEILING, no pause (n=5) | RESET only, no pause (n=5)
Per run it also records what accompanied the crest: a HALT within [t_desc-1, t_desc+3] s, descent_brake's cautious
post-HALT restart, and the number of MPPI optimizer-reset bursts - because ANY stop-and-go on this platform trips a
wheel-slip HALT at resume, which drags those in.
Interpretation rules (fixed in advance; an arm 'reproduces' if its max|y| differs from FIXED at p<0.05 AND its mean lies
within 0.5 m of ORIGINAL's, 'fails to' if it does not differ from FIXED at p<0.05):
  HOLD reproduces, CEILING and RESET do not -> a pause (with its HALT side effects) is the operative element
  CEILING reproduces, RESET does not         -> the reduced acceleration limits, not the pause, are sufficient
  RESET reproduces                           -> the optimizer reset alone is sufficient
  none reproduces                            -> the effect is something the revert does that these do not: NOT understood."""
import re, json, glob, os, numpy as np
import compare_detectors_curved_unloaded as C
P = C.P8
LD = f'{P}/curved_unloaded_pause_logs'


def wall_lines(launch):
    out = []
    for l in open(launch, errors='ignore'):
        m = re.search(r'\[([\w]+)-\d+\] \[(\w+)\] \[(\d+\.\d+)\] \[[^\]]*\]: (.*)', l)
        if m:
            out.append((m.group(1), float(m.group(3)), m.group(4)))
    return out


def run_info(base):
    r = C.read(base + '_result.log', base + '_launch.log', base + '_health.json')
    pj = base + '_pause.json'
    j = json.load(open(pj)) if os.path.exists(pj) else {}
    td = j.get('t_desc')
    lines = wall_lines(base + '_launch.log')
    d = dict(r)
    d['fired'] = bool(j.get('fired')); d['pushes'] = j.get('pushes', []); d['hold'] = {k: j.get(k) for k in ('speed_at_start', 'min_speed_in_hold', 'max_actuation_after_rampdown', 'states_during')}
    if td:
        d['crest_halt'] = any(n == 'state_machine_node' and 'HALT' in m and td - 1 <= t <= td + 3 for n, t, m in lines if '-> HALT' in m)
        d['cautious'] = any('Re-entering DESCENT after a HALT' in m and td <= t <= td + 6 for n, t, m in lines)
        rs = sorted(t for n, t, m in lines if n == 'controller_server' and 'Optimizer reset' in m and td - 0.5 <= t <= td + 6)
        bursts = 0; last = -9
        for t in rs:
            if t - last > 0.2: bursts += 1
            last = t
        d['reset_bursts'] = bursts
    return d


def collect(prefix, n, first=1):
    out = []
    for i in range(first, first + n):
        b = f'{LD}/{prefix}{i}'
        if os.path.exists(b + '_health.json') and 'overall_verdict:' in open(b + '_result.log').read():
            out.append(run_info(b))
    return out


def show(name, rows):
    print(f"\n{name}  (n={len(rows)})")
    print(f"  {'GT max|y|':>9s} {'verdict':>9s} {'HALTs':>6s} {'crest HALT':>10s} {'cautious restart':>17s} {'reset bursts':>13s} {'minRTF':>7s}  extra")
    for r in rows:
        ex = ''
        if r['hold'].get('min_speed_in_hold') is not None:
            ex = f"hold: speed at start {r['hold']['speed_at_start']:.3f}, min in hold {r['hold']['min_speed_in_hold']:.3f}, states {r['hold']['states_during']}"
        elif r['pushes']:
            ex = f"{len(r['pushes'])} pushes ({sum(1 for p in r['pushes'] if p[2])} ok)"
        print(f"  {r['y']:9.2f} {r['gt'].split(' ')[0]:>9s} {r['halts']:6d} {str(r.get('crest_halt')):>10s} {str(r.get('cautious')):>17s} {str(r.get('reset_bursts')):>13s} {r['min_rtf']:7.3f}  {ex}")


def main():
    O = C.group_original(); F = C.group_fixed()
    ctl = [C.read(f'{P}/curved_unloaded_fixedctl_logs/curved_unloaded_normal_fixctl_r{i}_result.log', f'{P}/curved_unloaded_fixedctl_logs/curved_unloaded_normal_fixctl_r{i}_launch.log', f'{P}/curved_unloaded_fixedctl_logs/curved_unloaded_normal_fixctl_r{i}_health.json') for i in range(1, 5)]
    Fall = F + ctl
    H = collect('curved_unloaded_normal_pause_a_r', 1) + collect('curved_unloaded_normal_pause_b_r', 5)
    Cc = collect('curved_unloaded_normal_ceil_r', 5); R = collect('curved_unloaded_normal_reset_r', 5)
    for n, rows in (('HOLD (pause via test hook)', H), ('CEILING (cautious limits, no pause)', Cc), ('RESET (param re-push, no pause)', R)):
        show(n, rows)
    fy = [r['y'] for r in Fall]; oy = [r['y'] for r in O]
    print("\n================ arms vs fixed (baseline, n=%d, mean %.2f) and vs original (n=%d, mean %.2f) ================" % (len(fy), np.mean(fy), len(oy), np.mean(oy)))
    print(f"  {'arm':28s} {'n':>2s} {'GT fails':>9s} {'mean max|y|':>12s} {'range':>11s} {'p vs fixed':>11s} {'p vs original':>14s}")
    for name, rows in (('original detector', None), ('HOLD pause', H), ('CEILING no pause', Cc), ('RESET no pause', R), ('fixed detector', None)):
        y = oy if name == 'original detector' else fy if name == 'fixed detector' else [r['y'] for r in rows]
        if not y: continue
        f = sum(v > 1.5 for v in y)
        pf = C.fisher_two_sided(f, len(y) - f, sum(v > 1.5 for v in fy), len(fy) - sum(v > 1.5 for v in fy)) if name != 'fixed detector' else float('nan')
        pm = C.perm_mean_p(y, fy) if name not in ('fixed detector',) and len(y) + len(fy) <= 22 else float('nan')
        po = C.perm_mean_p(y, oy) if name not in ('original detector',) and len(y) + len(oy) <= 22 else float('nan')
        print(f"  {name:28s} {len(y):2d} {f:>4d}/{len(y):<4d} {np.mean(y):12.2f} {min(y):5.2f}-{max(y):4.2f} {pm:11.4f} {po:14.4f}")
    print("  (p vs fixed / original = exact permutation test on the departure magnitude)")


if __name__ == '__main__':
    main()
