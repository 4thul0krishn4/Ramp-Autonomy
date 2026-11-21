#!/usr/bin/env python3
"""Closing analysis of the stall-detector / curved-world investigation.
PART 1  curved / normal / unloaded: scoped (6 earlier + N new) vs original (n=6) and vs fixed (n=10).
        Failure = ground-truth max|y| beyond the 1.5 m panel half-width (the matrix's own verdict).
        Reported: Wilson 95% intervals, exact Fisher, the Newcombe (hybrid score) 95% interval for the DIFFERENCE in failure
        proportion, and for the departure magnitude an exact Mann-Whitney test and bootstrap CI for the mean difference.
PART 2  the two curved cells whose overall verdict moved MARGINAL -> FAIL (HALT_THRASH) on the scoped detector.
        HALT_THRASH is the matrix's own rule (>= 8 HALTs); per cell, every run's HALT count and verdict is listed."""
import re, os, glob, json, math, random
import numpy as np
from scipy import stats
import compare_detectors_curved_unloaded as C
P = C.P8


def wilson(k, n, z=1.96):
    return C.wilson(k, n, z)


def newcombe_diff(k1, n1, k2, n2, z=1.96):
    """Newcombe hybrid-score 95% CI for p1 - p2"""
    l1, u1 = wilson(k1, n1, z); l2, u2 = wilson(k2, n2, z); p1, p2 = k1 / n1, k2 / n2
    return (p1 - p2) - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2), (p1 - p2) + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2)


def rd(base):
    return C.read(base + '_result.log', base + '_launch.log', base + '_health.json')


def group(dirn, prefix, n):
    out = []
    for i in range(1, n + 1):
        b = f'{P}/{dirn}/{prefix}{i}'
        if os.path.exists(b + '_health.json') and 'overall_verdict:' in open(b + '_result.log').read():
            out.append(rd(b))
    return out


def part1():
    O = C.group_original()
    ctl = group('curved_unloaded_fixedctl_logs', 'curved_unloaded_normal_fixctl_r', 4)
    Fx = C.group_fixed() + ctl
    S1 = group('curved_unloaded_scoped_logs', 'curved_unloaded_normal_scoped_r', 6)
    S2 = group('curved_unloaded_scoped2_logs', 'curved_unloaded_normal_scoped2_r', 12)
    S = S1 + S2
    C.table(f"SCOPED, new batch (n={len(S2)})", S2)
    fy = lambda G: [r['y'] for r in G]
    def report(name, G):
        y = fy(G); k = sum(v > 1.5 for v in y); lo, hi = wilson(k, len(y))
        print(f"  {name:34s} n={len(y):2d}  GT fail {k:2d}/{len(y):<2d} = {100*k/len(y):3.0f}%  (Wilson 95% {100*lo:.0f}-{100*hi:.0f}%)   max|y| mean {np.mean(y):.2f} sd {np.std(y, ddof=1):.2f} median {np.median(y):.2f} range {min(y):.2f}-{max(y):.2f}")
    print("\n======== PART 1: curved / normal / unloaded ========")
    for nm, G in (('ORIGINAL detector', O), ('FIXED detector (rotation check everywhere)', Fx), ('SCOPED, first batch', S1), ('SCOPED, new batch', S2), ('SCOPED, ALL', S)):
        report(nm, G)
    strict = [r for r in S if r['min_rtf'] >= 0.995]
    report('SCOPED, ALL, runs with RTF >= 0.995', strict)
    print(f"  (scoped runs under the 0.995 RTF bar: {len(S) - len(strict)}; min RTF over scoped runs {min(r['min_rtf'] for r in S):.3f}; any starved: {any(r['starved'] for r in S)})")
    for label, T in (('ALL scoped runs', S), ('scoped runs with RTF >= 0.995', strict)):
        ks, ns = sum(v > 1.5 for v in fy(T)), len(T)
        print(f"\n  --- {label} (n={ns}) ---")
        for oname, G in (('ORIGINAL (0/6)', O), ('FIXED (10/10)', Fx)):
            ko, no = sum(v > 1.5 for v in fy(G)), len(G)
            d = (ks / ns) - (ko / no); lo, hi = newcombe_diff(ks, ns, ko, no)
            fp = C.fisher_two_sided(ks, ns - ks, ko, no - ko)
            mw = stats.mannwhitneyu(fy(T), fy(G), alternative='two-sided').pvalue
            ci = C.boot_ci(fy(T), fy(G))
            print(f"    vs {oname:16s}: failure-rate difference {100*d:+.0f} points, Newcombe 95% CI [{100*lo:+.0f}, {100*hi:+.0f}];  Fisher p = {fp:.4f};  "
                  f"max|y| mean diff {np.mean(fy(T)) - np.mean(fy(G)):+.2f} m, bootstrap 95% CI [{ci[0]:+.2f}, {ci[1]:+.2f}], Mann-Whitney p = {mw:.4f}")
    # in-DESCENT latch association across all three detectors
    def anchor_latch(base):
        L = []
        for l in open(base + '_launch.log', errors='ignore'):
            m = re.search(r'\[([\w]+)-\d+\] \[(\w+)\] \[(\d+\.\d+)\] \[[^\]]*\]: (.*)', l)
            if m: L.append((m.group(1), float(m.group(3)), m.group(4)))
        td = next((t for n, t, m in L if 'nav2_param_switcher' in n and 'descent_mode' in m and 'applied' in m), None)
        if td is None: return None
        return any(n == 'state_machine_node' and 'Motion stall confirmed' in m and '-> HALT' in m and td <= t <= td + 6 for n, t, m in L)
    bases = ([(f'{P}/curved_unloaded_origdet_logs/curved_unloaded_normal_orig_r{i}') for i in range(1, 7)] +
             [(f'{P}/curved_unloaded_scoped_logs/curved_unloaded_normal_scoped_r{i}') for i in range(1, 7)] +
             [(f'{P}/curved_unloaded_scoped2_logs/curved_unloaded_normal_scoped2_r{i}') for i in range(1, 13)] +
             [f'{P}/matrix_regression_newdetector_logs/curved_unloaded_normal'] + [f'{P}/curved_unloaded_repeat_logs/curved_unloaded_normal_r{i}' for i in range(1, 6)] +
             [f'{P}/curved_unloaded_fixedctl_logs/curved_unloaded_normal_fixctl_r{i}' for i in range(1, 5)])
    rows = []
    for b in bases:
        if os.path.exists(b + '_health.json') and os.path.exists(b + '_launch.log'):
            a = anchor_latch(b)
            if a is not None:
                y = float(re.search(r'gt_max_abs_y_descent_m: ([\d.]+)', open(b + '_result.log').read()).group(1))
                rows.append((a, y > 1.5))
    wl = [r for r in rows if r[0]]; wo = [r for r in rows if not r[0]]
    print(f"\n  in-DESCENT stall latch vs outcome across original + scoped + fixed ({len(rows)} runs): "
          f"with latch {len(wl)} runs, {sum(not r[1] for r in wl)} pass / {sum(r[1] for r in wl)} fail;  without {len(wo)} runs, {sum(not r[1] for r in wo)} pass / {sum(r[1] for r in wo)} fail;  "
          f"Fisher p = {C.fisher_two_sided(sum(r[1] for r in wl), sum(not r[1] for r in wl), sum(r[1] for r in wo), sum(not r[1] for r in wo)):.2g}")
    S_all = S
    lat = []
    for i in range(1, 7): lat.append(anchor_latch(f'{P}/curved_unloaded_scoped_logs/curved_unloaded_normal_scoped_r{i}'))
    for i in range(1, 13):
        b = f'{P}/curved_unloaded_scoped2_logs/curved_unloaded_normal_scoped2_r{i}'
        if os.path.exists(b + '_launch.log'): lat.append(anchor_latch(b))
    print(f"  scoped runs that had the in-DESCENT latch: {sum(1 for x in lat if x)}/{len([x for x in lat if x is not None])}")


def cell(name, scoped_dirs, earlier):
    print(f"\n  --- {name} ---")
    rows = []
    for tag, base in earlier:
        if os.path.exists(base + '_result.log'):
            r = rd(base); rows.append((tag, r))
    for tag, base in scoped_dirs:
        if os.path.exists(base + '_health.json') and 'overall_verdict:' in open(base + '_result.log').read():
            rows.append((tag, rd(base)))
    print(f"    {'run':30s} {'HALTs':>6s} {'overall':>9s} {'GT max|y|':>10s} {'stall/slip':>11s} {'min RTF':>8s}")
    for tag, r in rows:
        print(f"    {tag:30s} {r['halts']:6d} {r['overall']:>9s} {r['y']:10.2f} {str(r['stall'])+'/'+str(r['slip']):>11s} {r['min_rtf']:8.3f}")
    sc = [r for t, r in rows if t.startswith('SCOPED')]; other = [r for t, r in rows if not t.startswith('SCOPED')]
    def s(G):
        if not G: return 'n/a'
        h = [r['halts'] for r in G]; f = sum(r['halts'] >= 8 for r in G)
        return f"n={len(G)} HALTs {sorted(h)} mean {np.mean(h):.1f}, HALT_THRASH (>=8) in {f}/{len(G)} (Wilson {tuple(round(100*x) for x in wilson(f, len(G)))}%), overall FAIL {sum(r['overall']=='FAIL' for r in G)}/{len(G)}, GT fails {sum(r['y']>1.5 for r in G)}/{len(G)}"
    print(f"    scoped runs   : {s(sc)}")
    print(f"    earlier runs  : {s(other)}   (old detector / fixed detector / prior matrix; single runs)")
    if sc and other:
        a = [r['halts'] for r in sc]; b = [r['halts'] for r in other]
        print(f"    scoped HALTs mean {np.mean(a):.1f} vs earlier {np.mean(b):.1f};  Mann-Whitney p = {stats.mannwhitneyu(a, b, alternative='two-sided').pvalue:.3f} (n={len(a)} vs {len(b)}: very low power)")


def part2():
    print("\n======== PART 2: the two curved cells whose HALT verdict worsened ========")
    M = f'{P}/matrix_regression_scoped_logs'; Old = f'{P}/matrix_regression_cleanhost_logs'; Fx = f'{P}/matrix_regression_newdetector_logs'
    cell('curved / 0.45 / unloaded (scoped matrix run: 13 HALTs)',
         [('SCOPED matrix run', f'{M}/curved_unloaded_mu045')] + [(f'SCOPED repeat {i}', f'{P}/cell_curved_mu045_unloaded_scoped_logs/curved_unloaded_mu045_scoped_r{i}') for i in range(1, 5)] +
         [(f'SCOPED repeat {i+4}', f'{P}/cell_curved_mu045_unloaded_scoped_logs/curved_unloaded_mu045_scoped_b_r{i}') for i in range(1, 5)],
         [('old detector (matrix run)', f'{Old}/curved_unloaded_mu045'), ('fixed detector (matrix run)', f'{Fx}/curved_unloaded_mu045')])
    cell('curved / 0.45 / loaded (scoped matrix run: 8 HALTs)',
         [('SCOPED matrix run', f'{M}/curved_loaded_mu045')] + [(f'SCOPED repeat {i}', f'{P}/cell_curved_mu045_loaded_scoped_logs/curved_loaded_mu045_scoped_r{i}') for i in range(1, 5)],
         [('old detector (matrix run)', f'{Old}/curved_loaded_mu045'), ('fixed detector (matrix run)', f'{Fx}/curved_loaded_mu045')])
    print("    (the very first matrix, before any of this work, is recorded in the README only: unloaded 1 HALT / 0.36 m MARGINAL; loaded 9 HALTs / 0.93 m FAIL HALT_THRASH)")


if __name__ == '__main__':
    part1(); part2()
