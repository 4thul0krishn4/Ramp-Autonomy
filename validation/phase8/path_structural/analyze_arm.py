#!/usr/bin/env python3
"""Compare a structural-path arm (curved / normal / unloaded) against the three baselines with the project's statistics.
Usage: analyze_arm.py <label> <dir> <prefix> [<label2> <dir2> <prefix2> ...]
Failure = ground-truth max|y| > 1.5 m. Baselines (same instrument): original 0/6, fixed 10/10, scoped 5/14."""
import os
# Workspace root. Set RAMP_WS to the ramp_autonomy_ws checkout before running.
RAMP_WS = os.environ.get('RAMP_WS')
if not RAMP_WS:
    raise SystemExit('set RAMP_WS to the ramp_autonomy_ws workspace root')
import sys, os, re, math, glob
import numpy as np
from scipy import stats
sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8'))
import compare_detectors_curved_unloaded as C
import analyze_closing_samples as A
P = C.P8


def load(dirn, prefix):
    out = []
    files = sorted(glob.glob(f'{dirn}/{prefix}*_result.log'), key=lambda f: int(re.search(rf'{re.escape(prefix)}(\d+)_result', f).group(1)))
    for f in files:
        b = f[:-len('_result.log')]
        if not (os.path.exists(b + '_health.json') and 'overall_verdict:' in open(f).read()):
            continue
        r = C.read(f, b + '_launch.log', b + '_health.json')
        t = open(f).read()
        r['leg'] = float((re.search(r'descent_leg_distance_m: ([\d.]+)', t) or [0, 'nan'])[1])
        r['n'] = int((re.search(r'n=(\d+) samples', t) or [0, 0])[1])
        m = re.search(r'gt_final_pose: \(([-\d.]+), ([-\d.]+), ([-\d.]+)', t)
        r['fx'], r['fy'], r['fz'] = (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else (float('nan'),) * 3
        r['name'] = os.path.basename(b)
        out.append(r)
    return out


def summarize(name, G):
    y = [r['y'] for r in G]; k = sum(v > 1.5 for v in y); n = len(y)
    lo, hi = C.wilson(k, n)
    print(f"  {name:34s} n={n:2d} GT fail {k:2d}/{n:<2d} = {100*k/n:3.0f}% (Wilson {100*lo:.0f}-{100*hi:.0f}%)  max|y| mean {np.mean(y):.2f} sd {np.std(y, ddof=1) if n > 1 else 0:.2f} median {np.median(y):.2f} range {min(y):.2f}-{max(y):.2f}")


def compare(name, G, base):
    y = [r['y'] for r in G]; k, n = sum(v > 1.5 for v in y), len(y)
    for bn, B in base:
        by = [r['y'] for r in B]; kb, nb = sum(v > 1.5 for v in by), len(by)
        d = k / n - kb / nb; lo, hi = A.newcombe_diff(k, n, kb, nb)
        fp = C.fisher_two_sided(k, n - k, kb, nb - kb)
        ci = C.boot_ci(y, by); mw = stats.mannwhitneyu(y, by, alternative='two-sided').pvalue
        print(f"    vs {bn:14s}: failure-rate diff {100*d:+4.0f} pts (Newcombe 95% CI {100*lo:+.0f}..{100*hi:+.0f}); Fisher p={fp:.4f}; "
              f"mean max|y| diff {np.mean(y)-np.mean(by):+.2f} m (boot CI {ci[0]:+.2f}..{ci[1]:+.2f}); rank p={mw:.4f}")


def main():
    O = C.group_original()
    Fx = C.group_fixed() + A.group('curved_unloaded_fixedctl_logs', 'curved_unloaded_normal_fixctl_r', 4)
    S = A.group('curved_unloaded_scoped_logs', 'curved_unloaded_normal_scoped_r', 6) + A.group('curved_unloaded_scoped2_logs', 'curved_unloaded_normal_scoped2_r', 12)
    for R in (O, Fx, S):     # progress metrics for baselines too
        for r in R:
            r.setdefault('leg', float('nan'))
    base = [('original 0/6', O), ('fixed 10/10', Fx), ('scoped 5/14', S)]
    print('BASELINES'); [summarize(n, B) for n, B in base]
    args = sys.argv[1:]
    for i in range(0, len(args), 3):
        label, dirn, prefix = args[i:i + 3]
        G = load(dirn, prefix)
        if not G:
            print(f'{label}: no runs'); continue
        print(f'\n=== ARM: {label} ===')
        print(f"  {'run':40s} {'GT max|y|':>9s} {'verdict':>9s} {'HALTs':>5s} {'stall/slip':>10s} {'leg m':>6s} {'n':>3s} {'final gz (x, y, z)':>24s} {'min RTF':>7s}")
        for r in G:
            print(f"  {r['name']:40s} {r['y']:9.2f} {r['overall']:>9s} {r['halts']:5d} {str(r['stall'])+'/'+str(r['slip']):>10s} {r['leg']:6.2f} {r['n']:3d} ({r['fx']:6.2f},{r['fy']:6.2f},{r['fz']:5.2f}) {r['min_rtf']:7.3f}{'  <0.995' if r['min_rtf'] < 0.995 else ''}")
        summarize(label + ' (all)', G)
        strict = [r for r in G if r['min_rtf'] >= 0.995]
        if len(strict) != len(G) and strict:
            summarize(label + ' (RTF>=0.995)', strict)
        compare(label, G, base)
        h = [r['halts'] for r in G]
        print(f"  HALTs: median {np.median(h):.0f} mean {np.mean(h):.1f} range {min(h)}-{max(h)}; HALT_THRASH(>=8) {sum(x >= 8 for x in h)}/{len(h)}")
        print(f"  descent progress: odom leg distance median {np.median([r['leg'] for r in G]):.2f} m (of a 6 m path); runs with leg >= 2.5 m (E2): {sum(r['leg'] >= 2.5 for r in G)}/{len(G)}")


if __name__ == '__main__':
    main()
