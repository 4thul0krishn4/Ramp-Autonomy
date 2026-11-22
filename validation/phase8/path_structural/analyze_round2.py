#!/usr/bin/env python3
"""Round 2: corrected turn procedure, curved / normal / unloaded. Pooled Arm P (scoped) and Arm P-fixed (unscoped) samples.
A run FAILS if E1 (max|y| > 1.5 m) or E2' (did not reach the ramp foot: final gz x > 5.0 m or z > 0.5 m)."""
RAMP_WS = os.environ['RAMP_WS']
import sys, os, glob, re
import numpy as np
sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8')); sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8/path_structural'))
import compare_detectors_curved_unloaded as C
import analyze_arm as AA
P = C.P8 + '/path_structural'


def runs(dirn, prefix):
    G = AA.load(f'{P}/{dirn}', prefix)
    for r in G:
        r['e1'] = r['y'] <= 1.5
        r['e2'] = (r['fx'] <= 5.0) and (r['fz'] <= 0.5)
        r['ok'] = r['e1'] and r['e2']
    return G


def report(name, G):
    n = len(G); k = sum(not r['ok'] for r in G); lo, hi = C.wilson(k, n)
    e1 = sum(not r['e1'] for r in G); e2 = sum(not r['e2'] for r in G)
    y = [r['y'] for r in G]; h = [r['halts'] for r in G]
    strict = [r for r in G if r['min_rtf'] >= 0.995]
    ks = sum(not r['ok'] for r in strict); los, his = C.wilson(ks, len(strict)) if strict else (float('nan'),) * 2
    print(f"  {name:34s} n={n:2d}  FAIL {k}/{n} (Wilson 95% {100*lo:.0f}-{100*hi:.0f}%)  [E1 fails {e1}, E2' fails {e2}]  max|y| mean {np.mean(y):.3f} max {max(y):.3f} | "
          f"HALTs median {np.median(h):.0f} range {min(h)}-{max(h)}, thrash(>=8) {sum(x >= 8 for x in h)}/{n} | min RTF {min(r['min_rtf'] for r in G):.3f}, "
          f"strict RTF>=0.995: {ks}/{len(strict)} fail")
    return k, n


def main():
    Ps = [('Arm P (scoped), earlier 6', 'armP_logs', 'curved_unloaded_normal_armP_r'), ('Arm P (scoped), new 12', 'armP2_logs', 'curved_unloaded_normal_armP2_r')]
    Pf = [('Arm P-fixed (unscoped), earlier 6', 'armPfixed_logs', 'curved_unloaded_normal_armPfixed_r'), ('Arm P-fixed (unscoped), new 12', 'armPfixed2_logs', 'curved_unloaded_normal_armPfixed2_r')]
    out = {}
    for label, arms in (('SCOPED', Ps), ('UNSCOPED FIXED', Pf)):
        print(f"\n=== {label} detector ===")
        allr = []
        for nm, d, pf in arms:
            G = runs(d, pf)
            if not G: print(f'  {nm}: no runs'); continue
            report(nm, G); allr += G
            if '--runs' in sys.argv:
                for r in G: print(f"      {r['name']:42s} max|y| {r['y']:.3f} final gz ({r['fx']:.2f},{r['fy']:+.2f},{r['fz']:.2f}) HALTs {r['halts']:2d} RTF {r['min_rtf']:.3f} {'' if r['ok'] else '  <-- FAIL'}")
        if allr:
            print('  --- pooled ---'); out[label] = report(f'{label} pooled', allr)
    if len(out) == 2:
        (ka, na), (kb, nb) = out['SCOPED'], out['UNSCOPED FIXED']
        print(f"\nscoped {ka}/{na} vs unscoped {kb}/{nb}: exact Fisher p = {C.fisher_two_sided(ka, na-ka, kb, nb-kb):.3f}")


if __name__ == '__main__':
    main()
