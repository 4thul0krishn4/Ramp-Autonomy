#!/usr/bin/env python3
"""Completion analysis of the post-geometry-fix reverification samples. Completion = ascent PASS (ground-truth verified) AND the descent reached the ramp foot
(final ground-truth x <= 5.0 m and z <= 0.5 m) AND max |y| <= 1.5 m. Usage: analyze_reverify.py [--runs]"""
RAMP_WS = os.environ['RAMP_WS']
import sys, os, re, glob, json
import numpy as np
sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8'))
import compare_detectors_curved_unloaded as C
H = os.path.join(RAMP_WS, 'validation/phase8/crest_freeze')
CELLS = [('curved / 0.45 / loaded', 'curved_loaded_mu045', 'curved_loaded_mu045_rv_r'), ('curved / normal / loaded', 'curved_loaded_normal', 'curved_loaded_normal_rv_r'),
         ('sharp / normal / loaded', 'sharp_loaded_normal', 'sharp_loaded_normal_rv_r'), ('sharp / 0.45 / loaded', 'sharp_loaded_mu045', 'sharp_loaded_mu045_rv_r'),
         ('curved / normal / unloaded (regression)', 'curved_unloaded_normal', 'curved_unloaded_normal_rv_r'), ('sharp / normal / unloaded (regression)', 'sharp_unloaded_normal', 'sharp_unloaded_normal_rv_r')]


def parse(base):
    t = open(base + '_result.log').read()
    g = lambda k: (re.search(rf'^{k}: (.*)$', t, re.M) or [None, ''])[1]
    h = json.load(open(base + '_health.json'))
    m = re.search(r'gt_final_pose: \(([-\d.]+), ([-\d.]+), ([-\d.]+)', t)
    fx, fy, fz = (float(m.group(i)) for i in (1, 2, 3)) if m else (float('nan'),) * 3
    y = re.search(r'max \|y\|=([\d.]+)', g('ground_truth_verdict'))
    asc = g('ascent_verdict')
    r = dict(name=os.path.basename(base), asc=asc.split(' (')[0], asc_full=asc, y=float(y.group(1)) if y else float('nan'), halts=int(g('halt_count') or -1), overall=g('overall_verdict'),
             fx=fx, fy=fy, fz=fz, rtf=h['min_rtf'], starved=h['starved'], dis=float(g('ascent_gt_vs_odom_disagreement_m') or 'nan'),
             drive=float(g('platform_drive_advanced_m') or 'nan'), leg=float(g('descent_leg_distance_m') or 'nan'))
    r['reached_foot'] = (r['fx'] <= 5.0 and r['fz'] <= 0.5)
    r['complete_prereg'] = r['asc'] == 'PASS' and r['reached_foot'] and (r['y'] <= 1.5)     # as pre-registered (calibrated on the pre-fix leg end, x 4.2-4.5)
    tr = re.search(r'gt_trace_json: (.*)', t)
    tr = json.loads(tr.group(1)) if tr else []
    r['travel'] = (tr[0][1] - tr[-1][1]) if len(tr) > 1 else float('nan')                    # ground-truth descent travel along x (m)
    # AMENDED criterion (see README): descent travel >= 4.5 m - the leg end moved from x 4.2-4.5 to 5.0-5.2 after the ramp was raised 0.05 m
    r['complete'] = r['asc'] == 'PASS' and (r['travel'] >= 4.5) and (r['y'] <= 1.5)
    return r


def load(dirn, prefix):
    out = []
    for f in sorted(glob.glob(f'{H}/reverify_{dirn}_logs/{prefix}*_result.log'), key=lambda x: int(re.search(rf'{re.escape(prefix)}(\d+)_result', x).group(1))):
        b = f[:-len('_result.log')]
        if os.path.exists(b + '_health.json') and 'overall_verdict:' in open(f).read():
            out.append(parse(b))
    return out


def main():
    for title, d, pf in CELLS:
        G = load(d, pf)
        if not G: print(f'\n=== {title}: no runs yet'); continue
        n = len(G); k = sum(r['complete'] for r in G); lo, hi = C.wilson(k, n); kp = sum(r['complete_prereg'] for r in G)
        strict = [r for r in G if r['rtf'] >= 0.995]
        ks = sum(r['complete'] for r in strict)
        asc = {}
        for r in G: asc[r['asc']] = asc.get(r['asc'], 0) + 1
        print(f"\n=== {title}: COMPLETED {k}/{n} = {100*k/n:.0f}% (Wilson 95% {100*lo:.0f}-{100*hi:.0f}%) [pre-registered foot criterion x<=5.0 & z<=0.5: {kp}/{n}];  strict RTF>=0.995: {ks}/{len(strict)};  ascent verdicts {asc}")
        h = [r['halts'] for r in G]; ys = [r['y'] for r in G if r['asc'] == 'PASS' and r['y'] == r['y']]
        print(f"    HALTs median {np.median(h):.0f} range {min(h)}-{max(h)}, thrash(>=8) {sum(x >= 8 for x in h)}/{n};  max|y| over completed-ascent runs {max(ys) if ys else float('nan'):.3f};  "
              f"min RTF {min(r['rtf'] for r in G):.3f}; starved {sum(r['starved'] for r in G)};  ascent GT-vs-odom disagreement {np.nanmin([r['dis'] for r in G]):+.2f}..{np.nanmax([r['dis'] for r in G]):+.2f} m")
        if '--runs' in sys.argv:
            for r in G: print(f"      {r['name']:38s} {r['asc']:34s} drive {r['drive']:5.2f} m  descent travel {r['travel']:.2f} m final gz ({r['fx']:.2f},{r['fy']:+.2f},{r['fz']:.2f}) max|y| {r['y']:.3f} HALTs {r['halts']:2d} RTF {r['rtf']:.3f} {'OK' if r['complete'] else '<-- NOT COMPLETE'}")


if __name__ == '__main__':
    main()
