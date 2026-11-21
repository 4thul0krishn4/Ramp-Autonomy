#!/usr/bin/env python3
"""Controlled comparison for the curved / normal / unloaded matrix cell: ORIGINAL vs FIXED stall detector.

Groups (identical instrument, guarded clean host, one instance, RTF monitored):
  fixed    : matrix_regression_newdetector_logs/curved_unloaded_normal  +  curved_unloaded_repeat_logs/r1..r5   (n=6)
  original : curved_unloaded_origdet_logs/curved_unloaded_normal_orig_r1..                                       (n=N)
Supplementary (NOT in the controlled comparison - different conditions): earlier old-detector runs
  2.68 m (earlier matrix, pre-guard, host never verified clean) and 1.01 m (clean-host regression, old detector).

Decision rule fixed BEFORE the original-detector runs: the detector is 'implicated' if failure counts differ
(exact Fisher, two-sided) or departure magnitudes differ (exact permutation test on the mean) at p < 0.05;
otherwise 'not distinguishable'. Power is reported honestly: with 6 vs 6, only a difference such as 6/6 vs <=1/6
failing is detectable. Ground-truth fail = max |y| beyond the 1.5 m panel half-width (the matrix's own verdict).
"""
import os
# Workspace root. Set RAMP_WS to the ramp_autonomy_ws checkout before running.
RAMP_WS = os.environ.get('RAMP_WS')
if not RAMP_WS:
    raise SystemExit('set RAMP_WS to the ramp_autonomy_ws workspace root')
import sys, re, glob, json, math, itertools, random
import numpy as np

P8 = os.path.join(RAMP_WS, 'validation/phase8')


def read(result_log, launch_log, health):
    t = open(result_log).read()
    g = lambda k: (re.search(rf'^{k}: (.*)$', t, re.M) or [None, ''])[1]
    y = float(g('gt_max_abs_y_descent_m') or 'nan')
    lt = open(launch_log, errors='ignore').read()
    h = json.load(open(health))
    return dict(y=y, gt=g('ground_truth_verdict')[:40], overall=g('overall_verdict'), halts=int(g('halt_count') or -1),
                stall=len(re.findall(r'Motion stall confirmed.*-> HALT', lt)), slip=len(re.findall(r'Wheel slip.*-> HALT', lt)),
                margin=len(re.findall(r'Stability margin.*-> HALT', lt)), min_rtf=h['min_rtf'], starved=h['starved'])


def group_fixed():
    d = [read(f'{P8}/matrix_regression_newdetector_logs/curved_unloaded_normal_result.log',
              f'{P8}/matrix_regression_newdetector_logs/curved_unloaded_normal_launch.log',
              f'{P8}/matrix_regression_newdetector_logs/curved_unloaded_normal_health.json')]
    for i in range(1, 6):
        b = f'{P8}/curved_unloaded_repeat_logs/curved_unloaded_normal_r{i}'
        d.append(read(b + '_result.log', b + '_launch.log', b + '_health.json'))
    return d


def group_original():
    d = []
    for f in sorted(glob.glob(f'{P8}/curved_unloaded_origdet_logs/curved_unloaded_normal_orig_r*_result.log'),
                    key=lambda x: int(re.search(r'_r(\d+)_result', x).group(1))):
        b = f[:-len('_result.log')]
        import os
        if not os.path.exists(b + '_health.json') or 'overall_verdict:' not in open(f).read():
            continue                      # run still in progress
        d.append(read(f, b + '_launch.log', b + '_health.json'))
    return d


def fisher_two_sided(a, b, c, d):
    """[[a,b],[c,d]] exact two-sided p"""
    n1, n2, k = a + b, c + d, a + c
    def pr(x):
        return math.comb(n1, x) * math.comb(n2, k - x) / math.comb(n1 + n2, k)
    p0 = pr(a)
    return sum(pr(x) for x in range(max(0, k - n2), min(n1, k) + 1) if pr(x) <= p0 * (1 + 1e-9))


def perm_mean_p(x, y):
    allv = list(x) + list(y); n = len(x)
    obs = abs(np.mean(x) - np.mean(y)); cnt = tot = 0
    for idx in itertools.combinations(range(len(allv)), n):
        s = set(idx)
        a = [allv[i] for i in s]; b = [allv[i] for i in range(len(allv)) if i not in s]
        tot += 1; cnt += abs(np.mean(a) - np.mean(b)) >= obs - 1e-12
    return cnt / tot


def boot_ci(x, y, n=20000, seed=1):
    rng = random.Random(seed)
    d = [np.mean([rng.choice(x) for _ in x]) - np.mean([rng.choice(y) for _ in y]) for _ in range(n)]
    return np.percentile(d, [2.5, 97.5])


def wilson(k, n, z=1.96):
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0, c - h), min(1, c + h)


def table(name, rows):
    print(f"\n{name}")
    print(f"  {'#':>2s} {'GT max|y| m':>12s} {'GT verdict':>42s} {'overall':>8s} {'HALTs':>6s} {'stall/slip/margin':>18s} {'min RTF':>8s}")
    for i, r in enumerate(rows, 1):
        print(f"  {i:2d} {r['y']:12.2f} {r['gt']:>42s} {r['overall']:>8s} {r['halts']:6d} {str(r['stall'])+'/'+str(r['slip'])+'/'+str(r['margin']):>18s} {r['min_rtf']:8.3f}"
              f"{'  STARVED' if r['starved'] else ''}{'  (RTF < 0.995)' if r['min_rtf'] < 0.995 else ''}")


def main():
    F, O = group_fixed(), group_original()
    table("FIXED detector (n=%d)" % len(F), F)
    table("ORIGINAL detector, controlled runs (n=%d)" % len(O), O)
    if not O:
        return
    fy, oy = [r['y'] for r in F], [r['y'] for r in O]
    ff, of = sum(y > 1.5 for y in fy), sum(y > 1.5 for y in oy)
    nf, no = len(F), len(O)
    print("\n================ comparison ================")
    for nm, k, n in (('fixed', ff, nf), ('original', of, no)):
        lo, hi = wilson(k, n)
        print(f"  {nm:9s}: ground-truth FAIL {k}/{n}  (Wilson 95% {100*lo:.0f}-{100*hi:.0f}%)")
    print(f"  exact Fisher, failure counts, two-sided: p = {fisher_two_sided(ff, nf-ff, of, no-of):.3f}")
    print(f"  departure magnitude (max |y|): fixed mean {np.mean(fy):.2f} sd {np.std(fy, ddof=1):.2f} range {min(fy):.2f}-{max(fy):.2f} | "
          f"original mean {np.mean(oy):.2f} sd {np.std(oy, ddof=1):.2f} range {min(oy):.2f}-{max(oy):.2f}")
    ci = boot_ci(oy, fy)
    print(f"  mean difference (original - fixed) = {np.mean(oy)-np.mean(fy):+.2f} m, bootstrap 95% CI [{ci[0]:+.2f}, {ci[1]:+.2f}]; "
          f"exact permutation p = {perm_mean_p(oy, fy):.3f}")
    for k, nm in (('halts', 'total HALTs'), ('stall', 'stall-latch HALT entries'), ('slip', 'slip HALT entries')):
        a, b = [r[k] for r in O], [r[k] for r in F]
        print(f"  {nm:26s}: original mean {np.mean(a):5.1f} (range {min(a)}-{max(a)}) | fixed mean {np.mean(b):5.1f} (range {min(b)}-{max(b)}) | perm p = {perm_mean_p(a, b):.3f}")
    print("  supplementary, not in the test: earlier old-detector runs 2.68 m (pre-guard host) and 1.01 m (clean)")
    allO = oy + [2.68, 1.01]
    print(f"  pooled old-detector (controlled + earlier two): fail {sum(y>1.5 for y in allO)}/{len(allO)}, mean {np.mean(allO):.2f} m")
    # power: which difference could 6 vs N have detected?
    print(f"  power note: with n={nf} vs n={no}, at p<0.05 only differences on the order of "
          f"{nf}/{nf} vs <= {max(0, int(round(no*0.17)))}/{no} failing (or a several-tenths-of-a-metre mean shift) are detectable")


if __name__ == '__main__':
    main()
