#!/usr/bin/env python3
"""Re-run motion_stall_detector_node's LiDAR scan-consistency algorithm offline on the
scans recorded by stall_scan_trial.py and compare its verdict with what the robot
physically did. Usage: analyze_scan.py <scan.npz> <trial_row.csv>

Detector algorithm being replicated (motion_stall_detector_node.py):
  * reference scan = the NEWEST scan that is >= STALL_WINDOW_S (5 s) old; current = newest scan
  * diff = mean |r_cur[i] - r_ref[i]| over beams finite in BOTH; count = number of such beams
  * use /scan_360 if its count >= 30 else /scan if its count >= 30, else no verdict
  * 'no movement' if diff < SCAN_DIFF_STALL_THRESHOLD_M (0.05 m)
  * a stall latches when the actuation command has been continuously nonzero for >= 5 s and
    the scan shows no movement (then holds >= 10 s)
"""
import sys, csv, math, ast
if len(sys.argv) < 3:
    raise SystemExit('usage: analyze_scan.py <scan.npz> <trial_row.csv>')
import numpy as np

NPZ, ROWCSV = sys.argv[1], sys.argv[2]
WIN, THR, MINB = 5.0, 0.05, 30
z = np.load(NPZ, allow_pickle=True)
row = list(csv.DictReader(open(ROWCSV)))[0]
T0 = float(row['t_start'])
L1, L2, L3 = (float(row[k]) for k in ('leg1_duration_s', 'leg2_duration_s', 'leg3_duration_s'))
t1, t2, t3 = T0 + L1, T0 + L1 + L2, T0 + L1 + L2 + L3
print("scan meta:", str(z['meta']))
print(f"trial {row['trial']}  leg3_ok={row['leg3_ok']}  legs: 1=[0,{L1:.0f}]s 2=[{L1:.0f},{L1 + L2:.0f}]s 3=[{L1 + L2:.0f},{L1 + L2 + L3:.0f}]s")

S, St = z['scan'], z['scan_t']
S3, S3t = z['scan360'], z['scan360_t']
odo_t, odo = z['odom_t'], z['odom']                  # x, y, yaw, vx, wz
sta_t, sta = z['stalled_t'], z['stalled']
stt, stv = z['state_t'], z['state']
act_t, act = z['act_t'], z['act']
print(f"messages: /scan {len(S)}  /scan_360 {len(S3)}  odom {len(odo)}  stalled {len(sta)}  state {len(stv)}")


def scan_diff(scans, times, now):
    i = np.searchsorted(times, now, side='right') - 1
    j = np.searchsorted(times, now - WIN, side='right') - 1     # newest scan with age >= WIN
    if i < 0 or j < 0 or i == j:
        return 0, None
    a, b = scans[i], scans[j]
    ok = np.isfinite(a) & np.isfinite(b)
    n = int(ok.sum())
    return (n, float(np.abs(a[ok] - b[ok]).mean())) if n else (0, None)


def verdict(now):
    n3, d3 = scan_diff(S3, S3t, now)
    nr, dr = scan_diff(S, St, now)
    if n3 >= MINB:
        return 'scan_360', n3, d3, d3 < THR
    if nr >= MINB:
        return 'scan', nr, dr, dr < THR
    return None, max(n3, nr), None, False


def motion(now):
    i1 = np.searchsorted(odo_t, now, side='right') - 1
    i0 = np.searchsorted(odo_t, now - WIN, side='right') - 1
    if i0 < 0 or i1 <= i0:
        return None, None
    d = math.hypot(odo[i1, 0] - odo[i0, 0], odo[i1, 1] - odo[i0, 1])
    dyaw = abs(math.atan2(math.sin(odo[i1, 2] - odo[i0, 2]), math.cos(odo[i1, 2] - odo[i0, 2])))
    return d, dyaw


def phase(now):
    if now < t1: return 'leg1 (flat drive)'
    if now < t2: return 'leg2 (ramp up + platform)'
    if now < t3:
        k = np.searchsorted(stt, now, side='right') - 1
        return 'leg3 DESCENT' if k >= 0 and stv[k] == 'DESCENT' else 'leg3 platform wander'
    return 'leg4 (flat drive)'


# ---- sweep the detector's verdict over the whole trial ----
grid = np.arange(T0 + WIN + 1, float(odo_t[-1]), 0.5)
acc = {}
for now in grid:
    sens, n, d, nomove = verdict(now)
    dist, dyaw = motion(now)
    if dist is None:
        continue
    a = acc.setdefault(phase(now), dict(n=0, moving=0, moving_nomove=0, sens={}, cnt=[], diff=[]))
    a['n'] += 1
    a['sens'][sens] = a['sens'].get(sens, 0) + 1
    if sens is not None:
        a['cnt'].append(n); a['diff'].append(d)
    truly_moving = dist > 0.5 or dyaw > 0.35            # >0.5 m or >20 deg in 5 s
    if truly_moving:
        a['moving'] += 1
        if nomove:
            a['moving_nomove'] += 1

print("\n== detector verdict vs real motion, by phase (0.5 s grid; 'truly moving' = >0.5 m or >20 deg over the prior 5 s) ==")
print(f"{'phase':28s} {'samples':>7s} {'sensor used':>26s} {'valid beams(med)':>16s} {'mean diff(med)':>14s} {'truly moving':>12s} {'...but detector says NO MOVEMENT':>32s}")
for ph, a in sorted(acc.items()):
    sens = ', '.join(f"{k or 'none'}:{v}" for k, v in a['sens'].items())
    cm = f"{np.median(a['cnt']):.0f}" if a['cnt'] else '-'
    dm = f"{np.median(a['diff']):.3f} m" if a['diff'] else '-'
    frac = f"{a['moving_nomove']}/{a['moving']} ({100 * a['moving_nomove'] / a['moving']:.0f}%)" if a['moving'] else '-'
    print(f"{ph:28s} {a['n']:7d} {sens:>26s} {cm:>16s} {dm:>14s} {a['moving']:12d} {frac:>32s}")

# ---- at each real latch: what did the detector see, what did the robot do? ----
print("\n== each stall latch (rising edge of /motion_stalled): detector inputs vs real motion in the prior 5 s ==")
print(f"{'t_rel':>7s} {'phase':28s} {'sensor':>9s} {'beams':>6s} {'mean diff':>10s} {'moved (m)':>10s} {'turned (deg)':>13s}")
edges = [sta_t[i] for i in range(1, len(sta)) if sta[i] == 1 and sta[i - 1] == 0]
false_lat = 0
for e in edges:
    sens, n, d, nomove = verdict(e - 0.05)
    dist, dyaw = motion(e)
    if dist is None:
        continue
    fl = dist > 0.5 or dyaw > 0.35
    false_lat += fl
    print(f"{e - T0:7.1f} {phase(e):28s} {str(sens):>9s} {n:6d} {('%.4f' % d) if d is not None else '-':>10s} {dist:10.2f} {math.degrees(dyaw):13.1f}{'   <-- robot was clearly moving' if fl else ''}")
print(f"latches: {len(edges)}; with clear real motion in the prior 5 s (>0.5 m or >20 deg): {false_lat}")

# ---- why is the scan blind? compare what moves vs what the scan sees ----
print("\n== what the scan looks like while the robot moves on the platform ==")
k = np.where((St > t2 + 20) & (St < t3 - 60))[0]
if len(k) > 5:
    sc = S[k[len(k) // 2]]
    fin = np.isfinite(sc)
    print(f"/scan sample: {fin.sum()}/{len(sc)} beams finite; range median {np.median(sc[fin]):.2f} m, "
          f"min {sc[fin].min():.2f}, max {sc[fin].max():.2f}, std across beams {sc[fin].std():.3f}")
    a0 = S[k[0]]; a1 = S[k[min(len(k) - 1, 50)]]
    m = np.isfinite(a0) & np.isfinite(a1)
    print(f"/scan two samples ~{St[k[min(len(k) - 1, 50)]] - St[k[0]]:.1f}s apart: beams finite in both {m.sum()}, mean|diff| {np.abs(a0[m] - a1[m]).mean():.4f} m")
k3 = np.where((S3t > t2 + 20) & (S3t < t3 - 60))[0]
if len(k3) > 5:
    sc = S3[k3[len(k3) // 2]]
    print(f"/scan_360 sample on platform: {np.isfinite(sc).sum()}/{len(sc)} beams finite")
