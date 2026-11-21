#!/usr/bin/env python3
"""Analyze a stall_highrate_trial.py recording: what triggers each HALT, is the
slip reading an onset-lag / small-denominator artifact, and do brief slip
HALTs feed the LiDAR stall latch. Usage: analyze_highrate.py <highrate.csv> <trial_row.csv>
"""
import sys, csv, math
import numpy as np
import pandas as pd

HR, ROWCSV = sys.argv[1], sys.argv[2]
HALF_TRACK, WHEEL_R = 0.35, 0.12          # track 0.7 m, wheel radius 0.12 m (stability_monitor_node)
MIN_CMD_RAD_S = 0.05                      # stability_monitor MIN_COMMANDED_SPEED_RAD_S
SLIP_THR = 0.5

row = list(csv.DictReader(open(ROWCSV)))[0]
T3S = float(row['t_start']) + float(row['leg1_duration_s']) + float(row['leg2_duration_s'])
T3E = T3S + float(row['leg3_duration_s'])
print(f"leg3 window: {T3E - T3S:.1f}s  (trial {row['trial']}, leg3_ok={row['leg3_ok']}, halts={row['leg3_halts']})")

df = pd.read_csv(HR, dtype=str)
df['t'] = df['t_wall'].astype(float)
def num(topic, cols):
    d = df[df.topic == topic].copy()
    for i, c in enumerate(cols):
        d[c] = pd.to_numeric(d[['a', 'b', 'c', 'd'][i]], errors='coerce')
    return d[['t'] + cols].sort_values('t').reset_index(drop=True)

act = num('act_cmd', ['vx', 'wz'])
whl = num('wheels', ['FL', 'RL', 'FR', 'RR'])
slp = num('slip', ['slip'])
stl = num('stalled', ['stalled'])
odo = num('odom', ['vx', 'wz', 'x', 'y'])
st = df[df.topic == 'state'][['t', 'a']].rename(columns={'a': 's'}).sort_values('t').reset_index(drop=True)

def asof(tab, t, col):
    i = np.searchsorted(tab['t'].values, t, side='right') - 1
    return float(tab[col].values[i]) if i >= 0 else float('nan')

# ---- HALT episodes inside leg 3 ----
eps = []
s = st['s'].values; t = st['t'].values
i = 0
while i < len(s):
    if s[i] == 'HALT' and (i == 0 or s[i - 1] != 'HALT'):
        j = i
        while j < len(s) and s[j] == 'HALT': j += 1
        t_in = t[i]; t_out = t[j] if j < len(s) else t[-1]
        eps.append((t_in, t_out)); i = j
    else:
        i += 1
eps3 = [(a, b) for a, b in eps if T3S <= a <= T3E]
print(f"HALT episodes: whole trial {len(eps)}, leg 3 {len(eps3)}")

rec = []
for a, b in eps3:
    stalled = asof(stl, a + 0.1, 'stalled')
    w = slp[(slp.t >= a - 0.25) & (slp.t <= a + 0.15)]
    smax = float(w.slip.max()) if len(w) else float('nan')
    cls = 'stall' if stalled >= 1 else ('slip' if smax > SLIP_THR else 'other')
    rec.append(dict(t_in=a, dur=b - a, cls=cls, slip=smax))
E = pd.DataFrame(rec)
win = T3E - T3S
print("\n== HALT time budget in leg 3 ==")
for c, g in E.groupby('cls'):
    print(f"  {c:6s}: {len(g):3d} episodes, total {g.dur.sum():6.1f}s ({100 * g.dur.sum() / win:4.1f}% of leg 3), "
          f"median {g.dur.median():5.2f}s  min {g.dur.min():5.2f}  max {g.dur.max():5.2f}")
print(f"  ALL   : {len(E):3d} episodes, total {E.dur.sum():6.1f}s ({100 * E.dur.sum() / win:4.1f}% of leg 3 spent in HALT)")

# ---- slip-triggered entries: what does the slip reading look like? ----
S = E[E.cls == 'slip'].copy()
print(f"\n== slip-triggered HALT entries: {len(S)} ==")
bins = [(0.5, 0.9), (0.9, 1.1), (1.1, 3), (3, 1e9)]
for lo, hi in bins:
    n = ((S.slip >= lo) & (S.slip < hi)).sum()
    print(f"  slip in [{lo}, {hi if hi < 1e8 else 'inf'}): {n}")

# resume times: command goes from below 0.05 (>=0.3s) to above
m = np.maximum(act.vx.abs().values, act.wz.abs().values)
ta = act.t.values
resumes = []
below_since = None
for k in range(len(ta)):
    if m[k] < 0.05:
        if below_since is None: below_since = ta[k]
    else:
        if below_since is not None and ta[k] - below_since >= 0.3:
            resumes.append(ta[k])
        below_since = None
resumes = np.array(resumes)
print(f"command-resume events (cmd from ~0 for >=0.3s to nonzero), whole trial: {len(resumes)}")

det = []
for _, r in S.iterrows():
    ti = r.t_in
    vx = asof(act, ti, 'vx'); wz = asof(act, ti, 'wz')
    cl, cr = (vx - wz * HALF_TRACK) / WHEEL_R, (vx + wz * HALF_TRACK) / WHEEL_R
    wv = {k: asof(whl, ti - 0.02, k) for k in ['FL', 'RL', 'FR', 'RR']}
    per = {}
    for k, side in (('FL', cl), ('RL', cl), ('FR', cr), ('RR', cr)):
        if abs(side) >= MIN_CMD_RAD_S:
            per[k] = abs(side - wv[k]) / abs(side)
    worst = max(per, key=per.get) if per else None
    cmd_w = {'FL': cl, 'RL': cl, 'FR': cr, 'RR': cr}[worst] if worst else float('nan')
    act_w = wv[worst] if worst else float('nan')
    prev = resumes[resumes <= ti]
    tsr = ti - prev[-1] if len(prev) else float('nan')
    det.append(dict(slip_pub=r.slip, slip_recomputed=max(per.values()) if per else float('nan'),
                    worst=worst, cmd_wheel=cmd_w, act_wheel=act_w,
                    min_abs_cmd=min(abs(cl), abs(cr)), chassis_vx=asof(odo, ti, 'vx'), cmd_vx=vx, cmd_wz=wz,
                    since_resume=tsr))
D = pd.DataFrame(det)
ok = D.dropna(subset=['slip_recomputed'])
print(f"\nrecomputed-vs-published slip at entry (formula check): median |diff| = "
      f"{(ok.slip_pub - ok.slip_recomputed).abs().median():.3f}  (n={len(ok)})")
def cat(r):
    if r.cmd_wheel != r.cmd_wheel: return 'no wheel above cmd floor'
    if abs(r.act_wheel) < 0.15 * abs(r.cmd_wheel): return 'A: wheel at rest while command applied (onset lag, slip~1)'
    if abs(r.cmd_wheel) < 1.0: return 'B: tiny commanded wheel rate (small denominator)'
    if r.act_wheel * r.cmd_wheel < 0: return 'C: actual wheel opposite sign to command'
    return 'D: other (wheel genuinely off command)'
D['cat'] = D.apply(cat, axis=1)
print("what the worst wheel looks like at slip-HALT entry:")
for c, g in D.groupby('cat'):
    print(f"  {len(g):3d}  {c}")
print(f"time since last command resume (s): median {D.since_resume.median():.2f}, "
      f"<=1.5s: {(D.since_resume <= 1.5).sum()}/{D.since_resume.notna().sum()}, <=3s: {(D.since_resume <= 3).sum()}")
print(f"real chassis speed at slip-HALT entry: median |vx| = {D.chassis_vx.abs().median():.3f} m/s "
      f"(median commanded |vx| = {D.cmd_vx.abs().median():.3f})")
print(f"|commanded| min wheel rate at entry (rad/s): median {D.min_abs_cmd.median():.2f}")

# ---- wheel lag after a command resume ----
lags = []
for tr in resumes[(resumes >= T3S) & (resumes <= T3E)]:
    vx = asof(act, tr + 0.4, 'vx'); wz = asof(act, tr + 0.4, 'wz')
    cl, cr = (vx - wz * HALF_TRACK) / WHEEL_R, (vx + wz * HALF_TRACK) / WHEEL_R
    tgt = max(abs(cl), abs(cr)); side = 'L' if abs(cl) >= abs(cr) else 'R'
    if tgt < 1.0: continue
    seg = whl[(whl.t >= tr) & (whl.t <= tr + 3.0)]
    v = (seg.FL.abs() + seg.RL.abs()).values / 2 if side == 'L' else (seg.FR.abs() + seg.RR.abs()).values / 2
    idx = np.where(v >= 0.63 * tgt)[0]
    lags.append(seg.t.values[idx[0]] - tr if len(idx) else float('nan'))
lags = np.array(lags)
print(f"\nwheel lag after command resume (time to reach 63% of commanded wheel rate), n={len(lags)}: "
      f"median {np.nanmedian(lags):.2f}s  p90 {np.nanpercentile(lags, 90):.2f}s  never-reached {np.isnan(lags).sum()}")

# ---- feedback: what preceded each stall latch? ----
T = E[E.cls == 'stall']
print(f"\n== {len(T)} stall-latched HALTs in leg 3: what happened in the 6s before each ==")
out = []
for _, r in T.iterrows():
    a = r.t_in
    n_slip = ((S.t_in >= a - 6) & (S.t_in < a)).sum()
    seg = odo[(odo.t >= a - 5) & (odo.t <= a)]
    disp = math.hypot(seg.x.iloc[-1] - seg.x.iloc[0], seg.y.iloc[-1] - seg.y.iloc[0]) if len(seg) > 1 else float('nan')
    aw = act[(act.t >= a - 5) & (act.t <= a)]
    cmd_frac = float((np.maximum(aw.vx.abs(), aw.wz.abs()) >= 0.05).mean()) if len(aw) else float('nan')
    out.append((n_slip, disp, cmd_frac))
O = pd.DataFrame(out, columns=['slip_halts_prev6s', 'disp_prev5s_m', 'cmd_frac_prev5s'])
print(f"  slip HALT entries in the 6s before: median {O.slip_halts_prev6s.median():.0f}, "
      f"episodes preceded by >=1: {(O.slip_halts_prev6s >= 1).sum()}/{len(O)}")
print(f"  real displacement over the 5s before the latch: median {O.disp_prev5s_m.median():.3f} m "
      f"(detector threshold on mean scan change: 0.05 m); <0.15 m in {(O.disp_prev5s_m < 0.15).sum()}/{len(O)}")
print(f"  fraction of those 5s with a nonzero actuation command: median {O.cmd_frac_prev5s.median():.2f}")

# ---- raw telemetry excerpt so the result is inspectable ----
t0 = T3S + 40
print(f"\n== telemetry excerpt: leg-3 t+40s .. t+70s, 0.5 s resolution ==")
print("  t_rel  state   act_vx  act_wz  wheelL  wheelR  slip   stalled  odom_vx")
for tt in np.arange(t0, t0 + 30, 0.5):
    sidx = np.searchsorted(st.t.values, tt, side='right') - 1
    wl = (asof(whl, tt, 'FL') + asof(whl, tt, 'RL')) / 2
    wr = (asof(whl, tt, 'FR') + asof(whl, tt, 'RR')) / 2
    print(f"  {tt - T3S:5.1f}  {st.s.values[sidx]:7s} {asof(act, tt, 'vx'):6.2f} {asof(act, tt, 'wz'):6.2f} "
          f"{wl:7.2f} {wr:7.2f} {asof(slp, tt, 'slip'):6.2f} {asof(stl, tt, 'stalled'):5.0f}   {asof(odo, tt, 'vx'):6.3f}")

# overall motion duty in leg 3
mv = odo[(odo.t >= T3S) & (odo.t <= T3E)]
print(f"\nleg 3 real chassis motion: fraction of odom samples with |vx|>0.05 m/s = "
      f"{(mv.vx.abs() > 0.05).mean():.2f}; state==HALT fraction = {E.dur.sum() / win:.2f}")
