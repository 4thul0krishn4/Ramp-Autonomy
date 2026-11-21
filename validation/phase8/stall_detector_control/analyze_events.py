#!/usr/bin/env python3
"""Score recorded sessions against Gazebo ground truth.

For every positive-control event: (1) was the chassis GENUINELY frozen (ground truth), (2) did the
detector latch, and how fast; and during the freeze, which rotation observers told the truth.
For every latch (rising edge of /motion_stalled): was the robot actually moving at that moment.

Definitions (all on ground truth, never on robot sensors):
  frozen interval : starts at the first time the pose has stayed within 3 cm / 2 deg of itself for 1.0 s
                    after the cage appears; VERIFIED if it lasts to the end of the observation with
                    <5 cm drift and <3 deg yaw change.
  moving at t     : over the 2 s before t the robot travelled >= 0.20 m or turned >= 0.10 rad.
  TRUE latch      : robot NOT moving at the latch time AND a verified freeze is in progress.
  FALSE latch     : robot moving at the latch time (the failure being fixed).
Usage: analyze_events.py <session.npz> [more.npz ...] [--json out.json]
"""
import sys, json, math
import numpy as np

EVID_S = 2.0              # a latch is only JUSTIFIED by a freeze (commanded, chassis blocked) that has lasted at least this long
DEADLINE_S = 8.0          # a freeze must be latched within this long of the frozen interval starting
WHEEL_R, HALF_TRACK = 0.12, 0.35


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class Session:
    def __init__(self, path):
        z = np.load(path, allow_pickle=True)
        self.path = path
        self.extra = json.loads(str(z['extra']))
        self.events = json.loads(str(z['events']))
        T = z['truth']
        self.tt, self.tx, self.ty, self.tz, self.tyaw = T[:, 0], T[:, 1], T[:, 2], T[:, 3], np.unwrap(T[:, 4])
        self.imu, self.act, self.wheels = z['imu'], z['act'], z['wheels']
        self.ekf, self.raw = z['odom_ekf'], z['odom_raw']
        self.stalled = z['stalled']
        self.state_t, self.state = z['state_t'], z['state']
        self.t_end = self.tt[-1]
        s = self.stalled
        self.edges = [float(s[i, 0]) for i in range(1, len(s)) if s[i, 1] == 1 and s[i - 1, 1] == 0]
        if len(s) and s[0, 1] == 1:
            self.edges.insert(0, float(s[0, 0]))

    def pose(self, t):
        return (np.interp(t, self.tt, self.tx), np.interp(t, self.tt, self.ty), np.interp(t, self.tt, self.tyaw))

    def moving_at(self, t, span=2.0):
        x1, y1, a1 = self.pose(t)
        x0, y0, a0 = self.pose(t - span)
        return math.hypot(x1 - x0, y1 - y0) >= 0.20 or abs(a1 - a0) >= 0.10

    def stalled_at(self, t):
        i = np.searchsorted(self.stalled[:, 0], t, side='right') - 1
        return bool(i >= 0 and self.stalled[i, 1] == 1)

    def frozen_start(self, t_from, t_to):
        """first time in [t_from, t_to] from which the pose stays within 3 cm / 2 deg for >= 1.0 s"""
        for t in np.arange(t_from, t_to - 1.0, 0.1):
            xs = np.array([self.pose(u) for u in np.arange(t, t + 1.0 + 1e-9, 0.1)])
            if (np.hypot(xs[:, 0] - xs[0, 0], xs[:, 1] - xs[0, 1]).max() < 0.03 and np.abs(xs[:, 2] - xs[0, 2]).max() < np.radians(2)):
                return float(t)
        return None

    def cmd_wheel_rates(self, t0, t1):
        m = (self.act[:, 0] >= t0) & (self.act[:, 0] <= t1)
        vx, wz = self.act[m, 1], self.act[m, 2]
        return np.median(np.abs(vx)), np.median(np.abs(wz)), (vx - wz * HALF_TRACK) / WHEEL_R, (vx + wz * HALF_TRACK) / WHEEL_R


def commanded_at(S, t):
    i = np.searchsorted(S.act[:, 0], t, side='right') - 1
    return i >= 0 and (abs(S.act[i, 1]) >= 0.05 or abs(S.act[i, 2]) >= 0.05)


def blocked_over(S, t, dur=2.0):
    """chassis blocked: NET displacement < 6 cm and NET yaw change < 0.10 rad over the window
    (the complement of moving_at's 0.20 m / 0.10 rad; net, so rocking inside a wedge cancels)"""
    x0, y0, a0 = S.pose(t); x1, y1, a1 = S.pose(t + dur)
    return math.hypot(x1 - x0, y1 - y0) < 0.06 and abs(a1 - a0) < 0.10


def blocked_episodes(S, t0, t1):
    """Maximal intervals where the actuation command is nonzero AND ground truth shows the chassis not
    moving (3 cm / 2 deg over the next second): a genuine freeze - wheels commanded, chassis blocked.
    Immune to HALT pauses (a HALT zeroes the command, so it never counts)."""
    ts = np.arange(t0, t1 - 2.0, 0.1)
    ok = np.array([commanded_at(S, t) and blocked_over(S, t) for t in ts])
    eps, i = [], 0
    while i < len(ts):
        if ok[i]:
            j = i
            gap = 0
            while j + 1 < len(ts) and (ok[j + 1] or gap < 2):
                gap = 0 if ok[j + 1] else gap + 1
                j += 1
            end = ts[j] + 2.0
            if end - ts[i] >= 2.0:
                eps.append((float(ts[i]), float(min(end, t1))))
            i = j + 1
        else:
            i += 1
    return eps


def score_event(S, ev):
    r = dict(kind=ev['kind'], spawn_ok=ev['ok'], rtt=round(ev.get('rtt', 0), 2))
    if not ev['ok'] or 't_obs_end' not in ev:
        r['episodes'] = []
        return r
    t1 = ev['t_obs_end']
    eps = blocked_episodes(S, ev['t_done'] + 0.3, t1)
    out = []
    for (a, b) in eps:
        length = b - a
        det = [e for e in S.edges if a + EVID_S <= e <= b + 0.3]
        early = [e for e in S.edges if a <= e < a + EVID_S]
        rec = dict(start_after_spawn=round(a - ev['t_done'], 1), length=round(length, 1),
                   latch_latency=None if not det else round(det[0] - a, 2), detected=bool(det),
                   early_unjustified_latch=bool(early),
                   # scorable only if it is long enough for a correct detector to have had to fire
                   scorable=bool(det) or length >= DEADLINE_S)
        # what each rotation/velocity source said while the chassis was blocked and being commanded
        w0, w1 = a + 0.3, min(b, a + 4.0)
        if w1 - w0 > 0.5:
            cvx, cwz, cl, cr = S.cmd_wheel_rates(w0, w1)
            m = (S.wheels[:, 0] >= w0) & (S.wheels[:, 0] <= w1)
            wv = np.median(np.abs(S.wheels[m, 1:5])) if m.any() else float('nan')
            def med(arr, col):
                k = (arr[:, 0] >= w0) & (arr[:, 0] <= w1)
                return round(float(np.median(np.abs(arr[k, col]))), 3) if k.any() else None
            rec.update(cmd_vx=round(float(cvx), 3), cmd_wz=round(float(cwz), 3),
                       wheel_rate_over_cmd=round(float(wv / max(1e-6, np.median(np.abs(np.r_[cl, cr])))), 2),
                       wz_imu_gyro=med(S.imu, 7), wz_ekf=med(S.ekf, 5), wz_raw_wheel_odom=med(S.raw, 5),
                       vx_ekf=med(S.ekf, 4), vx_raw_wheel_odom=med(S.raw, 4))
        out.append(rec)
    r['episodes'] = out
    r['n_episodes'] = len(out)
    r['n_scorable'] = sum(e['scorable'] for e in out)
    r['n_detected'] = sum(e['detected'] for e in out)
    return r


def justified(S, e):
    """commanded throughout the EVID_S before the latch AND chassis blocked (net < 6 cm, < 0.10 rad) over it"""
    ts = np.arange(e - EVID_S, e, 0.1)
    if not all(commanded_at(S, t) for t in ts):
        return False
    x0, y0, a0 = S.pose(e - EVID_S); x1, y1, a1 = S.pose(e)
    return math.hypot(x1 - x0, y1 - y0) < 0.06 and abs(a1 - a0) < 0.10


def label_latches(S):
    """MOVING = false latch; JUSTIFIED = true latch on a genuine command-and-blocked freeze; else other"""
    out = []
    for e in S.edges:
        out.append(dict(t=e, moving=S.moving_at(e), justified=justified(S, e)))
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.endswith('.json') and a != '--json']
    jout = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
    allsum = []
    for p in args:
        S = Session(p)
        ex = S.extra
        print(f"\n=== {p.split('/')[-1]}  mode={ex.get('mode')} payload={ex.get('payload')} "
              f"health={ex.get('health')} result={ex.get('result')}")
        evs = [score_event(S, e) for e in S.events]
        for e in evs:
            print('  EVENT', json.dumps(e, default=lambda o: bool(o) if isinstance(o, np.bool_) else float(o)))
        lat = label_latches(S)
        dur = (ex['result'].get('t_leg3_end', S.t_end) - ex['result'].get('t_leg2_end', S.tt[0])) if ex.get('result') else 0
        l3 = [l for l in lat if ex.get('result') and ex['result']['t_leg2_end'] <= l['t'] <= ex['result']['t_leg3_end']]
        fa = [l for l in lat if l['moving']]
        ja = [l for l in lat if l['justified']]
        print(f"  latches total={len(lat)}  MOVING (false)={len(fa)}  JUSTIFIED (true)={len(ja)}  other={len(lat)-len(fa)-len(ja)}  in leg 3: {len(l3)} "
              f"(moving: {sum(l['moving'] for l in l3)}), leg-3 duration {dur:.0f}s")
        allsum.append(dict(file=p, mode=ex.get('mode'), payload=ex.get('payload'), events=evs, latches=lat,
                           leg3_s=dur, result=ex.get('result'), health=ex.get('health')))
    if jout:
        json.dump(allsum, open(jout, 'w'), indent=1, default=lambda o: bool(o) if isinstance(o, np.bool_) else float(o))


if __name__ == '__main__':
    main()
