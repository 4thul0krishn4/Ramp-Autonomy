#!/usr/bin/env python3
"""Print the whole-cell timeline recorded by descent_diag.py: ground-truth vs EKF pose/yaw, commands, FSM state.
Usage: analyze_diag.py <run_diag.npz> [dt=1.0] [t_from=-14] [t_to=25]   (times relative to the descent goal being sent)"""
import sys, math
import numpy as np
z = np.load(sys.argv[1], allow_pickle=True)
dt = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
t_from = float(sys.argv[3]) if len(sys.argv) > 3 else -14.0
t_to = float(sys.argv[4]) if len(sys.argv) > 4 else 25.0
T = z['truth']; E = z['ekf']; A = z['act']; N = z['nav']; IM = z['imu']
tg = float(z['t_descent_goal'][0]) if len(z['t_descent_goal']) else float('nan')
st_t, st = z['state_t'], z['state']
deg = lambda a: math.degrees(a)
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def at(arr, t, col):
    return float(np.interp(t, arr[:, 0], arr[:, col])) if len(arr) else float('nan')
def yaw_at(arr, t, col):     # unwrap-safe interpolation of an angle column
    if not len(arr): return float('nan')
    i = np.searchsorted(arr[:, 0], t); i = min(max(i, 1), len(arr) - 1)
    return arr[i, col]
def state_at(t):
    k = np.searchsorted(st_t, t, side='right') - 1
    return st[k] if k >= 0 else '-'
print(f"{z['label']}: descent goal sent at wall {tg:.1f}; truth samples {len(T)}, ekf {len(E)}, act {len(A)}")
print(f"{'t':>6s} {'state':>8s} | {'gt x':>6s} {'gt y':>6s} {'gt yaw':>7s} | {'ekf x':>6s} {'ekf y':>6s} {'ekf yaw':>7s} | {'act vx':>6s} {'act wz':>6s} | {'nav vx':>6s} {'nav wz':>6s}")
t = t_from
while t <= t_to:
    w = tg + t
    a = A[(A[:, 0] > w - dt) & (A[:, 0] <= w)] if len(A) else np.zeros((0, 3))
    n = N[(N[:, 0] > w - dt) & (N[:, 0] <= w)] if len(N) else np.zeros((0, 3))
    m = lambda x, c: f"{np.mean(x[:, c]):6.2f}" if len(x) else '     -'
    print(f"{t:6.1f} {state_at(w):>8s} | {at(T,w,1):6.2f} {at(T,w,2):6.2f} {deg(yaw_at(T,w,4)):7.0f} | {at(E,w,1):6.2f} {at(E,w,2):6.2f} {deg(yaw_at(E,w,3)):7.0f} | {m(a,1)} {m(a,2)} | {m(n,1)} {m(n,2)}")
    t += dt
