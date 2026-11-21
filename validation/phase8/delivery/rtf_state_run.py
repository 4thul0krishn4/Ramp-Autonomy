#!/usr/bin/env python3
"""One RTF measurement run on Floor 2 (real-time mode, whole stack up): robot spawned on the delivery pad; state = attached | attached_plugin | detached_idle | detached_away | unloaded.
Measures /clock vs wall time in 5 s windows for a 100 s measurement period after a 20 s settle (and, for the detached states, after the release and unload). 'drive' states run a fixed
raw-cmd_vel pattern (0.3 m/s forward 13 s / backward 13 s along the lane). Host state (AC, power profile, EPP, average CPU MHz every 10 s) is recorded so a contaminated run can be excluded.
Usage: rtf_state_run.py <state> <out.json>"""
import json, os, subprocess, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
STATE, OUT = sys.argv[1], sys.argv[2]
def host():
    def rd(p):
        try: return open(p).read().strip()
        except Exception: return None
    mh = [float(l.split(':')[1]) for l in open('/proc/cpuinfo') if l.startswith('cpu MHz')]
    prof = subprocess.run(['powerprofilesctl', 'get'], capture_output=True, text=True).stdout.strip()
    return dict(ac=rd('/sys/class/power_supply/AC/online') or rd('/sys/class/power_supply/ACAD/online') or rd('/sys/class/power_supply/AC0/online'), profile=prof, epp=rd('/sys/devices/system/cpu/cpu0/cpufreq/energy_performance_preference'),
                gov=rd('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'), avg_mhz=round(sum(mh) / len(mh), 0), max_mhz=round(max(mh), 0), min_mhz=round(min(mh), 0), load1=os.getloadavg()[0])
def poses():
    out = subprocess.run(['gz', 'topic', '-e', '-t', '/world/construction_site/dynamic_pose/info', '-n', '1'], capture_output=True, text=True, timeout=5).stdout; res = {}
    for nm in ('ramp_robot', 'payload_box'):
        i = out.find(f'name: "{nm}"')
        if i < 0: continue
        blk = out[i:i + 700]; p = blk.find('position {'); pb = blk[p:p + 200]
        def f(k):
            a = pb.find(k + ':'); b = pb.find('\n', a); return float(pb[a + len(k) + 1:b].strip())
        res[nm] = tuple(round(f(k), 3) for k in ('x', 'y', 'z'))
    return res
rclpy.init(); n = Node('rtf_state_run'); pub = n.create_publisher(Twist, '/cmd_vel_nav', 10)
wins = []; ref = [None]
def on_clock(m):
    s = m.clock.sec + m.clock.nanosec * 1e-9; w = time.time()
    if ref[0] is None: ref[0] = (s, w); return
    if w - ref[0][1] >= 5.0: wins.append((round(w, 1), round((s - ref[0][0]) / (w - ref[0][1]), 4))); ref[0] = (s, w)
n.create_subscription(Clock, '/clock', on_clock, 20)
def spin(sec):
    e = time.time() + sec
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.02)
res = dict(state=STATE, host_start=host()); spin(6.0)
unl = None
if STATE in ('detached_idle', 'detached_away'):
    unl = subprocess.Popen(['python3', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'payload_unloader.py'), '--log', OUT + '.unloader.log'], stdout=open(OUT + '.unloader.out', 'w'), stderr=subprocess.STDOUT)
    spin(6.0); subprocess.run(['gz', 'topic', '-t', '/payload/detach', '-m', 'gz.msgs.Empty', '-p', ' '], capture_output=True); spin(9.0)
    res['after_unload'] = poses(); print('after unload', res['after_unload'], flush=True)
spin(5.0); wins.clear(); ref[0] = None
t0 = time.time(); hosts = [host()]; drive = STATE in ('attached', 'attached_plugin', 'detached_away', 'unloaded'); phase_t = t0; sign = 1.0
while time.time() - t0 < 100.0:
    if drive:
        if time.time() - phase_t > 13.0: sign = -sign; phase_t = time.time()
        t = Twist(); t.linear.x = 0.3 * sign; pub.publish(t)
    rclpy.spin_once(n, timeout_sec=0.03)
    if len(hosts) < int((time.time() - t0) / 10) + 1: hosts.append(host())
for _ in range(10): pub.publish(Twist()); spin(0.05)
res['final_poses'] = poses(); res['host_during'] = hosts; res['host_end'] = host()
v = [x[1] for x in wins]
res['rtf_windows'] = wins; res['rtf'] = dict(n=len(v), mean=round(sum(v) / len(v), 4), min=min(v), max=max(v)) if v else None
json.dump(res, open(OUT, 'w'), indent=1); print('RTF', res['rtf'], 'avg MHz during', [h['avg_mhz'] for h in hosts][:3], flush=True)
if unl:
    try: unl.wait(timeout=5)
    except Exception: unl.kill()
os._exit(0)
