#!/usr/bin/env python3
"""LIVE positive control of the slip HALT decision on the running stack. The real stability monitor is stopped (by PID) so the only /wheel_slip source is this script, which then plays
synthetic slip episodes at 20 Hz - the rate of the real metric - on an idle robot at the gate, and records when /ramp_state enters/leaves HALT relative to each episode's onset:
 0.05 s spike, 0.10 s spike (the longest artefact ever seen), 0.20 s, 0.30 s and 2.0 s sustained.  (Physical wheel blocking cannot produce a sustained metric in this simulator - the
 wheel velocity controller keeps tracking the command - see README.) Usage: synthetic_slip_control.py <out.json>"""
import json, os, signal, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
OUT = sys.argv[1]
def kill_monitor():
    n = 0
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid(): continue
        try: cmd = open(f'/proc/{pid}/cmdline').read().replace('\0', ' ')
        except Exception: continue
        if 'stability_monitor_node' in cmd and 'synthetic_slip_control' not in cmd and 'launch' not in cmd.split()[0:2]:
            try: os.kill(int(pid), signal.SIGKILL); n += 1
            except Exception: pass
    return n
rclpy.init(); node = Node('synthetic_slip_control'); pub = node.create_publisher(Float32, '/wheel_slip', 10)
states = []; node.create_subscription(String, '/ramp_state', lambda m: states.append((time.time(), m.data)), 50)
def spin(s):
    e = time.time() + s
    while time.time() < e: rclpy.spin_once(node, timeout_sec=0.01)
spin(3.0); killed = kill_monitor(); spin(1.5)
res = []
def play(name, dur):
    t0 = time.time(); n_before = len(states)
    end = t0 + dur
    while time.time() < end:
        pub.publish(Float32(data=0.9)); spin(0.05)
    t_end = time.time()
    for _ in range(40): pub.publish(Float32(data=0.0)); spin(0.05)       # 2 s of calm
    ev = [(round(t - t0, 3), s) for t, s in states[n_before:]]
    halt = next((a for a, s in ev if s == 'HALT'), None); rel = next((a for a, s in ev if s != 'HALT' and halt is not None and a > halt), None)
    res.append(dict(name=name, duration=dur, halt_after_s=halt, released_after_s=rel, events=ev[:6]))
    print(f'{name:28s} HALT at +{halt} s, released at +{rel} s', flush=True)
for name, dur in (('0.05 s spike', 0.05), ('0.10 s spike', 0.10), ('0.20 s sustained', 0.20), ('0.30 s sustained', 0.30), ('2.0 s sustained', 2.0)):
    play(name, dur)
json.dump(dict(killed=killed, results=res), open(OUT, 'w'), indent=1); os._exit(0)
