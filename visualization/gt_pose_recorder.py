#!/usr/bin/env python3
"""Record Gazebo ground-truth pose of ramp_robot (x, y, z, roll, pitch, yaw) to CSV at ~20 Hz, from `gz topic --json-output`.
Usage: gt_pose_recorder.py <out.csv> <gz_world_name>   (stop with SIGINT/SIGTERM)"""
import json, math, signal, subprocess, sys, time
out, world = sys.argv[1], sys.argv[2]
stop = False
signal.signal(signal.SIGINT, lambda *a: globals().__setitem__('stop', True)); signal.signal(signal.SIGTERM, lambda *a: globals().__setitem__('stop', True))
p = subprocess.Popen(['gz', 'topic', '-e', '-t', f'/world/{world}/dynamic_pose/info', '--json-output'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
f = open(out, 'w'); f.write('t_wall,x,y,z,roll,pitch,yaw\n'); last = 0.0
for line in p.stdout:
    if stop: break
    now = time.time()
    if now - last < 0.05 or '"ramp_robot"' not in line: continue
    try: m = json.loads(line)
    except Exception: continue
    for q in m.get('pose', []):
        if q.get('name') == 'ramp_robot':
            pos, o = q.get('position', {}), q.get('orientation', {})
            x_, y_, z_, w_ = o.get('x', 0.0), o.get('y', 0.0), o.get('z', 0.0), o.get('w', 1.0)
            roll = math.atan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_)); pitch = math.asin(max(-1, min(1, 2 * (w_ * y_ - z_ * x_)))); yaw = math.atan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))
            f.write(f"{now:.3f},{pos.get('x', 0):.4f},{pos.get('y', 0):.4f},{pos.get('z', 0):.4f},{roll:.4f},{pitch:.4f},{yaw:.4f}\n"); last = now; break
f.close(); p.kill()
