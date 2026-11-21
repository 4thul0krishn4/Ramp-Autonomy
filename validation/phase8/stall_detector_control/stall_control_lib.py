#!/usr/bin/env python3
"""Building blocks for the stall-detector positive control.

TruthStream   - continuous Gazebo ground-truth pose of the robot (independent of every
                robot sensor, wheel odometry and the EKF), from `gz topic --json-output`.
Cage          - a LOW four-walled fixture spawned around the chassis while it is being
                commanded to move. Wheels keep spinning at the commanded rate, the chassis
                cannot translate or (beyond a few degrees) rotate, so ground truth is an
                unambiguous freeze. Low on purpose (0.30 m): the horizontal ring LiDAR is
                mounted 0.5 m above the base, so like the real 5 cm seam step that first
                froze this robot, the blockage is invisible to it.
Recorder      - raw inputs of every candidate stall signal, at native rate.
"""
import json, math, subprocess, threading, time
import numpy as np
from sensor_msgs.msg import Imu, JointState, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

WORLD = 'construction_site'
# robot footprint incl. wheels (robot.urdf.xacro: base 0.8 x 0.6, wheels at x=+-0.28,y=+-0.35, r=0.12, w=0.08)
ROBOT_L, ROBOT_W = 0.80, 0.78
FLOOR_BELOW_MODEL_Z = 0.125      # model origin sits 0.125 m above the floor (laydown: z=0.275, floor 0.15)
WHEEL_JOINTS = ['front_left_wheel_joint', 'rear_left_wheel_joint',
                'front_right_wheel_joint', 'rear_right_wheel_joint']


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class TruthStream:
    """Ground-truth (t_wall, x, y, z, yaw) of `ramp_robot`, throttled to ~1/min_dt Hz."""
    def __init__(self, world=WORLD, model='ramp_robot', min_dt=0.02):
        self.model, self.min_dt = model, min_dt
        self.samples = []
        self._stop = False
        self.proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', f'/world/{world}/dynamic_pose/info', '--json-output'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self.th = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.th.start()
        t0 = time.time()
        while not self.samples and time.time() - t0 < 15:
            time.sleep(0.1)
        if not self.samples:
            raise RuntimeError('TruthStream: no ground-truth samples received')
        return self

    def _run(self):
        last = 0.0
        needle = f'"{self.model}"'
        for line in self.proc.stdout:
            if self._stop:
                break
            now = time.time()
            if now - last < self.min_dt or needle not in line:
                continue
            try:
                msg = json.loads(line)
            except Exception:
                continue
            for p in msg.get('pose', []):
                if p.get('name') == self.model:
                    pos, o = p.get('position', {}), p.get('orientation', {})
                    self.samples.append((now, pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0),
                                         yaw_from_quat(o.get('x', 0.0), o.get('y', 0.0), o.get('z', 0.0), o.get('w', 1.0))))
                    last = now
                    break

    def latest(self):
        return self.samples[-1]

    def velocity(self, span=0.3):
        """(speed m/s, yaw rate rad/s) from the last `span` seconds."""
        s = self.samples
        if len(s) < 3:
            return 0.0, 0.0
        t1 = s[-1][0]
        i = len(s) - 1
        while i > 0 and t1 - s[i][0] < span:
            i -= 1
        a, b = s[i], s[-1]
        dt = max(1e-3, b[0] - a[0])
        return math.hypot(b[1] - a[1], b[2] - a[2]) / dt, wrap(b[4] - a[4]) / dt

    def stop(self):
        self._stop = True
        try:
            self.proc.kill()
        except Exception:
            pass


def cage_sdf(name, x, y, floor_z, yaw, clearance, wall_t=0.05, wall_h=0.30, mu=1.0):
    # clearance = (front, back, side) metres, or one float for all four
    """SDF for four static walls around a robot-shaped rectangle, expressed in the robot frame."""
    L, W, t, h = ROBOT_L, ROBOT_W, wall_t, wall_h
    cf_, cb_, cs_ = (clearance,) * 3 if not isinstance(clearance, (tuple, list)) else clearance
    zc = h / 2.0 + 0.002

    def box(nm, px, py, sx, sy):
        return (f"<collision name='c_{nm}'><pose>{px:.4f} {py:.4f} {zc:.4f} 0 0 0</pose>"
                f"<geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry>"
                f"<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>"
                f"<visual name='v_{nm}'><pose>{px:.4f} {py:.4f} {zc:.4f} 0 0 0</pose>"
                f"<geometry><box><size>{sx:.4f} {sy:.4f} {h:.4f}</size></box></geometry>"
                f"<material><ambient>0.9 0.2 0.1 1</ambient></material></visual>")
    fxp, fxn, fy = L / 2 + cf_ + t / 2, L / 2 + cb_ + t / 2, W / 2 + cs_ + t / 2
    Wc, Lc = W + 2 * cs_ + 2 * t, L + cf_ + cb_
    body = (box('front', fxp, 0, t, Wc) + box('back', -fxn, 0, t, Wc) +
            box('left', (cf_ - cb_) / 2, fy, Lc, t) + box('right', (cf_ - cb_) / 2, -fy, Lc, t))
    return (f"<sdf version='1.9'><model name='{name}'><static>true</static>"
            f"<pose>{x:.4f} {y:.4f} {floor_z:.4f} 0 0 {yaw:.5f}</pose><link name='link'>{body}</link></model></sdf>")


def _gz_service(service, reqtype, req, timeout_ms=5000):
    t0 = time.time()
    r = subprocess.run(['gz', 'service', '-s', service, '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
                        '--timeout', str(timeout_ms), '--req', req], capture_output=True, text=True, timeout=15)
    return ('true' in r.stdout.lower()), time.time() - t0, (r.stdout + r.stderr).strip()


def spawn_cage(name, x, y, floor_z, yaw, clearance):
    sdf = cage_sdf(name, x, y, floor_z, yaw, clearance)
    return _gz_service(f'/world/{WORLD}/create', 'gz.msgs.EntityFactory', f'sdf: "{sdf}"')


def remove_cage(name):
    return _gz_service(f'/world/{WORLD}/remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')


class Recorder:
    """Mixin for an rclpy Node: raw candidate-signal streams at native rate."""
    def setup_recorder(self, joint_decimate=10):
        self.R = {k: [] for k in ('imu', 'scan_t', 'scan', 'scan360_t', 'scan360', 'act', 'wheels',
                                  'odom_ekf', 'odom_raw', 'stalled', 'state')}
        self._jd, self._jc = joint_decimate, 0
        self.create_subscription(Imu, '/imu', self._r_imu, 200)
        self.create_subscription(LaserScan, '/scan', self._r_scan, 20)
        self.create_subscription(LaserScan, '/scan_360', self._r_scan360, 20)
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', self._r_act, 100)
        self.create_subscription(JointState, '/joint_states', self._r_js, 100)
        self.create_subscription(Odometry, '/odometry/filtered', self._r_odom_ekf, 50)
        self.create_subscription(Odometry, '/odom', self._r_odom_raw, 50)
        self.create_subscription(Bool, '/motion_stalled', self._r_stalled, 50)
        self.create_subscription(String, '/ramp_state', self._r_state, 50)

    def _r_imu(self, m):
        o, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
        self.R['imu'].append((time.time(), o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z))

    def _r_scan(self, m):
        self.R['scan_t'].append(time.time()); self.R['scan'].append(np.asarray(m.ranges, dtype=np.float32))

    def _r_scan360(self, m):
        self.R['scan360_t'].append(time.time()); self.R['scan360'].append(np.asarray(m.ranges, dtype=np.float32))

    def _r_act(self, m):
        self.R['act'].append((time.time(), m.linear.x, m.angular.z))

    def _r_js(self, m):
        self._jc += 1
        if self._jc % self._jd:
            return
        v = dict(zip(m.name, m.velocity))
        self.R['wheels'].append((time.time(),) + tuple(v.get(j, float('nan')) for j in WHEEL_JOINTS))

    def _r_odom_ekf(self, m):
        q = m.pose.pose.orientation
        self.R['odom_ekf'].append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y,
                                   yaw_from_quat(q.x, q.y, q.z, q.w), m.twist.twist.linear.x, m.twist.twist.angular.z))

    def _r_odom_raw(self, m):
        q = m.pose.pose.orientation
        self.R['odom_raw'].append((time.time(), m.pose.pose.position.x, m.pose.pose.position.y,
                                   yaw_from_quat(q.x, q.y, q.z, q.w), m.twist.twist.linear.x, m.twist.twist.angular.z))

    def _r_stalled(self, m):
        self.R['stalled'].append((time.time(), int(m.data)))

    def _r_state(self, m):
        self.R['state'].append((time.time(), m.data))

    def save(self, path, truth, events, extra=None):
        R = self.R
        np.savez_compressed(
            path, imu=np.array(R['imu']), scan_t=np.array(R['scan_t']), scan=np.array(R['scan']),
            scan360_t=np.array(R['scan360_t']), scan360=np.array(R['scan360']),
            act=np.array(R['act']), wheels=np.array(R['wheels']),
            odom_ekf=np.array(R['odom_ekf']), odom_raw=np.array(R['odom_raw']),
            stalled=np.array(R['stalled']), state_t=np.array([s[0] for s in R['state']]),
            state=np.array([s[1] for s in R['state']]),
            truth=np.array(truth.samples), events=json.dumps(events), extra=json.dumps(extra or {}))
