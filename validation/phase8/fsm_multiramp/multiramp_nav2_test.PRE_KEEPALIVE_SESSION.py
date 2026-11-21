#!/usr/bin/env python3
"""Nav2/MPPI-driven multi-ramp traversal with live /ramp_state + live controller-parameter instrumentation.

Purpose (fsm_multiramp session): find out, WITH LIVE EVIDENCE, what /ramp_state reports while the robot climbs ramp 1, crosses Floor 1, climbs ramp 2,
turns on Floor 2 and comes back down both ramps - all driven by Nav2's MPPI controller (FollowPath action, straight paths), NOT by raw cmd_vel and NOT
by ground-truth-distance driving (the only non-MPPI motion is the 180 degree closed-loop turn on the flat Floor 2, the corrected turn standard).

Recorded for every run (JSON, one file):
  * every /ramp_state transition with the FSM's own inputs at that instant (IMU pitch + pitch rate, odom body vx, odom x, Gazebo ground-truth x/z, break-point look-ahead)
  * the live controller parameters read back from /controller_server (rcl_interfaces GetParameters - the service `ros2 param get` calls) at 2 Hz:
    FollowPath.vx_max / wz_max / ax_max / PathAngleCritic.max_angle_to_furthest / PathAlignCritic.cost_weight / offset_from_furthest
  * Gazebo ground truth (x, y, z) at ~1 Hz (background thread) -> physical truth of "climbing" / "descending" (dz/dt) to grade the labels independently of the FSM
  * every IMU message with the other FSM inputs (margin, slip, stall, perception) so the FSM's decision function can be REPLAYED offline (old vs new code)
  * simulator real-time factor from /clock vs wall time (5 s windows) during the live-driven legs
Usage: multiramp_nav2_test.py <out.json> [--legs all|ascent]
"""
import json, math, os, sys, threading, time, subprocess
from pathlib import Path as _P
for _d in _P(__file__).resolve().parents:
    if (_d / 'harness' / 'harness_guard.py').exists():
        sys.path.insert(0, str(_d / 'harness')); break
import harness_guard as hg
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters
from ramp_autonomy_msgs.msg import RampPercept

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/multiramp.json'
LEGS = sys.argv[3] if len(sys.argv) > 3 and sys.argv[2] == '--legs' else 'all'
WORLD = 'construction_site'
PARAMS = ['FollowPath.vx_max', 'FollowPath.wz_max', 'FollowPath.ax_max', 'FollowPath.PathAngleCritic.max_angle_to_furthest',
          'FollowPath.PathAlignCritic.cost_weight', 'FollowPath.PathAlignCritic.offset_from_furthest']
PARAM_KEYS = ['vx_max', 'wz_max', 'ax_max', 'angle_tol_rad', 'align_weight', 'align_offset']
# odom-frame geometry (odom x = gazebo x - 1.0)
X_LAYDOWN_END = 27.0
X_FLOOR1_BEFORE_RAMP2 = 54.0      # 1 m before Floor 1's east edge / ramp-2 base (odom 54.97)
X_FLOOR2_TARGET = 75.5            # 7.5 m onto Floor 2 (ramp-2 top seam at odom 67.94)
X_FLOOR1_AFTER_DESCENT = 47.0     # Floor 1, between the two ramps (ramp-1 top odom 39.97, ramp-2 base 54.97)
X_GROUND_END = 21.0               # on the flat ground west of ramp 1 (ramp-1 base at odom 27.3)


def get_gz_pose():
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{WORLD}/dynamic_pose/info', '-n', '1'], capture_output=True, text=True, timeout=3).stdout
        i = out.find('name: "ramp_robot"')
        if i < 0: return None
        blk = out[i:i + 500]; p = blk.find('position {'); pb = blk[p:p + 150]
        def f(k):
            a = pb.find(k + ':'); b = pb.find('\n', a); return float(pb[a + len(k) + 1:b].strip())
        return f('x'), f('y'), f('z')
    except Exception:
        return None


class Probe(Node):
    def __init__(self):
        super().__init__('multiramp_probe')
        self.t0 = time.time()
        self.state = 'APPROACH'; self.pitch = 0.0; self.pitch_rate = 0.0; self._prev_pitch = 0.0
        self.odom_x = 0.0; self.odom_y = 0.0; self.vx = 0.0; self.ekf_yaw = 0.0
        self.margin = 1.0; self.slip = 0.0; self.stall = False; self.bp_dist = -1.0; self.bp_conf = 0.0
        self.params = {k: None for k in PARAM_KEYS}
        self.gz = None; self.gz_log = []; self._gz_stop = False
        self.transitions = []; self.samples = []; self.imu_log = []; self.legs = []; self.halt_count = 0
        self.sim_clock = None; self.rtf_windows = []; self._rtf_ref = None
        self.create_subscription(Imu, '/imu', self._on_imu, 50)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 10)
        self.create_subscription(Float32, '/stability_margin', lambda m: setattr(self, 'margin', m.data), 10)
        self.create_subscription(Float32, '/wheel_slip', lambda m: setattr(self, 'slip', m.data), 10)
        self.create_subscription(Bool, '/motion_stalled', lambda m: setattr(self, 'stall', m.data), 10)
        self.create_subscription(RampPercept, '/ramp_perception', self._on_percept, 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 20)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.client = ActionClient(self, FollowPath, '/follow_path')
        self.pclient = self.create_client(GetParameters, '/controller_server/get_parameters')
        self._pending = None
        self.create_timer(0.5, self._poll_params)
        self.create_timer(0.5, self._sample)
        threading.Thread(target=self._gz_loop, daemon=True).start()

    # ---- callbacks
    def now(self): return round(time.time() - self.t0, 2)

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.degrees(math.asin(sinp)); self.pitch_rate = self.pitch - self._prev_pitch; self._prev_pitch = self.pitch
        self.imu_log.append((self.now(), round(self.pitch, 4), round(self.margin, 4), round(self.slip, 4), int(self.stall), round(self.bp_conf, 3), round(self.bp_dist, 3),
                             round(self.vx, 4), self.state))

    def _on_state(self, msg):
        if msg.data != self.state:
            gz = self.gz
            self.transitions.append(dict(t=self.now(), frm=self.state, to=msg.data, pitch=round(self.pitch, 2), pitch_rate=round(self.pitch_rate, 2), vx=round(self.vx, 3),
                                         odom_x=round(self.odom_x, 2), gz_x=None if gz is None else round(gz[0], 2), gz_z=None if gz is None else round(gz[2], 3),
                                         bp_dist=round(self.bp_dist, 2), bp_conf=round(self.bp_conf, 2)))
            if msg.data == 'HALT': self.halt_count += 1
            self.state = msg.data

    def _on_odom(self, msg):
        self.vx = msg.twist.twist.linear.x      # raw wheel odometry: body forward speed (the FSM's own direction input); its POSITION is not used (wheel yaw drifts through the turn)

    def _on_ekf(self, msg):
        self.odom_x = msg.pose.pose.position.x; self.odom_y = msg.pose.pose.position.y      # EKF position: the frame MPPI follows the path in
        q = msg.pose.pose.orientation
        self.ekf_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_percept(self, msg):
        self.bp_dist = msg.breakpoint_distance_m; self.bp_conf = msg.confidence

    def _on_clock(self, msg):
        s = msg.clock.sec + msg.clock.nanosec * 1e-9; w = time.time()
        if self._rtf_ref is None: self._rtf_ref = (s, w); return
        if w - self._rtf_ref[1] >= 5.0:
            self.rtf_windows.append((self.now(), round((s - self._rtf_ref[0]) / (w - self._rtf_ref[1]), 3), self.state)); self._rtf_ref = (s, w)

    def _poll_params(self):
        if self._pending is not None and self._pending.done():
            try:
                vals = self._pending.result().values
                for k, v in zip(PARAM_KEYS, vals):
                    self.params[k] = v.double_value if v.type == 3 else (v.integer_value if v.type == 2 else None)
            except Exception:
                pass
            self._pending = None
        if self._pending is None and self.pclient.service_is_ready():
            req = GetParameters.Request(); req.names = PARAMS; self._pending = self.pclient.call_async(req)

    def _gz_loop(self):
        while not self._gz_stop:
            p = get_gz_pose()
            if p is not None:
                self.gz = p; self.gz_log.append((self.now(), round(p[0], 3), round(p[1], 3), round(p[2], 3)))
            time.sleep(0.7)

    def _sample(self):
        self.samples.append(dict(t=self.now(), state=self.state, pitch=round(self.pitch, 2), vx=round(self.vx, 3), odom_x=round(self.odom_x, 2), **{k: (None if v is None else round(v, 4)) for k, v in self.params.items()}))

    def dump(self, ok=None, extra=None):
        out = dict(ok=ok, legs=self.legs, transitions=self.transitions, samples=self.samples, gz=self.gz_log, imu=self.imu_log, rtf_windows=self.rtf_windows,
                   halt_count=self.halt_count, param_keys=PARAM_KEYS)
        out.update(extra or {}); json.dump(out, open(OUT, 'w'))

    # ---- driving
    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end: rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        for _ in range(10):
            self.cmd_pub.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.05)

    def turn_closed_loop(self, target_yaw, tol_deg=4.0, wz_max=0.5, timeout_s=40.0):
        end = time.time() + timeout_s
        while time.time() < end:
            err = math.atan2(math.sin(target_yaw - self.ekf_yaw), math.cos(target_yaw - self.ekf_yaw))
            if abs(err) < math.radians(tol_deg): break
            t = Twist(); t.angular.z = math.copysign(min(wz_max, max(0.12, 0.8 * abs(err))), err)
            self.cmd_pub.publish(t); rclpy.spin_once(self, timeout_sec=0.03)
        self.stop(); return self.ekf_yaw

    def make_path(self, xa, xb, yaw, step=0.4):
        path = Path(); path.header.frame_id = 'odom'; path.header.stamp = self.get_clock().now().to_msg()
        d = 1.0 if xb > xa else -1.0; x = xa
        while (d > 0 and x <= xb) or (d < 0 and x >= xb):
            ps = PoseStamped(); ps.header = path.header; ps.pose.position.x = x
            ps.pose.orientation.z = math.sin(yaw / 2); ps.pose.orientation.w = math.cos(yaw / 2); path.poses.append(ps); x += step * d
        return path

    def follow(self, name, xb, yaw, timeout=420.0, tol=0.5):
        """One MPPI-driven leg: FollowPath along a straight odom-frame path from just ahead of the robot to xb. Physical gate: ground-truth displacement vs odometry."""
        d = 1.0 if xb > self.odom_x else -1.0
        gz0, x0, t_leg0 = self.gz, self.odom_x, self.now()
        path = self.make_path(self.odom_x + 0.3 * d, xb, yaw)
        goal = FollowPath.Goal(); goal.path = path; goal.controller_id = 'FollowPath'
        fut = self.client.send_goal_async(goal); end = time.time() + 10
        while not fut.done() and time.time() < end: rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.legs.append(dict(name=name, ok=False, why='goal rejected')); return False
        rfut = gh.get_result_async(); deadline = time.time() + timeout; status = None; why = 'timeout'
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.odom_x - xb) < tol: why = 'reached target'; break
            if rfut.done():
                status = rfut.result().status; why = f'action result status {status}'; break
        cf = gh.cancel_goal_async(); self._keep = getattr(self, '_keep', []) + [fut, rfut, cf]      # keep the futures alive: a freed pending result future segfaulted rclpy once
        self.stop(); t_wait = time.time() + 6.0
        while time.time() < t_wait and not (cf.done() and rfut.done()): rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_for(1.0); t_stop = self.now()
        t_wait = time.time() + 6.0                                        # wait for a ground-truth sample taken AFTER the robot stopped (the poll thread lags by up to ~1 s)
        while time.time() < t_wait and not (self.gz_log and self.gz_log[-1][0] > t_stop + 0.3): rclpy.spin_once(self, timeout_sec=0.05)
        gz1 = self.gz; odom_disp = self.odom_x - x0
        gt_disp = None if gz0 is None or gz1 is None else gz1[0] - gz0[0]
        dis = None if gt_disp is None else gt_disp - odom_disp
        # physical gate: GT displacement must agree with wheel odometry. 0.5 m for short legs; MPPI-driven long legs showed a real -0.53 m offset over 27 m (wheel-odometry path length on the
        # slope + MPPI weaving), so the tolerance scales as 4 % of the leg (a frozen chassis is off by metres, not centimetres)
        gate = max(0.5, 0.04 * abs(odom_disp))
        ok = abs(self.odom_x - xb) < max(tol, 0.8) and dis is not None and abs(dis) <= gate
        self.legs.append(dict(name=name, t0=t_leg0, t1=self.now(), target_odom_x=xb, final_odom_x=round(self.odom_x, 2), why=why, odom_disp=round(odom_disp, 3),
                              gt_disp=None if gt_disp is None else round(gt_disp, 3), gt_minus_odom=None if dis is None else round(dis, 3), final_gz=None if gz1 is None else [round(v, 3) for v in gz1], ok=ok))
        self.get_logger().info(f'{name}: {why}; odom_x={self.odom_x:.2f} state={self.state} GT-vs-odom {dis} -> ok={ok}')
        self.dump()
        return ok


def main():
    guard = hg.verify(); monitor = hg.HealthMonitor(period=20.0).start()
    rclpy.init(); n = Probe(); n.client.wait_for_server(timeout_sec=15.0); n.spin_for(3.0)
    ok = True
    try:
        steps = [('leg 1: gate -> laydown (MPPI)', 'L1', X_LAYDOWN_END, 0.0), ('leg 2: ramp 1 ascent -> Floor 1 (MPPI)', 'L2', X_FLOOR1_BEFORE_RAMP2, 0.0),
                 ('leg 3: ramp 2 ascent -> Floor 2 (MPPI)', 'L3', X_FLOOR2_TARGET, 0.0)]
        for text, nm, xb, yaw in steps:
            n.get_logger().info(text)
            if not n.follow(nm, xb, yaw): ok = False; break
        if ok and LEGS == 'all':
            n.get_logger().info('turn 180 deg on Floor 2 (closed loop, EKF yaw)'); n.spin_for(2.0)
            n.turn_closed_loop(math.pi); n.spin_for(1.5)
            n.get_logger().info(f'turned: ekf yaw {math.degrees(n.ekf_yaw):.0f} deg')
            for text, nm, xb, yaw in [('leg 4: ramp 2 descent -> Floor 1 (MPPI)', 'L4', X_FLOOR1_AFTER_DESCENT, math.pi), ('leg 5: ramp 1 descent -> ground (MPPI)', 'L5', X_GROUND_END, math.pi)]:
                n.get_logger().info(text)
                if not n.follow(nm, xb, yaw): ok = False; break
    finally:
        n.stop(); n._gz_stop = True
        health = monitor.stop()
        n.dump(ok, dict(health=health, guard_start=guard))
        print(f'wrote {OUT}: ok={ok} legs={[ (l["name"], l["ok"]) for l in n.legs]} transitions={len(n.transitions)} halts={n.halt_count}')
        print('transitions:')
        for t in n.transitions: print('  ', t)
        r = [w[1] for w in n.rtf_windows]
        print(f'sim RTF (from /clock, 5 s windows, n={len(r)}): mean {sum(r)/len(r):.3f} min {min(r):.3f}' if r else 'no /clock RTF windows')
        print(f'HARNESS HEALTH: start={guard} during-run={health}', flush=True)
        sys.stdout.flush(); os._exit(0 if ok else 1)      # rclpy.shutdown() segfaulted with the cancelled action futures; all data is written by now


if __name__ == '__main__':
    main()
