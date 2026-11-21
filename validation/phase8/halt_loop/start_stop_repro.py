#!/usr/bin/env python3
"""HALT/recovery loop reproduction + instrumentation (halt_loop session).

Repeated START-FROM-REST events under Nav2/MPPI on flat ground in the construction site laydown (flat_mode), with a closed-loop 180 deg turn between legs - the situation in which the
75-HALT thrash occurred (descent start on the platform after the turn). Every start is an onset from rest; every leg end is a stop. Records, at message rate and with wall-clock stamps:
  /model/ramp_robot/cmd_vel (actuation), /cmd_vel_gated_in (pre-watchdog), /joint_states (4 wheel velocities), /wheel_slip, /stability_margin, /motion_stalled, /ramp_state, EKF twist.
Also logs the FSM-relevant IMU tick values. Usage: start_stop_repro.py <out.json> [n_legs]"""
import json, math, os, sys, time
from pathlib import Path as _P
for _d in _P(__file__).resolve().parents:
    if (_d / 'harness' / 'harness_guard.py').exists():
        sys.path.insert(0, str(_d / 'harness')); break
import harness_guard as hg
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Imu, JointState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import random

OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/halt_loop.json'
N_LEGS = int(sys.argv[2]) if len(sys.argv) > 2 else 20
INJECT = os.environ.get('INJECT', '') not in ('', '0')      # one no-op set_parameters (an optimizer reset, exactly what the old keepalive did) per leg, mid-leg, at speed
LEG_M = 8.0
X_LO, X_HI = 3.0, 25.0          # odom x limits of the flat laydown lane used (robot spawns at odom 0)

class P(Node):
    def __init__(self):
        super().__init__('halt_loop_probe')
        self.ekf_yaw = 0.0; self.odom_x = 0.0; self.state = 'APPROACH'
        self.ev = dict(state=[], cmd=[], gated=[], joint=[], slip=[], margin=[], stall=[], ekf=[], imu=[])
        n = lambda: time.time()
        self.create_subscription(String, '/ramp_state', lambda m: self.ev['state'].append((n(), m.data)) or setattr(self, 'state', m.data), 50)
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', lambda m: self.ev['cmd'].append((n(), round(m.linear.x, 4), round(m.angular.z, 4))), 100)
        self.create_subscription(Twist, '/cmd_vel_gated_in', lambda m: self.ev['gated'].append((n(), round(m.linear.x, 4), round(m.angular.z, 4))), 100)
        self.create_subscription(JointState, '/joint_states', self._on_joint, 100)
        self.create_subscription(Float32, '/wheel_slip', lambda m: self.ev['slip'].append((n(), round(m.data, 4))), 100)
        self.create_subscription(Float32, '/stability_margin', lambda m: self.ev['margin'].append((n(), round(m.data, 4))), 50)
        self.create_subscription(Bool, '/motion_stalled', lambda m: self.ev['stall'].append((n(), int(m.data))), 50)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, 50)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.client = ActionClient(self, FollowPath, '/follow_path')
        self.legs = []; self.injections = []
        self.setp = self.create_client(SetParameters, '/controller_server/set_parameters')

    def _on_joint(self, m):
        v = {nm: vel for nm, vel in zip(m.name, m.velocity)}
        self.ev['joint'].append((time.time(), *[round(v.get(k, 0.0), 3) for k in ('front_left_wheel_joint', 'rear_left_wheel_joint', 'front_right_wheel_joint', 'rear_right_wheel_joint')]))

    def _on_ekf(self, m):
        q = m.pose.pose.orientation
        self.ekf_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_x = m.pose.pose.position.x
        self.ev['ekf'].append((time.time(), round(m.twist.twist.linear.x, 4), round(m.twist.twist.angular.z, 4), round(self.odom_x, 3)))

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end: rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(8): self.cmd_pub.publish(Twist()); rclpy.spin_once(self, timeout_sec=0.03)

    def turn(self, target, tol_deg=4.0, wz_max=0.5, timeout_s=40.0):
        end = time.time() + timeout_s
        while time.time() < end:
            err = math.atan2(math.sin(target - self.ekf_yaw), math.cos(target - self.ekf_yaw))
            if abs(err) < math.radians(tol_deg): break
            t = Twist(); t.angular.z = math.copysign(min(wz_max, max(0.12, 0.8 * abs(err))), err)
            self.cmd_pub.publish(t); rclpy.spin_once(self, timeout_sec=0.03)
        self.stop()

    def inject_reset(self):
        """no-op set_parameters (same value): resets MPPI's optimizer without changing anything - the natural trigger of the loop, delivered on demand"""
        if not self.setp.service_is_ready(): return
        req = SetParameters.Request(parameters=[Parameter(name='FollowPath.vx_max', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.8))])
        self.injections.append(time.time()); f = self.setp.call_async(req); self._keep = getattr(self, '_keep', []) + [f]

    def leg(self, xa, xb, yaw):
        d = 1.0 if xb > xa else -1.0
        path = Path(); path.header.frame_id = 'odom'; path.header.stamp = self.get_clock().now().to_msg()
        x = xa
        while (d > 0 and x <= xb) or (d < 0 and x >= xb):
            ps = PoseStamped(); ps.header = path.header; ps.pose.position.x = x
            ps.pose.orientation.z = math.sin(yaw / 2); ps.pose.orientation.w = math.cos(yaw / 2); path.poses.append(ps); x += 0.4 * d
        goal = FollowPath.Goal(); goal.path = path; goal.controller_id = 'FollowPath'
        t_start = time.time(); fut = self.client.send_goal_async(goal); end = time.time() + 10
        while not fut.done() and time.time() < end: rclpy.spin_once(self, timeout_sec=0.05)
        gh = fut.result()
        if gh is None or not gh.accepted: return None
        rfut = gh.get_result_async(); deadline = time.time() + 60.0; why = 'timeout'
        inj_at = t_start + random.uniform(4.0, 7.0); injected = False
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if INJECT and not injected and time.time() > inj_at and abs(self.odom_x - xb) > 2.5 and self.state in ('APPROACH', 'EXIT'):
                self.inject_reset(); injected = True
            if abs(self.odom_x - xb) < 0.5: why = 'reached'; break
            if rfut.done(): why = 'result'; break
        cf = gh.cancel_goal_async(); self._keep = getattr(self, '_keep', []) + [fut, rfut, cf]
        self.stop(); self.spin_for(1.0)
        return dict(t_start=t_start, t_end=time.time(), why=why, xa=xa, xb=xb, final_x=self.odom_x)

def main():
    guard = hg.verify(); mon = hg.HealthMonitor(period=20.0).start()
    rclpy.init(); n = P(); n.client.wait_for_server(timeout_sec=15.0); n.spin_for(3.0)
    yaw = 0.0
    try:
        for k in range(N_LEGS):
            xa = n.odom_x; xb = min(xa + LEG_M, X_HI) if yaw == 0.0 else max(xa - LEG_M, X_LO)
            if abs(xb - xa) < 3.0:                       # too close to the lane end: turn around first
                yaw = math.pi if yaw == 0.0 else 0.0; n.turn(yaw); n.spin_for(1.5); continue
            r = n.leg(xa + (0.3 if yaw == 0.0 else -0.3), xb, yaw); n.legs.append(r)
            n.get_logger().info(f'leg {k}: {r["why"] if r else "REJECTED"} x {xa:.1f}->{n.odom_x:.1f} state={n.state}')
            n.spin_for(1.5)
            yaw = math.pi if yaw == 0.0 else 0.0; n.turn(yaw); n.spin_for(1.5)
    finally:
        n.stop(); health = mon.stop()
        json.dump(dict(t0=None, legs=n.legs, injections=n.injections, ev=n.ev, health=health, guard=guard), open(OUT, 'w'))
        print(f'wrote {OUT}: legs={len(n.legs)} halts={sum(1 for e in n.ev["state"] if e[1]=="HALT")} health={health}', flush=True)
        os._exit(0)

if __name__ == '__main__':
    main()
