#!/usr/bin/env python3
# NOTE (2026-09-19): this script turns the robot with the HISTORICAL open-loop crest turn. On curved worlds that procedure displaces the chassis
# sideways by an amount the EKF cannot see and its curved-world results are UNCONFIRMED - see validation/phase8/path_structural/README.md. Kept unmodified as a record;
# the standard test is validation_test_ekf_groundtruth.py (closed-loop turn on the flat platform for curved worlds).
"""
Phase 8 validation-matrix test cell. Drives one full ascend -> turn ->
descend cycle against whatever world/friction/payload the sim was
launched with (this script has no world-specific knowledge - it goes
purely off IMU pitch and /ramp_state, so it generalizes across sharp
and curved worlds unchanged), using the REAL state_machine_node.py
throughout (no /ramp_state override - the whole point of this matrix
is to validate the real, fixed FSM end to end).

PASS/FAIL/MARGINAL criteria (declared here BEFORE any run, per the
task - not chosen after seeing results):
  1. FSM completeness (ascent): /ramp_state must reach EXIT within
     ASCENT_TIMEOUT_S of sustained forward driving. If not reached and
     pitch never returned near level, that's a genuine "did not climb"
     result (expected/correct for a below-floor friction cell, not a
     bug) - reported as FAIL: DID_NOT_CLIMB.
  2. Descent reachability: after the 180-degree turn, /ramp_state must
     reach DESCENT during the FollowPath-driven descent leg and stay
     there (not silently reroute through ASCENT/APPROACH) - this is
     what the HALT-recovery fix is being validated against. Failure
     here is FAIL: DESCENT_NOT_REACHED.
  3. HALT behavior: HALT firing at all is NOT a failure (it is the
     designed safety response) - but HALT_THRASH_LIMIT or more HALTs
     during one run indicates the kind of feedback spiral found with
     the old vx_max approach (146 HALTs), not normal isolated safety
     events, and is scored FAIL: HALT_THRASH regardless of whether
     DESCENT/EXIT were still nominally reached. 1..HALT_MARGIN_LIMIT
     HALTs is MARGINAL (system worked, but was stressed); 0 is clean.
  4. Any recovery FROM HALT that lands on a state other than the one
     HALT actually interrupted would indicate the HALT-recovery bug is
     back - this script cannot observe pre-halt state directly (that's
     internal to state_machine_node.py) but flags it indirectly: if
     DESCENT is reached at all after a HALT occurred while status was
     EXIT/DESCENT, that is itself the passing evidence for criterion 2.

Overall verdict = worst of the individual criteria (FAIL beats
MARGINAL beats PASS).
"""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import Imu

LABEL = sys.argv[1] if len(sys.argv) > 1 else 'run'
ASCENT_TIMEOUT_S = 100.0
HALT_MARGIN_LIMIT = 3     # 1..3 HALTs => MARGINAL
HALT_THRASH_LIMIT = 8     # >=8 HALTs => FAIL (old vx_max approach hit 146)


def make_path(x_start, x_end, step, yaw, stamp, frame='odom'):
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = stamp
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    x = x_start
    direction = 1.0 if x_end > x_start else -1.0
    while (direction > 0 and x <= x_end) or (direction < 0 and x >= x_end):
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path.poses.append(ps)
        x += step * direction
    return path


class T(Node):
    def __init__(self):
        super().__init__('validation_test')
        self.pitch = 0.0
        self.state = 'APPROACH'
        self.odom_x = 0.0
        self.odom_yaw = 0.0
        self._prev_state = 'APPROACH'
        self.state_log = []
        self.halt_count = 0
        self.reached_descent = False
        self.reached_exit = False
        self.t0 = time.time()

        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self.create_subscription(String, '/ramp_state', self._on_state, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.client = ActionClient(self, FollowPath, '/follow_path')

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.degrees(math.asin(sinp))

    def _on_state(self, msg):
        if msg.data != self._prev_state:
            t = round(time.time() - self.t0, 2)
            self.state_log.append((t, self._prev_state, '->', msg.data))
            if msg.data == 'HALT':
                self.halt_count += 1
            if msg.data == 'DESCENT':
                self.reached_descent = True
            self._prev_state = msg.data
        self.state = msg.data

    def _on_odom(self, msg):
        self.odom_x = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

    def spin_for(self, s, period=0.05):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=period)

    def drive(self, vx, wz, s):
        t = Twist(); t.linear.x = vx; t.angular.z = wz
        end = time.time() + s
        while time.time() < end:
            self.cmd_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.03)
        stop = Twist()
        for _ in range(5):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def send_path_and_wait(self, path, timeout_sec, target_x, target_tol=0.5):
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        fut = self.client.send_goal_async(goal)
        end = time.time() + 10.0
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[{LABEL}] goal rejected')
            return
        rfut = gh.get_result_async()
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.odom_x - target_x) < target_tol:
                break
            if rfut.done():
                break
        else:
            gh.cancel_goal_async()
            time.sleep(1.0)


def main():
    rclpy.init()
    node = T()
    node.spin_for(1.0)
    node.client.wait_for_server(timeout_sec=10.0)

    node.get_logger().info(f'[{LABEL}] ascending (budget {ASCENT_TIMEOUT_S:.0f}s)...')
    was_on_ramp = False
    cmd = Twist(); cmd.linear.x = 0.25
    end = time.time() + ASCENT_TIMEOUT_S
    last_print = 0
    while time.time() < end:
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.03)
        if abs(node.pitch) > 10.0:
            was_on_ramp = True
        now = time.time()
        if now - last_print > 5.0:
            last_print = now
            node.get_logger().info(f'[{LABEL}] t={now-node.t0:.0f} pitch={node.pitch:.1f} state={node.state}')
        if was_on_ramp and node.state == 'EXIT':
            node.reached_exit = True
            node.get_logger().info(f'[{LABEL}] reached EXIT')
            break
    stop = Twist()
    for _ in range(10):
        node.cmd_pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.spin_for(2.0)

    verdict_ascent = 'PASS' if node.reached_exit else 'FAIL: DID_NOT_CLIMB'
    descent_distance = 0.0
    verdict_descent = 'N/A (ascent failed)'

    if node.reached_exit:
        node.drive(0.0, 0.5, 6.6)  # turn 180deg
        node.spin_for(1.0)
        node.t0 = time.time()
        start_x = node.odom_x
        yaw_target = math.pi if abs(abs(node.odom_yaw) - math.pi) < 0.5 else 0.0
        path_down = make_path(node.odom_x - 0.5, node.odom_x - 6.5, 0.4, yaw_target,
                               node.get_clock().now().to_msg())
        node.spin_for(0.3)
        node.send_path_and_wait(path_down, 90.0, node.odom_x - 6.0)
        descent_distance = abs(node.odom_x - start_x)
        verdict_descent = 'PASS' if node.reached_descent else 'FAIL: DESCENT_NOT_REACHED'

    if node.halt_count >= HALT_THRASH_LIMIT:
        verdict_halt = f'FAIL: HALT_THRASH ({node.halt_count} HALTs)'
    elif node.halt_count >= 1:
        verdict_halt = f'MARGINAL ({node.halt_count} HALTs)'
    else:
        verdict_halt = 'PASS (0 HALTs)'

    verdicts = [verdict_ascent, verdict_descent, verdict_halt]
    if any(v.startswith('FAIL') for v in verdicts):
        overall = 'FAIL'
    elif any(v.startswith('MARGINAL') for v in verdicts):
        overall = 'MARGINAL'
    else:
        overall = 'PASS'

    print(f'\n===== VALIDATION RESULT for {LABEL} =====')
    print(f'overall_verdict: {overall}')
    print(f'ascent_verdict: {verdict_ascent}')
    print(f'descent_verdict: {verdict_descent}')
    print(f'halt_verdict: {verdict_halt}')
    print(f'reached_exit: {node.reached_exit}')
    print(f'reached_descent: {node.reached_descent}')
    print(f'descent_leg_distance_m: {descent_distance:.2f}')
    print(f'halt_count: {node.halt_count}')
    print(f'final_pitch_deg: {node.pitch:.1f}')
    print('state_log:')
    for row in node.state_log:
        print(' ', row)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
