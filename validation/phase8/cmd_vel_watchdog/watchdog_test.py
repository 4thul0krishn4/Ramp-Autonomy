#!/usr/bin/env python3
"""
Live test of cmd_vel_watchdog_node.py's independent HALT enforcement.

Simulates the exact failure mode the watchdog exists for: the normal
MPPI reconfiguration path (nav2_param_switcher_node.py ->
/controller_server/set_parameters) is stuck/failed - here, literally
killed, so it can never correct anything - while the controller keeps
a HIGH vx_max and is actively driving a real path (so it's genuinely
commanding non-zero velocity, not idle). Then HALT is declared (this
script takes over /ramp_state, the same "override" technique used
elsewhere in this project's own test history) and the actuation topic
(/model/ramp_robot/cmd_vel, downstream of the watchdog) is watched to
confirm it goes - and stays - at (or ramps to) zero, while
cmd_vel_gated_in (upstream of the watchdog, i.e. what MPPI/
velocity_smoother actually wanted to send) is watched simultaneously to
prove the upstream command was genuinely non-zero when HALT hit.
"""
import subprocess, time, math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


def make_path(x_start, x_end, step, stamp, frame='odom'):
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = stamp
    x = x_start
    while x <= x_end:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
        x += step
    return path


class Test(Node):
    def __init__(self):
        super().__init__('watchdog_test')
        self.gated_in_last = Twist()
        self.actuation_last = Twist()
        self.gated_in_nonzero_seen = False
        self.timeline = []
        self.t0 = time.time()
        self.create_subscription(Twist, '/cmd_vel_gated_in', self._on_gated, 10)
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', self._on_actuation, 10)
        self.state_pub = self.create_publisher(String, '/ramp_state', 10)
        self.client = ActionClient(self, FollowPath, '/follow_path')

    def _on_gated(self, msg):
        self.gated_in_last = msg
        if abs(msg.linear.x) > 0.01:
            self.gated_in_nonzero_seen = True

    def _on_actuation(self, msg):
        self.actuation_last = msg

    def sample(self):
        t = time.time() - self.t0
        self.timeline.append((round(t, 3), round(self.gated_in_last.linear.x, 4),
                               round(self.actuation_last.linear.x, 4)))

    def spin_for(self, s, period=0.02):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=period)
            self.sample()


def main():
    rclpy.init()
    node = Test()
    node.spin_for(1.0)
    node.client.wait_for_server(timeout_sec=10.0)

    node.get_logger().info('killing nav2_param_switcher_node and state_machine_node '
                            '(simulating a stuck/failed reconfiguration path)...')
    subprocess.run(['pkill', '-9', '-f', 'nav2_param_switcher_node'])
    subprocess.run(['pkill', '-9', '-f', 'state_machine_node'])
    node.spin_for(1.0)

    node.get_logger().info('forcing FollowPath.vx_max=0.8 directly (simulating a stale, unsafe profile)...')
    subprocess.run(['ros2', 'param', 'set', '/controller_server', 'FollowPath.vx_max', '0.8'])
    node.spin_for(0.5)

    node.state_pub.publish(String(data='APPROACH'))
    node.spin_for(0.3)
    goal = FollowPath.Goal()
    goal.path = make_path(0.0, 10.0, 0.4, node.get_clock().now().to_msg())
    goal.controller_id = 'FollowPath'
    fut = node.client.send_goal_async(goal)
    fend = time.time() + 5.0
    while not fut.done() and time.time() < fend:
        rclpy.spin_once(node, timeout_sec=0.1)
    gh = fut.result()
    if gh is None or not gh.accepted:
        node.get_logger().error('goal rejected')
        rclpy.shutdown()
        return

    node.get_logger().info('driving toward the goal, keeping /ramp_state=APPROACH, waiting for real motion...')
    keepalive_end = time.time() + 8.0
    while time.time() < keepalive_end:
        node.state_pub.publish(String(data='APPROACH'))
        rclpy.spin_once(node, timeout_sec=0.1)
        node.sample()
        if node.gated_in_nonzero_seen:
            break

    node.get_logger().info(f'gated_in_nonzero_seen={node.gated_in_nonzero_seen}')

    node.get_logger().info('=== DECLARING HALT (reconfiguration path stays dead throughout) ===')
    halt_t0 = time.time()
    node.state_pub.publish(String(data='HALT'))
    node.spin_for(0.05)

    zero_t = None
    deadline = time.time() + 3.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        node.sample()
        if zero_t is None and abs(node.actuation_last.linear.x) < 1e-6:
            zero_t = time.time() - halt_t0
        node.state_pub.publish(String(data='HALT'))

    print(f'\n=== RESULT ===')
    print(f'time from HALT publish to actuation topic reading exactly zero: '
          f'{zero_t*1000:.1f} ms' if zero_t is not None else 'NEVER WENT ZERO')
    print(f'final gated_in (upstream, what MPPI wanted): {node.gated_in_last.linear.x:.4f}')
    print(f'final actuation (downstream, watchdog output): {node.actuation_last.linear.x:.4f}')
    print('\n=== TIMELINE around HALT (t, gated_in_vx, actuation_vx) ===')
    for row in node.timeline:
        if row[0] >= (halt_t0 - node.t0) - 0.2:
            print(row)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
