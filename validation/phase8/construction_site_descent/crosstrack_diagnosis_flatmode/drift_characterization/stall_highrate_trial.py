#!/usr/bin/env python3
"""One stall_diagnosis_trials.py trial (unchanged logic, same gates) plus a
native-rate recorder for the signals that decide HALT: the actuation command
the slip metric is computed against, the 4 real wheel joint velocities, the
published /wheel_slip, /motion_stalled, /ramp_state, and the real chassis
speed from /odometry/filtered. Dumps one long-format CSV (t_wall, t_sim,
topic, a, b, c, d) so each HALT entry can be examined at message resolution
instead of the 3 s snapshots. Usage: same 3 args as stall_diagnosis_trials.py
plus an optional 4th: output path for the high-rate CSV.
"""
import sys, time, csv
import stall_diagnosis_trials as sd
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String
from nav_msgs.msg import Odometry

HR_OUT = sys.argv[4] if len(sys.argv) > 4 else '/tmp/highrate.csv'
WHEELS = ['front_left_wheel_joint', 'rear_left_wheel_joint',
          'front_right_wheel_joint', 'rear_right_wheel_joint']
INSTANCE = None


class HighRateTest(sd.Test):
    def __init__(self):
        super().__init__()
        global INSTANCE
        INSTANCE = self
        self.hr = []
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', self._hr_act, 50)
        self.create_subscription(Twist, '/cmd_vel_nav', self._hr_nav, 50)
        self.create_subscription(JointState, '/joint_states', self._hr_js, 50)
        self.create_subscription(Float32, '/wheel_slip', self._hr_slip, 50)
        self.create_subscription(Bool, '/motion_stalled', self._hr_stalled, 50)
        self.create_subscription(String, '/ramp_state', self._hr_state, 50)
        self.create_subscription(Odometry, '/odometry/filtered', self._hr_odom, 50)

    def _row(self, topic, a='', b='', c='', d=''):
        self.hr.append((round(time.time(), 3),
                        round(self.get_clock().now().nanoseconds * 1e-9, 3),
                        topic, a, b, c, d))

    def _hr_act(self, m):
        self._row('act_cmd', m.linear.x, m.angular.z)

    def _hr_nav(self, m):
        self._row('nav_cmd', m.linear.x, m.angular.z)

    def _hr_js(self, m):
        v = dict(zip(m.name, m.velocity))
        self._row('wheels', *[round(v.get(w, float('nan')), 4) for w in WHEELS])

    def _hr_slip(self, m):
        self._row('slip', round(m.data, 4))

    def _hr_stalled(self, m):
        self._row('stalled', int(m.data))

    def _hr_state(self, m):
        self._row('state', m.data)

    def _hr_odom(self, m):
        self._row('odom', round(m.twist.twist.linear.x, 4),
                  round(m.twist.twist.angular.z, 4),
                  round(m.pose.pose.position.x, 3), round(m.pose.pose.position.y, 3))


sd.Test = HighRateTest
try:
    sd.main()
finally:
    if INSTANCE is not None:
        with open(HR_OUT, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_wall', 't_sim', 'topic', 'a', 'b', 'c', 'd'])
            w.writerows(INSTANCE.hr)
        print(f'high-rate rows written: {len(INSTANCE.hr)} -> {HR_OUT}')
