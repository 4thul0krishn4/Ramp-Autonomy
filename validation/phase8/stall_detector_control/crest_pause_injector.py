#!/usr/bin/env python3
"""Causal-test tool. Three arms, each fired ONCE at the first DESCENT entry (after EXIT), none of them a stall latch:
  hold    (default) a pause, by a mechanism that is NOT a stall latch and NOT a HALT: on the first entry into DESCENT (after having seen EXIT) it waits DELAY_S, then publishes /test_hold=True
for DURATION_S. The (temporarily patched) cmd_vel watchdog ramps actuation to zero and holds it exactly as a HALT
would - /ramp_state, the FSM, the MPPI profile and the stall detector are untouched. Records whether the robot
actually stopped.
  ceiling the descent brake's post-HALT restart WITHOUT any pause: FollowPath.ax_max/ax_min/az_max pushed to 40% of
          the ceiling (2.0 m/s^2, 3.0 rad/s^2) at t_desc+delay+duration (when the original's HALT would end), then
          +5% every 0.5 s until 100% - exactly descent_brake_node's profile.
  reset   the MPPI parameter pushes a HALT causes, WITHOUT any pause and WITHOUT changing any limit: the current
          FollowPath.ax_max/ax_min/az_max are re-pushed unchanged at t_desc+delay and t_desc+delay+duration.
Usage: crest_pause_injector.py <out.json> [delay_s=0.4] [duration_s=1.75] [mode=hold|ceiling|reset]"""
import sys, time, json, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

OUT = sys.argv[1]
DELAY_S = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
DURATION_S = float(sys.argv[3]) if len(sys.argv) > 3 else 1.75
MODE = sys.argv[4] if len(sys.argv) > 4 else 'hold'
CEIL_AX, CEIL_AZ = 2.0, 3.0
PNAMES = ['FollowPath.ax_max', 'FollowPath.ax_min', 'FollowPath.az_max']


class Injector(Node):
    def __init__(self):
        super().__init__('crest_pause_injector')
        self.pub = self.create_publisher(Bool, '/test_hold', 10)
        self.setp = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.getp = self.create_client(GetParameters, '/controller_server/get_parameters')
        self.pushes = []                    # (t, description, ok)
        self.frac = None; self.next_push = None
        self.create_subscription(String, '/ramp_state', self._state, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._odom, 50)
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', self._act, 50)
        self.seen_exit = False
        self.fired = False
        self.state = None
        self.t_desc = self.t_start = self.t_end = None
        self.speed, self.act = [], []       # (t, value)
        self.states = []                    # (t, state) changes
        self.holding = False
        self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._republish)

    def _state(self, m):
        if m.data != self.state:
            self.states.append((time.time(), m.data)); self.state = m.data
        if m.data == 'EXIT':
            self.seen_exit = True
        if m.data == 'DESCENT' and self.seen_exit and not self.fired:
            self.fired = True; self.t_desc = time.time()
            self.get_logger().warn(f'first DESCENT after EXIT: hold starts in {DELAY_S:.2f} s for {DURATION_S:.2f} s')

    def _odom(self, m):
        v = m.twist.twist.linear
        self.speed.append((time.time(), (v.x ** 2 + v.y ** 2) ** 0.5))

    def _act(self, m):
        self.act.append((time.time(), abs(m.linear.x) + abs(m.angular.z)))

    def _setparams(self, vals, desc):
        req = SetParameters.Request(parameters=[Parameter(name=n, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))) for n, v in zip(PNAMES, vals)])
        f = self.setp.call_async(req)
        t = time.time()
        f.add_done_callback(lambda fut, t=t, d=desc: self.pushes.append((t, d, all(r.successful for r in fut.result().results) if fut.result() else False)))

    def _reset_push(self):
        f = self.getp.call_async(GetParameters.Request(names=PNAMES))
        def done(fut):
            v = [p.double_value for p in fut.result().values]
            self._setparams(v, f'unchanged re-push {[round(x, 3) for x in v]}')
        f.add_done_callback(done)

    def _tick(self):
        now = time.time()
        if MODE == 'ceiling' and self.fired:
            t0 = self.t_desc + DELAY_S + DURATION_S
            if self.next_push is None and now >= t0:
                self.frac = 0.4; self.next_push = now
            if self.next_push is not None and now >= self.next_push and self.frac is not None and self.frac <= 1.0 + 1e-9:
                self._setparams([CEIL_AX * self.frac, -CEIL_AX * self.frac, CEIL_AZ * self.frac], f'ceiling {self.frac:.2f}')
                self.frac = round(self.frac + 0.05, 3); self.next_push = now + 0.5
                if self.frac > 1.0 + 1e-9: self.frac = None
            return
        if MODE == 'reset' and self.fired:
            if self.t_start is None and now - self.t_desc >= DELAY_S:
                self.t_start = now; self._reset_push()
            if self.t_start is not None and self.t_end is None and now - self.t_start >= DURATION_S:
                self.t_end = now; self._reset_push()
            return
        if MODE != 'hold':
            return
        if self.fired and not self.holding and self.t_start is None and now - self.t_desc >= DELAY_S:
            self.holding = True; self.t_start = now
            self.pub.publish(Bool(data=True))
        if self.holding and now - self.t_start >= DURATION_S:
            self.holding = False; self.t_end = now
            self.pub.publish(Bool(data=False))
            self.get_logger().warn('hold released')

    def _republish(self):
        if self.holding:
            self.pub.publish(Bool(data=True))

    def summary(self):
        r = dict(mode=MODE, fired=self.fired, delay_s=DELAY_S, duration_s=DURATION_S, t_desc=self.t_desc, t_start=self.t_start, t_end=self.t_end,
                 pushes=[(round(t - self.t_desc, 2) if self.t_desc else t, d, ok) for t, d, ok in self.pushes])
        if MODE == 'hold' and self.t_start and self.t_end:
            a, b = self.t_start, self.t_end
            sp = [v for t, v in self.speed if a <= t <= b]
            ac = [v for t, v in self.act if a + 0.6 <= t <= b]          # after the 2 m/s^2 ramp-down
            pre = [v for t, v in self.speed if a - 0.3 <= t < a]
            post = [v for t, v in self.speed if b + 1.0 <= t <= b + 1.6]
            r.update(speed_at_start=(sum(pre) / len(pre)) if pre else None, min_speed_in_hold=min(sp) if sp else None,
                     mean_speed_last_second_of_hold=(sum(v for t, v in self.speed if b - 1.0 <= t <= b) / max(1, len([1 for t, v in self.speed if b - 1.0 <= t <= b]))),
                     max_actuation_after_rampdown=max(ac) if ac else None, speed_1s_after_release=(sum(post) / len(post)) if post else None,
                     states_during=[s for t, s in self.states if a - 0.1 <= t <= b + 0.1])
        return r


def main():
    rclpy.init(); n = Injector()
    def dump(*_):
        json.dump(n.summary(), open(OUT, 'w'), indent=1); rclpy.shutdown(); sys.exit(0)
    signal.signal(signal.SIGTERM, dump)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            json.dump(n.summary(), open(OUT, 'w'), indent=1)
        except Exception:
            pass


if __name__ == '__main__':
    main()
