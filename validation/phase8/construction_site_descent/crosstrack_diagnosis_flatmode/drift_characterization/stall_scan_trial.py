#!/usr/bin/env python3
"""stall_diagnosis_trials.py trial (same guard, same 360 s cap, same gates) plus a
recorder for exactly the inputs motion_stall_detector_node's LiDAR scan-consistency
check uses: raw /scan and /scan_360 ranges, the actuation command it watches, its
own /motion_stalled output, /ramp_state, and real chassis motion from
/odometry/filtered. analyze_scan.py then re-runs the detector's algorithm offline
and compares its verdict to what the robot physically did. Usage: same 3 args as
stall_diagnosis_trials.py plus a 4th: output .npz path.
"""
import sys, time
import numpy as np
import stall_diagnosis_trials as sd
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

NPZ_OUT = sys.argv[4] if len(sys.argv) > 4 else '/tmp/scan_trial.npz'
INSTANCE = None


class ScanTest(sd.Test):
    def __init__(self):
        super().__init__()
        global INSTANCE
        INSTANCE = self
        self.rec = {k: [] for k in ('scan_t', 'scan', 'scan360_t', 'scan360', 'act_t', 'act',
                                    'stalled_t', 'stalled', 'state_t', 'state', 'odom_t', 'odom')}
        self.scan_meta = {}
        self.create_subscription(LaserScan, '/scan', self._s, 10)
        self.create_subscription(LaserScan, '/scan_360', self._s360, 10)
        self.create_subscription(Twist, '/model/ramp_robot/cmd_vel', self._a, 50)
        self.create_subscription(Bool, '/motion_stalled', self._st, 50)
        self.create_subscription(String, '/ramp_state', self._state, 50)
        self.create_subscription(Odometry, '/odometry/filtered', self._o, 50)

    def _s(self, m):
        self.rec['scan_t'].append(time.time()); self.rec['scan'].append(np.asarray(m.ranges, dtype=np.float32))
        self.scan_meta.setdefault('scan', dict(angle_min=m.angle_min, angle_max=m.angle_max,
                                               range_min=m.range_min, range_max=m.range_max, n=len(m.ranges)))

    def _s360(self, m):
        self.rec['scan360_t'].append(time.time()); self.rec['scan360'].append(np.asarray(m.ranges, dtype=np.float32))
        self.scan_meta.setdefault('scan360', dict(angle_min=m.angle_min, angle_max=m.angle_max,
                                                  range_min=m.range_min, range_max=m.range_max, n=len(m.ranges)))

    def _a(self, m):
        self.rec['act_t'].append(time.time()); self.rec['act'].append((m.linear.x, m.angular.z))

    def _st(self, m):
        self.rec['stalled_t'].append(time.time()); self.rec['stalled'].append(int(m.data))

    def _state(self, m):
        self.rec['state_t'].append(time.time()); self.rec['state'].append(m.data)

    def _o(self, m):
        q = m.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.rec['odom_t'].append(time.time())
        self.rec['odom'].append((m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                                 m.twist.twist.linear.x, m.twist.twist.angular.z))


sd.Test = ScanTest
try:
    sd.main()
finally:
    if INSTANCE is not None:
        r = INSTANCE.rec
        np.savez_compressed(
            NPZ_OUT,
            scan_t=np.array(r['scan_t']), scan=np.array(r['scan']),
            scan360_t=np.array(r['scan360_t']), scan360=np.array(r['scan360']),
            act_t=np.array(r['act_t']), act=np.array(r['act']),
            stalled_t=np.array(r['stalled_t']), stalled=np.array(r['stalled']),
            state_t=np.array(r['state_t']), state=np.array(r['state']),
            odom_t=np.array(r['odom_t']), odom=np.array(r['odom']),
            meta=str(INSTANCE.scan_meta))
        print(f"scan recording written: {NPZ_OUT}  scan={len(r['scan'])} scan360={len(r['scan360'])} "
              f"state={len(r['state'])} odom={len(r['odom'])}  meta={INSTANCE.scan_meta}")
