#!/usr/bin/env python3
"""Isolated EKF fusion validation - NO ramp, NO Nav2/FollowPath involved.
Flat ground only (construction_site.sdf's access road + laydown area,
X:[0,28], well short of the ramp at X=28.3). Drives straight, then does
the EXACT SAME open-loop in-place spin (0.5 rad/s, 6.6 s) that originally
exposed the wheel-odometry-under-turning problem, then drives straight
again - directly comparing /odometry/filtered (the new EKF-fused
estimate) against Gazebo ground truth at every sample, for all three
phases. Reports actual position/yaw error, not just "the EKF is running".
"""
import math, time, csv, sys, subprocess
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

OUT_CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/ekf_validation.csv'
WORLD_NAME = 'construction_site'
DRIVE_VX = 0.4
STRAIGHT1_TARGET_X = 8.0
# Run 1 of this test showed the open-loop spin can land anywhere from
# ~135-180 deg (never precise - the whole reason the EKF exists). Driving
# to a fixed X TARGET for phase 3 meant that when the spin undershot, the
# robot spent much longer covering the diagonal distance needed to still
# reach that X, drifting far enough in Y to physically hit the office
# zone's wall (Y=8) - wheels then kept "reporting" motion into the wall,
# which no pure dead-reckoning approach (raw odom OR EKF) can be expected
# to see coming without an obstacle sensor. Phase 3 now drives for a
# fixed, short TIME instead of to a position target, keeping it well
# inside open space regardless of the actual spin outcome.
STRAIGHT2_DURATION_S = 12.0
GATE_X = 1.0


def get_gz_pose(node_name='ramp_robot'):
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{WORLD_NAME}/dynamic_pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=2
        ).stdout
        idx = out.find(f'name: "{node_name}"')
        if idx < 0:
            return None
        block = out[idx:idx + 500]
        pos_i = block.find('position {')
        pos_block = block[pos_i:pos_i + 150]

        def parse_field(txt, key):
            i = txt.find(key + ':')
            if i < 0:
                return None
            j = txt.find('\n', i)
            return float(txt[i + len(key) + 1:j].strip())

        px = parse_field(pos_block, 'x')
        py = parse_field(pos_block, 'y')
        pz = parse_field(pos_block, 'z')
        ori_i = block.find('orientation {')
        ori_block = block[ori_i:ori_i + 200]
        ox = parse_field(ori_block, 'x')
        oy = parse_field(ori_block, 'y')
        oz = parse_field(ori_block, 'z')
        ow = parse_field(ori_block, 'w')
        return px, py, pz, ox, oy, oz, ow
    except Exception:
        return None


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Test(Node):
    def __init__(self):
        super().__init__('ekf_validation_test')
        self.filt_x = 0.0
        self.filt_y = 0.0
        self.filt_yaw = 0.0
        self.have_filt = False
        self.rows = []
        self.t0 = time.time()
        self.create_subscription(Odometry, '/odometry/filtered', self._on_filt, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

    def _on_filt(self, msg):
        self.filt_x = msg.pose.pose.position.x
        self.filt_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.filt_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_filt = True

    def hard_stop(self, n=15):
        stop = Twist()
        for _ in range(n):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.05)

    def sample(self, phase, tag=''):
        t = time.time() - self.t0
        gz = get_gz_pose()
        row = {'t': round(t, 2), 'phase': phase, 'tag': tag,
               'filt_x': round(self.filt_x, 3), 'filt_y': round(self.filt_y, 3),
               'filt_yaw_deg': round(math.degrees(self.filt_yaw), 2)}
        if gz:
            gx, gy, gz_, ox, oy, oz, ow = gz
            gyaw = yaw_from_quat(ox, oy, oz, ow)
            row['gz_x'] = round(gx, 3)
            row['gz_y'] = round(gy, 3)
            row['gz_yaw_deg'] = round(math.degrees(gyaw), 2)
            row['pos_err_m'] = round(math.hypot(self.filt_x - gx, self.filt_y - gy), 3)
            row['yaw_err_deg'] = round(math.degrees(abs(wrap_angle(self.filt_yaw - gyaw))), 2)
        self.rows.append(row)
        return row

    def drive_to(self, target_x, vx, sample_every=1.0, timeout=120.0, phase='drive'):
        last_sample = 0.0
        cmd = Twist(); cmd.linear.x = vx
        deadline = time.time() + timeout
        going_forward = target_x >= self.filt_x
        while time.time() < deadline:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time() - self.t0
            if now - last_sample >= sample_every:
                last_sample = now
                self.sample(phase)
            if going_forward and self.filt_x >= target_x:
                break
            if not going_forward and self.filt_x <= target_x:
                break
        self.hard_stop()
        self.sample(phase, tag=f'reached_x={target_x}')

    def spin_180(self, sample_every=0.5, phase='spin'):
        cmd = Twist(); cmd.angular.z = 0.5
        end = time.time() + 6.6
        last_sample = 0.0
        while time.time() < end:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time() - self.t0
            if now - last_sample >= sample_every:
                last_sample = now
                self.sample(phase)
        self.hard_stop()
        self.spin_for(1.0)
        self.sample(phase, tag='spin_done')

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive_for(self, duration_s, vx, sample_every=1.0, phase='drive'):
        last_sample = 0.0
        cmd = Twist(); cmd.linear.x = vx
        end = time.time() + duration_s
        while time.time() < end:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time() - self.t0
            if now - last_sample >= sample_every:
                last_sample = now
                self.sample(phase)
        self.hard_stop()
        self.sample(phase, tag='drive_for_done')


def main():
    rclpy.init()
    node = Test()
    # Wait for the EKF to actually start publishing before trusting it.
    wait_end = time.time() + 15.0
    while not node.have_filt and time.time() < wait_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.have_filt:
        node.get_logger().error('never received /odometry/filtered - aborting')
        rclpy.shutdown()
        return
    node.spin_for(1.0)
    node.sample('start')

    node.get_logger().info('phase 1: straight drive (flat ground, no ramp)...')
    node.drive_to(STRAIGHT1_TARGET_X, DRIVE_VX, phase='straight1')
    node.get_logger().info(f'phase 1 done: filt=({node.filt_x:.2f},{node.filt_y:.2f})')

    node.get_logger().info('phase 2: in-place 180 deg spin (0.5 rad/s, 6.6s - same maneuver that originally exposed the problem)...')
    node.spin_180(phase='spin')
    node.get_logger().info(f'phase 2 done: filt_yaw={math.degrees(node.filt_yaw):.1f} deg')

    node.get_logger().info('phase 3: straight drive again (now facing the other way), fixed duration to stay clear of walls...')
    node.drive_for(STRAIGHT2_DURATION_S, DRIVE_VX, phase='straight2')
    node.get_logger().info(f'phase 3 done: filt=({node.filt_x:.2f},{node.filt_y:.2f})')

    with open(OUT_CSV, 'w', newline='') as f:
        fieldnames = ['t', 'phase', 'tag', 'filt_x', 'filt_y', 'filt_yaw_deg',
                      'gz_x', 'gz_y', 'gz_yaw_deg', 'pos_err_m', 'yaw_err_deg']
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in node.rows:
            w.writerow(row)

    # Summary stats per phase
    print('\n=== EKF VALIDATION RESULT ===')
    for phase in ['straight1', 'spin', 'straight2']:
        rows = [r for r in node.rows if r['phase'] == phase and 'pos_err_m' in r]
        if not rows:
            print(f'{phase}: no ground-truth-matched samples')
            continue
        pos_errs = [r['pos_err_m'] for r in rows]
        yaw_errs = [r['yaw_err_deg'] for r in rows]
        print(f'{phase}: n={len(rows)} pos_err_m[min={min(pos_errs):.3f} max={max(pos_errs):.3f} mean={sum(pos_errs)/len(pos_errs):.3f}] '
              f'yaw_err_deg[min={min(yaw_errs):.2f} max={max(yaw_errs):.2f} mean={sum(yaw_errs)/len(yaw_errs):.2f}]')
    print(f'wrote {len(node.rows)} rows to {OUT_CSV}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
