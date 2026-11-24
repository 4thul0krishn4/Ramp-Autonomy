#!/usr/bin/env python3
"""
Full loaded delivery mission, ground-truth verified to the same standard as every other round trip in this project: spawn loaded at the gate (payload lumped is NOT
used here - detachable_payload:=true, the payload is the separate model held by DetachableJoint) -> ramp 1 up -> Floor 1 -> ramp 2 up -> Floor 2 -> the delivery pad
-> stop (payload_delivery_node.py fires the release on its own, no test-script command) -> confirm the payload actually landed on the dock (ground truth) -> closed-loop
turn -> ramp 2 down -> Floor 1 -> ramp 1 down -> gate, empty.

Reuses validation/phase8/fsm_multiramp/multiramp_nav2_test.py's Probe (MPPI FollowPath legs, ground-truth-vs-odometry gate per leg, live controller-parameter checks,
RTF windows) unchanged; this script only adds the delivery-zone leg and the payload landing verification.

Phase 8 side-chute redesign: no longer spawns a payload_unloader.py companion process. The old top-mount design needed an external scripted wrench to push the
payload off the deck (see validation/phase8/delivery/README.md section 1 - "nothing pushes it off"); the new side chute (robot.urdf.xacro's payload_chute_joint) is a
genuine incline steep enough to overcome the payload's own friction, so once the DetachableJoint releases, gravity and contact friction alone carry it down and off -
see validation/phase8/payload_chute/README.md for the verification that this is real rigid-body dynamics, not a canned animation.
Usage: full_mission_test.py <out.json>
"""
import json, os, subprocess, sys, time
if len(sys.argv) < 2:
    raise SystemExit('usage: full_mission_test.py <out.json>')
from pathlib import Path as _P

sys.path.insert(0, str(_P(__file__).resolve().parents[1] / 'fsm_multiramp'))
import multiramp_nav2_test as M   # Probe, get_gz_pose, X_* constants, harness_guard as hg

import rclpy
from std_msgs.msg import String

X_DELIVERY_PAD = 78.5             # odom; world x 79.5 (floor2_delivery_zone / dock centre) - 1.0
DOCK_Y_MIN, DOCK_Y_MAX = 0.3, 0.9    # world frame (== odom y, no x/y rotation on Floor 2): floor2_delivery_dock's span, phase-8 side-chute redesign (generate_floor2.py)
DOCK_Z = 6.732                    # world frame; F2_TOP(6.55) + DECK_H(0.032) + payload half-height(0.15) - see generate_floor2.py's own dock comment
RELEASE_TIMEOUT_S = 15.0          # payload_delivery_node's own DWELL_S (1.0 s) plus generous margin for the release + gravity-driven slide to settle


def get_gz_pose_payload():
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{M.WORLD}/dynamic_pose/info', '-n', '1'], capture_output=True, text=True, timeout=3).stdout
        i = out.find('name: "payload_box"')
        if i < 0:
            return None
        blk = out[i:i + 500]
        p = blk.find('position {'); pb = blk[p:p + 150]
        def f(k):
            a = pb.find(k + ':'); b = pb.find('\n', a); return float(pb[a + len(k) + 1:b].strip())
        return f('x'), f('y'), f('z')
    except Exception:
        return None


class MissionProbe(M.Probe):
    def __init__(self):
        super().__init__()
        self.delivery_status_log = []
        self.payload_state_log = []
        self.create_subscription(String, '/delivery/status', lambda m: self.delivery_status_log.append((self.now(), m.data)), 10)
        self.create_subscription(String, '/payload/state', lambda m: self.payload_state_log.append((self.now(), m.data)), 10)

    def wait_for_release(self, timeout=RELEASE_TIMEOUT_S):
        """Stop and wait: the delivery node's own dwell timer + gz's own release are what fire here, not a command from this script (see module docstring)."""
        self.stop()
        end = time.time() + timeout
        released = False
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if any(s == 'detached' for _, s in self.payload_state_log):
                released = True
                break
        # Let gravity + friction carry the payload down the chute and settle on the dock (no scripted push to wait on
        # anymore - see module docstring). ~0.45 m of incline at net accel ~1.48 m/s^2 is under 1 s; 5 s gives real margin
        # for the departure/settle on the dock too.
        self.spin_for(5.0)
        return released


def main():
    guard = M.hg.verify()
    monitor = M.hg.HealthMonitor(period=20.0).start()
    rclpy.init()
    n = MissionProbe()
    n.client.wait_for_server(timeout_sec=15.0)
    n.spin_for(3.0)
    ok = True
    released = False      # only reassigned if every forward leg succeeds (see the `if ok:` block below) - initialized
    payload_landed = None  # here so a forward-leg failure still produces a valid `out` dict in `finally`, not a crash.
    payload_start = get_gz_pose_payload()
    try:
        legs = [
            ('leg 1: gate -> laydown', 'L1', M.X_LAYDOWN_END, 0.0),
            ('leg 2: ramp 1 ascent -> Floor 1', 'L2', M.X_FLOOR1_BEFORE_RAMP2, 0.0),
            ('leg 3: ramp 2 ascent -> Floor 2', 'L3', M.X_FLOOR2_TARGET, 0.0),
            ('leg 4: Floor 2 -> delivery pad', 'L4', X_DELIVERY_PAD, 0.0),
        ]
        for text, name, xb, yaw in legs:
            n.get_logger().info(text)
            if not n.follow(name, xb, yaw):
                ok = False
                break

        released = False
        payload_landed = None
        if ok:
            n.get_logger().info('stopped on the delivery pad - waiting for payload_delivery_node.py to release (no command from this script)')
            released = n.wait_for_release()
            p_final = get_gz_pose_payload()
            payload_landed = dict(
                start=payload_start, final=p_final,
                on_dock=(p_final is not None and DOCK_Y_MIN - 0.05 <= p_final[1] <= DOCK_Y_MAX + 0.05 and abs(p_final[2] - DOCK_Z) <= 0.03),
            )
            n.get_logger().info(f'released={released} payload landing check: {payload_landed}')
            ok = ok and released and payload_landed['on_dock']

        if ok:
            n.get_logger().info('closed-loop turn on Floor 2 (EKF yaw, the project standard turn)')
            yaw_before = n.ekf_yaw
            n.turn_closed_loop(__import__('math').pi)
            n.get_logger().info(f'turned: ekf yaw {__import__("math").degrees(yaw_before):.0f} -> {__import__("math").degrees(n.ekf_yaw):.0f} deg')
            n.spin_for(1.0)
            return_legs = [
                ('leg 5: ramp 2 descent -> Floor 1', 'L5', M.X_FLOOR1_AFTER_DESCENT, __import__('math').pi),
                ('leg 6: ramp 1 descent -> ground', 'L6', M.X_GROUND_END, __import__('math').pi),
            ]
            for text, name, xb, yaw in return_legs:
                n.get_logger().info(text)
                if not n.follow(name, xb, yaw):
                    ok = False
                    break
    finally:
        n.stop()
        n._gz_stop = True
        health = monitor.stop()
        out = dict(
            ok=ok, legs=n.legs, transitions=n.transitions, delivery_status_log=n.delivery_status_log, payload_state_log=n.payload_state_log,
            payload_landed=payload_landed, released=released,
            samples=n.samples, gz=n.gz_log, imu=n.imu_log, rtf_windows=n.rtf_windows, halt_count=n.halt_count,
            health=health, guard_start=guard, param_keys=M.PARAM_KEYS, t0_epoch=n.t0, cmd=n.cmd_log, percept=n.percept_log,
        )
        json.dump(out, open(sys.argv[1], 'w'))
        print(f'wrote {sys.argv[1]}: ok={ok} legs={[(l["name"], l["ok"]) for l in n.legs]} released={out["released"]} payload_landed={payload_landed} halts={n.halt_count}', flush=True)
        print('delivery status transitions:', [s for i, (t, s) in enumerate(n.delivery_status_log) if i == 0 or s != n.delivery_status_log[i - 1][1]], flush=True)
        r = [w[1] for w in n.rtf_windows]
        if r:
            print(f'sim RTF (from /clock, 5 s windows, n={len(r)}): mean {sum(r) / len(r):.3f} min {min(r):.3f}', flush=True)
        print(f'HARNESS HEALTH: start={guard} during-run={health}', flush=True)
        sys.stdout.flush()
        os._exit(0 if ok else 1)


if __name__ == '__main__':
    main()
