#!/usr/bin/env python3
"""
One-off diagnostic (NOT a regular demo/validation script): verifies the cargo_mode:=loose descent's dramatic pitch-spike/margin-collapse result is genuine
rigid-body dynamics, not a contact/numerical artifact - see docs/construction_site_design.md "Unsecured cargo demo mode -> verification". Reuses
cargo_descent_demo.py's CargoDemo class UNCHANGED, adds two things: (1) full orientation (quaternion) for both robot and payload alongside their positions
(the original demo only logged position), (2) a poller on /payload/contact (gz-sim's own <sensor type="contact">, added ONLY when launched with
contact_diag:=true - see payload_box.sdf.xacro/ramp_world.launch.py; every existing/validated launch omits it, byte-identical output confirmed) logging
max penetration depth and contact-point count over time. Requires the launch command to pass contact_diag:=true in addition to cargo_mode:=loose.
Usage: contact_diag_run.py <out.csv> <payload_gt.csv> <contact.csv> --from-top --no-rtf-gate
"""
import csv, subprocess, sys, threading, time
from pathlib import Path as _P

# cargo_descent_demo.py reads OUT_CSV/PAYLOAD_CSV/FROM_TOP/NO_RTF_GATE from sys.argv at IMPORT time (module-level code) - strip our extra
# positional arg (contact.csv) out of sys.argv BEFORE importing it, so its own parsing sees the same argv shape it expects.
CONTACT_CSV = sys.argv[3]
sys.argv = [sys.argv[0], sys.argv[1], sys.argv[2]] + sys.argv[4:]

sys.path.insert(0, str(_P(__file__).resolve().parent))
import cargo_descent_demo as C
D = C.D


def gz_pose_full(model_name):
    """Position + orientation quaternion for the named model, ground truth, same source topic as payload_gt_pose()."""
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{D.WORLD_NAME}/dynamic_pose/info', '-n', '1'],
                              capture_output=True, text=True, timeout=3).stdout
        i = out.find(f'name: "{model_name}"')
        if i < 0:
            return None
        blk = out[i:i + 500]
        p = blk.find('position {'); pb = blk[p:p + 150]
        o = blk.find('orientation {'); ob = blk[o:o + 150]
        def f(block, k):
            a = block.find(k + ':'); b = block.find('\n', a); return float(block[a + len(k) + 1:b].strip())
        pos = (f(pb, 'x'), f(pb, 'y'), f(pb, 'z'))
        quat = (f(ob, 'x'), f(ob, 'y'), f(ob, 'z'), f(ob, 'w'))
        return pos, quat
    except Exception:
        return None


def contact_reading():
    """Max penetration depth (m) and contact-point count from /payload/contact, one-shot poll."""
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', '/payload/contact', '-n', '1'], capture_output=True, text=True, timeout=2).stdout
        depths = [float(v) for v in [ln.split(':', 1)[1].strip() for ln in out.splitlines() if ln.strip().startswith('depth:')]]
        if not depths:
            return 0, None
        return len(depths), max(depths)
    except Exception:
        return None, None


class ContactDiagDemo(C.CargoDemo):
    def __init__(self):
        self.contact_log = []   # (t, n_contacts, max_depth_m)
        self._stop_contact_thread = False
        super().__init__()
        threading.Thread(target=self._contact_loop, daemon=True).start()

    def _payload_loop(self):
        while not self._stop_payload_thread:
            rp = gz_pose_full('ramp_robot'); pp = gz_pose_full('payload_box')
            self.payload_log.append((round(time.time() - self.t0, 2), rp, pp))
            time.sleep(0.2)

    def _contact_loop(self):
        while not self._stop_contact_thread:
            n, depth = contact_reading()
            self.contact_log.append((round(time.time() - self.t0, 2), n, depth))
            time.sleep(0.1)


def main():
    guard = D.hg.verify(fail_exit=False) if C.NO_RTF_GATE else D.hg.verify()
    if C.NO_RTF_GATE and not guard.get('ok'):
        print(f'!! NO_RTF_GATE: proceeding despite failed verify() - {guard}')
    monitor = D.hg.HealthMonitor(period=20.0).start()
    D.rclpy.init()
    node = ContactDiagDemo()
    try:
        node.wait_for_odom()
        node.sample(tag='start')
        if C.FROM_TOP:
            D.RAMP_APPROACH_X = D.RAMP_TOP_X + 0.5 - D.ELEV_MID_X
            D.RAMP_EXIT_X = D.RAMP_BOTTOM_X - 2.0 - D.ELEV_MID_X
            node.get_logger().info(f'spawned at the elevated level (odom_x={node.odom_x:.2f}); patched RAMP_APPROACH_X={D.RAMP_APPROACH_X:.2f} RAMP_EXIT_X={D.RAMP_EXIT_X:.2f}')
            node.ascent_verified = True
        ok = False
        if node.ascent_verified:
            node.get_logger().info('leg 3 (THE DEMO LEG): closed-loop FollowPath arc-turn (elevated level) -> taper onto ramp centreline -> DESCEND the ramp...')
            ok = node.follow_arc_and_ramp(sample_every=0.5)
            node.get_logger().info(f'leg 3 done: ok={ok} odom_x={node.odom_x:.2f} odom_y={node.odom_y:.2f} pitch={node.pitch:.2f} state={node.state}')
    finally:
        node.hard_stop()
        node.descent_active = False
        node._stop_payload_thread = True
        node._stop_contact_thread = True

    with open(C.OUT_CSV, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['t', 'tag', 'state', 'pitch', 'margin', 'slip', 'odom_x', 'odom_y', 'gz_x', 'gz_y', 'gz_z'])
        w.writeheader()
        for r in node.rows: w.writerow(r)
    with open(C.PAYLOAD_CSV, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'robot_x', 'robot_y', 'robot_z', 'robot_qx', 'robot_qy', 'robot_qz', 'robot_qw',
                     'payload_x', 'payload_y', 'payload_z', 'payload_qx', 'payload_qy', 'payload_qz', 'payload_qw'])
        for t, rp, pp in node.payload_log:
            row = [t]
            row += (list(rp[0]) + list(rp[1])) if rp else [None] * 7
            row += (list(pp[0]) + list(pp[1])) if pp else [None] * 7
            w.writerow(row)
    with open(CONTACT_CSV, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 'n_contacts', 'max_depth_m'])
        for row in node.contact_log: w.writerow(row)

    print('\n=== CONTACT DIAG RESULT ===')
    print(f'descent leg ok: {ok}')
    print(f'min stability_margin (descent): {node.descent_min_margin:.3f}  descent max |pitch|: {node.descent_max_abs_pitch:.2f} deg')
    print(f'descent HALT events: {node.descent_halt_count}')
    print(f'payload samples: {len(node.payload_log)} -> {C.PAYLOAD_CSV}')
    print(f'contact samples: {len(node.contact_log)} -> {CONTACT_CSV}')
    health = monitor.stop()
    print(f'HARNESS HEALTH: start={guard} during-run={health}')
    D.rclpy.shutdown()


if __name__ == '__main__':
    main()
