#!/usr/bin/env python3
"""
Unsecured-cargo physics DEMO (opt-in, isolated - see docs/construction_site_design.md "Unsecured cargo demo mode"). Reuses
construction_site_descent/arc_turn_followpath_test_ekf.py's Test class UNCHANGED (same ascent legs, same closed-loop arc-turn +
descent FollowPath goal, same ground-truth-vs-EKF physical gate, same HALT/margin/slip bookkeeping) - the ONLY thing this script
adds is logging the PAYLOAD's own ground-truth pose (relative to the chassis) throughout, which only means anything when
cargo_mode:=loose has made it a genuine free body (secured/default runs still log it fine, it just stays glued to the chassis).
Skips leg 4 (the flat return drive) - the demo is about the ramp legs, not the whole round trip.
Usage: cargo_descent_demo.py <out.csv> <payload_gt.csv>
"""
import csv, sys, threading, time
from pathlib import Path as _P
sys.path.insert(0, str(_P(__file__).resolve().parents[2] / 'construction_site_descent'))
import arc_turn_followpath_test_ekf as D   # Test, LAYDOWN_END_X, ELEV_MID_X, get_gz_pose, hg, rclpy

OUT_CSV, PAYLOAD_CSV = sys.argv[1], sys.argv[2]
FROM_TOP = '--from-top' in sys.argv[3:]   # spawn already at the elevated level (see record_cargo_demo.sh: spawn_x/y/z set accordingly) and run ONLY the
# demo leg (arc-turn + descent) - the requested comparison ('the loaded ramp descent... run twice back to back'). Skips legs 1/2 (gate -> laydown -> ascent);
# see cargo_ascent_finding.py / the design doc for the SEPARATE, honestly-documented finding about what happened when cargo_mode:=loose attempted the ascent.
NO_RTF_GATE = '--no-rtf-gate' in sys.argv[3:]   # GUI recording only (record_cargo_demo.sh): the live client + H.264 encode load routinely drags RTF under
# the headless-validation gate's MIN_RTF=0.95 on this host even on an otherwise-idle machine (measured ~0.66-0.69 while recording). gz-sim's physics step is
# fixed-size and driven by sim clock, not wall clock, so a low RTF here means the recording plays back closer to real speed than a normal run, not that the
# physics/contact results are wrong - so for this DEMO capability only, log the measurement instead of aborting on it. Every headless/validated invocation of
# this script (see ../cargo_demo/headless_test and the pending-tasks' secured/loose comparison) omits this flag and keeps the strict abort-on-fail gate.


def payload_gt_pose():
    try:
        import subprocess
        out = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{D.WORLD_NAME}/dynamic_pose/info', '-n', '1'], capture_output=True, text=True, timeout=3).stdout
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


class CargoDemo(D.Test):
    def __init__(self):
        super().__init__()
        self.payload_log = []   # (t, robot_gz, payload_gt) sampled at ~5 Hz throughout, independent of the base class's own sparser sampling
        self._stop_payload_thread = False
        threading.Thread(target=self._payload_loop, daemon=True).start()

    def _payload_loop(self):
        while not self._stop_payload_thread:
            rp = D.get_gz_pose(); pp = payload_gt_pose()
            self.payload_log.append((round(time.time() - self.t0, 2), rp, pp))
            time.sleep(0.2)


def main():
    guard = D.hg.verify(fail_exit=False) if NO_RTF_GATE else D.hg.verify()
    if NO_RTF_GATE and not guard.get('ok'):
        print(f'!! NO_RTF_GATE: proceeding despite failed verify() - {guard}')
    monitor = D.hg.HealthMonitor(period=20.0).start()
    D.rclpy.init()
    node = CargoDemo()
    try:
        node.wait_for_odom()
        node.sample(tag='start')

        if FROM_TOP:
            # Fresh spawn's odom origin is (0,0,0) wherever the launch file put it (spawn_x/y/z); the base class's RAMP_APPROACH_X/RAMP_EXIT_X are
            # calibrated to the STANDARD gate spawn's odom origin (world x=1.0). Re-express them relative to ELEV_MID_X (where they'd read if this were a
            # real post-ascent odom_x) so follow_arc_and_ramp()'s path-building - unmodified - lands in the same physical place either way.
            D.RAMP_APPROACH_X = D.RAMP_TOP_X + 0.5 - D.ELEV_MID_X   # -5.5
            D.RAMP_EXIT_X = D.RAMP_BOTTOM_X - 2.0 - D.ELEV_MID_X    # -20.6715
            node.get_logger().info(f'spawned at the elevated level (odom_x={node.odom_x:.2f}); patched RAMP_APPROACH_X={D.RAMP_APPROACH_X:.2f} RAMP_EXIT_X={D.RAMP_EXIT_X:.2f}')
            node.ascent_verified = True
        else:
            node.get_logger().info('leg 1: gate -> laydown area...')
            ok1 = node.drive_to(D.LAYDOWN_END_X, D.DRIVE_VX, leg_name='leg 1')
            node.get_logger().info(f'leg 1 done: odom_x={node.odom_x:.2f} state={node.state} physically_verified={ok1}')

            ok2 = False
            if ok1:
                node.get_logger().info('leg 2: laydown -> up the ramp -> onto elevated level...')
                ok2 = node.drive_to(D.ELEV_MID_X, D.DRIVE_VX, sample_every=1.0, leg_name='leg 2 (ramp ascent)')
                node.get_logger().info(f'leg 2 done: odom_x={node.odom_x:.2f} pitch={node.pitch:.2f} state={node.state} physically_verified={ok2}')
            node.ascent_verified = bool(ok1 and ok2)

        ok = False
        if node.ascent_verified:
            node.get_logger().info('leg 3 (THE DEMO LEG): closed-loop FollowPath arc-turn (elevated level) -> taper onto ramp centreline -> DESCEND the ramp...')
            ok = node.follow_arc_and_ramp(sample_every=0.5)
            node.get_logger().info(f'leg 3 done: ok={ok} odom_x={node.odom_x:.2f} odom_y={node.odom_y:.2f} pitch={node.pitch:.2f} state={node.state}')
        else:
            node.get_logger().error('ASCENT NOT PHYSICALLY VERIFIED - not attempting the descent leg')
    finally:
        node.hard_stop()
        node.descent_active = False
        node._stop_payload_thread = True

    with open(OUT_CSV, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['t', 'tag', 'state', 'pitch', 'margin', 'slip', 'odom_x', 'odom_y', 'gz_x', 'gz_y', 'gz_z'])
        w.writeheader()
        for r in node.rows: w.writerow(r)
    with open(PAYLOAD_CSV, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['t', 'robot_gz_x', 'robot_gz_y', 'robot_gz_z', 'payload_gz_x', 'payload_gz_y', 'payload_gz_z'])
        for t, rp, pp in node.payload_log:
            w.writerow([t] + (list(rp) if rp else [None, None, None]) + (list(pp) if pp else [None, None, None]))

    print('\n=== CARGO DEMO RESULT ===')
    print(f'ascent_verified: {node.ascent_verified}')
    for c in node.leg_checks: print(f'leg_check: {c}')
    print(f'descent leg ok (FollowPath + verified-progress gate): {ok}')
    print(f'total HALT events: {node.halt_count}  descent HALT events: {node.descent_halt_count}')
    print(f'min stability_margin (overall): {node.min_margin:.3f}  descent min margin: {node.descent_min_margin:.3f}')
    print(f'max wheel_slip (overall): {node.max_slip:.3f}  descent max slip: {node.descent_max_slip:.3f}')
    print(f'descent max |pitch|: {node.descent_max_abs_pitch:.2f} deg   descent max |ground-truth lateral Y|: {node.descent_max_abs_gz_y:.3f} m')
    print(f'payload samples logged: {len(node.payload_log)} -> {PAYLOAD_CSV}')
    health = monitor.stop()
    print(f'HARNESS HEALTH: start={guard} during-run={health}')
    D.rclpy.shutdown()


if __name__ == '__main__':
    main()
