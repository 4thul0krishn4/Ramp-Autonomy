#!/usr/bin/env python3
"""Ramp-2 traversal test (empty-payload robot, no delivery logic): gate -> laydown -> ramp 1 -> Floor 1 -> ramp 2 -> Floor 2, same driving style and same
ground-truth verification standard as ramp 1 (raw cmd_vel ascent legs; every leg physically verified against Gazebo ground truth; guard-checked host).
Reuses the round-trip driver's Test class (its drive_to / physical-check / sampling code) unchanged.
PASS = all four legs physically verified (GT vs EKF displacement <= 0.5 m), no abort/timeout, final ground-truth pose on Floor 2 (z >= 6.65 m, x within the slab, near the
target), and ground-truth lateral excursion on Floor 1 -> ramp 2 -> Floor 2 <= 1.0 m.   Usage: ramp2_traversal_test.py <out.csv>"""
import os
RAMP_WS = os.environ['RAMP_WS']
import csv, json, math, sys, time
sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8/construction_site_descent'))
import arc_turn_followpath_test_ekf as R          # module-level code only defines things; OUT_CSV is read from argv[1]
import rclpy

OUT = R.OUT_CSV
RAMP2_TARGET_ODOM_X = 75.5         # gz x = odom x + 1.0 (spawn at gz x = 1.0) -> gz x 76.5: 7.6 m onto Floor 2, before the delivery zone (x 78-81)
FLOOR1_BEFORE_RAMP2 = 54.0         # odom -> gz 55.0, 0.97 m before Floor 1's east edge / ramp-2 base (55.97)
F2_TOP, F2_X0, F2_X1 = 6.55, 68.94, 86.94


class T2(R.Test):
    def __init__(self):
        super().__init__()
        self.state_log = []

    def _on_state(self, msg):
        prev = self.state
        super()._on_state(msg)
        if msg.data != prev:
            self.state_log.append((round(time.time() - self.t0, 1), prev, msg.data, round(self.last_gz[2], 2) if self.last_gz else None, round(self.odom_x, 1)))


def main():
    guard = R.hg.verify()
    monitor = R.hg.HealthMonitor(period=20.0).start()
    rclpy.init(); node = T2(); ok = {}
    try:
        node.wait_for_odom(); node.sample(tag='start')
        legs = [('leg 1: gate -> laydown', R.LAYDOWN_END_X, 'leg 1'), ('leg 2: laydown -> ramp 1 -> Floor 1', R.ELEV_MID_X, 'leg 2 (ramp-1 ascent)'),
                ('leg 3: Floor 1 -> ramp-2 base', FLOOR1_BEFORE_RAMP2, 'leg 3 (Floor 1 traverse)'), ('leg 4: ramp 2 -> Floor 2', RAMP2_TARGET_ODOM_X, 'leg 4 (ramp-2 ascent)')]
        for text, tx, name in legs:
            node.get_logger().info(text + '...')
            t_leg = time.time() - node.t0
            ok[name] = node.drive_to(tx, R.DRIVE_VX, sample_every=1.0, leg_name=name)
            node.get_logger().info(f'{name} done: odom_x={node.odom_x:.2f} pitch={node.pitch:.2f} state={node.state} physically_verified={ok[name]}')
            if not ok[name] or node.aborted:
                node.get_logger().error(f'{name} NOT physically verified / aborted - stopping'); break
    finally:
        node.hard_stop()
    gz = node.last_gz
    rows = node.rows
    with open(OUT, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['t', 'tag', 'state', 'pitch', 'margin', 'slip', 'odom_x', 'odom_y', 'gz_x', 'gz_y', 'gz_z']); w.writeheader()
        for r in rows: w.writerow(r)
    # ground-truth lateral excursion from Floor 1 east portion onward (gz x >= 50) and z profile
    late = [abs(float(r['gz_y'])) for r in rows if r.get('gz_x') not in (None, '') and float(r['gz_x']) >= 50.0]
    maxy = max(late) if late else float('nan')
    fin_ok = gz is not None and gz[2] >= F2_TOP + 0.10 and F2_X0 <= gz[0] <= F2_X1 and abs(gz[0] - (RAMP2_TARGET_ODOM_X + 1.0)) <= 0.7
    reasons = []
    if not all(ok.get(n, False) for n in ('leg 1', 'leg 2 (ramp-1 ascent)', 'leg 3 (Floor 1 traverse)', 'leg 4 (ramp-2 ascent)')): reasons.append('a leg was not physically verified / not reached')
    if node.aborted: reasons.append('aborted')
    if not fin_ok: reasons.append(f'final ground-truth pose not on Floor 2 near target: {gz}')
    if not (maxy <= 1.0): reasons.append(f'ground-truth lateral excursion {maxy:.2f} m > 1.0 m')
    print('\n=== RAMP-2 TRAVERSAL RESULT ===')
    print(f'aborted: {node.aborted}')
    for c in node.leg_checks: print(f'leg_check: {c}')
    print(f'total HALT events: {node.halt_count}')
    print(f'min stability_margin observed: {node.min_margin:.3f}')
    print(f'max wheel_slip observed: {node.max_slip:.3f}')
    print(f'ground-truth max |y| from Floor 1 east part onward (gz x >= 50): {maxy:.3f} m')
    print(f'final gz pose: {gz}   (Floor 2 top z = {F2_TOP}; robot origin sits ~0.125 m above the surface)')
    print('state transitions (t, from, to, gz z, odom x):')
    for s in node.state_log: print('   ', s)
    print('RAMP2_RESULT:', 'PASS' if not reasons else 'FAIL - ' + '; '.join(reasons))
    print(f'wrote {len(rows)} rows to {OUT}')
    health = monitor.stop(); print(f'HARNESS HEALTH: start={guard} during-run={health}')
    if health['starved']:
        R.hg.banner('this run executed while the host was STARVED (min RTF %s, max load1 %s); its result is INVALID.' % (health['min_rtf'], health['max_load1']))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
