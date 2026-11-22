#!/usr/bin/env python3
"""Unit checks for ramp_autonomy_nav.path_source (no simulator needed)."""
import os
RAMP_WS = os.environ['RAMP_WS']
import math, sys
sys.path.insert(0, os.path.join(RAMP_WS, 'validation/phase8'))
import rclpy.time
from builtin_interfaces.msg import Time
from ramp_autonomy_nav import path_source as ps
import validation_test_ekf_groundtruth as V   # the historical make_path lives here (importing does not run main)

stamp = Time()
ok = True
def check(name, cond):
    global ok
    ok &= bool(cond); print(('PASS ' if cond else 'FAIL ') + name)

# 1. default spacing reproduces the historical path. The historical make_path() ACCUMULATES x += step, so
#    float drift can silently drop the final endpoint for some start values (e.g. x0=0.0 gives 15 points, not
#    16). The new generator multiplies instead. So: identical for the real start values, and where they differ the
#    historical path must be an exact prefix of the new one (only the dropped endpoint differs). The validation
#    script keeps calling the historical make_path() at the default spacing so baseline runs stay bit-identical.
for x0, yaw in ((-12.3, math.pi), (5.0, 0.0), (0.0, math.pi)):
    old = V.make_path(x0 - 0.5, x0 - 6.5, 0.4, yaw, stamp)
    new = ps.make_straight_path(x0 - 0.5, x0 - 6.5, yaw, stamp, spacing=0.4)
    m = min(len(old.poses), len(new.poses))
    maxd = max(abs(a.pose.position.x - b.pose.position.x) for a, b in zip(old.poses[:m], new.poses[:m]))
    check(f'default spacing matches historical make_path (x0={x0}, n={len(old.poses)}/{len(new.poses)}, max dx {maxd:.2e} m, old is a prefix of new)',
          maxd < 1e-6 and len(new.poses) >= len(old.poses) and len(new.poses) - len(old.poses) <= 1)
    print('   ', ps.path_stats(old), ps.path_stats(new))
# 2. dense straight path: correct count/length/lateral constraint
d = ps.make_straight_path(-12.8, -18.8, math.pi, stamp, spacing=0.05)
n, L, mx, mean = ps.path_stats(d)
check(f'0.05 m path: {n} pts, length {L:.3f} m, max spacing {mx:.4f}', abs(L - 6.0) < 1e-6 and mx <= 0.05 + 1e-6 and n == 121)
check('every waypoint pinned to y=0', all(abs(p.pose.position.y) < 1e-12 for p in d.poses))
check('endpoints identical to the 0.4 m path', abs(d.poses[0].pose.position.x + 12.8) < 1e-9 and abs(d.poses[-1].pose.position.x + 18.8) < 1e-6)
# 3. densify_path keeps originals and bounds spacing, incl. a curved path and yaw wrap
import importlib.util
spec = importlib.util.spec_from_file_location('dt', os.path.join(RAMP_WS, 'validation/phase8/construction_site_descent/crosstrack_diagnosis_flatmode/drift_characterization/drift_trials.py'))
try:
    dt = importlib.util.module_from_spec(spec); spec.loader.exec_module(dt)
    arc = dt.build_turn_and_ramp_path(0.0, 0.0, 2.0, -4.0, -12.0, stamp)
    dd = ps.densify_path(arc, 0.05)
    n0, L0, mx0, _ = ps.path_stats(arc); n1, L1, mx1, _ = ps.path_stats(dd)
    orig = {(round(p.pose.position.x, 9), round(p.pose.position.y, 9)) for p in dd.poses}
    kept = all((round(p.pose.position.x, 9), round(p.pose.position.y, 9)) in orig for p in arc.poses)
    check(f'construction-site arc path densified {n0}->{n1} pts, max spacing {mx0:.3f}->{mx1:.3f}, original waypoints kept', kept and mx1 <= 0.05 + 1e-6)
    check(f'polyline length preserved ({L0:.4f} vs {L1:.4f} m)', abs(L0 - L1) < 1e-6)
except Exception as e:
    check(f'construction-site path densify ({e})', False)
print('ALL PASS' if ok else 'SOME FAILED')
sys.exit(0 if ok else 1)
