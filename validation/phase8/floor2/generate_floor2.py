#!/usr/bin/env python3
"""Generate the Floor-2 extension of construction_site.sdf (reproducible; every dimension is a named constant with its source).

Writes the block between the markers
    <!-- FLOOR2:BEGIN --> ... <!-- FLOOR2:END -->
into src/ramp_autonomy_gazebo/worlds/construction_site.sdf (re-runnable) and edits ONLY the east perimeter fence (site extended east).
Everything belonging to the proven Floor-1 route (gate, access road, laydown, ramp 1, elevated_level) is left untouched.
Also writes validation/phase8/floor2/floor2_layout.json (every model's world-frame AABB) for the overlap / clearance checks.

Real-world grounding (see docs/construction_site_design.md, "Floor 2"): OSHA 1926.502 guardrails (top rail 42 in +/-3 = 1.07 m, midrail midway, toeboard >= 3.5 in),
1926.451 scaffolds (guardrail 38-45 in, toeboard 3.5 in, platform >= 18 in, access), 1926.1052 stairs (30-50 deg, landing >= 30 in every 12 ft rise, uniform risers),
1926.701(b) (protruding rebar guarded against impalement), 1926.706 (masonry walls > 8 ft braced), floor-to-floor 3.0-3.6 m, column starter-bar lap 40-50 bar diameters.
"""
import os
RAMP_WS = os.environ['RAMP_WS']
import json, math, re, sys

WORLD = os.path.join(RAMP_WS, 'src/ramp_autonomy_gazebo/worlds/construction_site.sdf')
LAYOUT = os.path.join(RAMP_WS, 'validation/phase8/floor2/floor2_layout.json')

# ------------------------------------------------------------------ geometry constants
F1_TOP, RISE, T = 3.35, 3.2, 0.1              # Floor 1 top (existing elevated_level), floor-to-floor rise (matches ramp 1), slab thickness
F2_TOP = F1_TOP + RISE                          # 6.55
DX = 27.97                                      # ramp-1 chain translated so its bottom seam lands on Floor 1's east edge (55.97 = 28.0 + 27.97)
F1_EAST = 55.97                                 # elevated_level east edge (48.47 + 7.5)
FLOOR2_X0, FLOOR2_X1 = 68.94, 86.94             # 40.97 + 27.97 = ramp-2 top seam; 18 m long
FLOOR2_Y = 8.0                                  # half-width (16 m wide)
EXT_Y = 10.0                                    # Floor-1 extension half-width (20 m): 2 m apron each side for the facade scaffold
GR_TOP, GR_MID, GR_TOE = 1.07, 0.535, 0.10      # OSHA 1926.502: top rail 42 in (1.07 m), midrail midway, toeboard >= 3.5 in (0.089 m)
POST_SP = 2.4                                   # <= 8 ft post spacing (App. B non-mandatory guideline)
LANE = 2.25                                     # ramp half-width; the robot lane on Floor 2 is |y| <= 2.75 and kept free of props

C = dict(conc=(0.60, 0.60, 0.62), conc_dk=(0.50, 0.50, 0.52), rebar=(0.42, 0.22, 0.12), cap=(1.0, 0.45, 0.0), ply=(0.80, 0.64, 0.42), steel=(0.32, 0.36, 0.46),
         yellow=(0.95, 0.78, 0.10), galv=(0.72, 0.74, 0.78), plank=(0.72, 0.55, 0.34), net=(0.15, 0.55, 0.20), cmu=(0.66, 0.66, 0.63), timber=(0.66, 0.50, 0.30),
         green=(0.10, 0.72, 0.25), slab=(0.62, 0.62, 0.62), ramp=(0.50, 0.47, 0.42), stair=(0.56, 0.56, 0.58))

models, layout, includes = [], [], []


def f(v):
    return f'{v:.4f}'.rstrip('0').rstrip('.') if abs(v) > 1e-9 else '0'


def aabb(pose, size):
    """conservative world AABB of a box (yaw about z in 90-degree steps, pitch about y arbitrary)"""
    x, y, z, r, p, w = pose
    sx, sy, sz = size
    if abs(math.sin(w)) > 0.7: sx, sy = sy, sx
    hx = abs(math.cos(p)) * sx / 2 + abs(math.sin(p)) * sz / 2
    hz = abs(math.sin(p)) * sx / 2 + abs(math.cos(p)) * sz / 2
    return (x - hx, y - sy / 2, z - hz, x + hx, y + sy / 2, z + hz)


class Model:
    def __init__(self, name, x=0.0, y=0.0, z=0.0, group='', pitch=0.0):
        self.name, self.pose, self.el, self.group, self.pitch = name, (x, y, z), [], group, pitch

    def box(self, size, pose, color, collision=True, mu=None, transp=None, label=None, visual=True):
        self.el.append(dict(size=size, pose=pose, color=color, collision=collision, mu=mu, transp=transp, label=label, visual=visual))
        return self

    def xml(self):
        x, y, z = self.pose
        s = [f'    <model name="{self.name}">', '      <static>true</static>', f'      <pose>{f(x)} {f(y)} {f(z)} 0 {f(self.pitch)} 0</pose>', '      <link name="link">']
        for i, e in enumerate(self.el):
            sx, sy, sz = e['size']; px, py, pz, pr, pp, pw = e['pose']
            pose = f'<pose>{f(px)} {f(py)} {f(pz)} {f(pr)} {f(pp)} {f(pw)}</pose>'
            geo = f'<geometry><box><size>{f(sx)} {f(sy)} {f(sz)}</size></box></geometry>'
            if e['collision']:
                fr = f'<surface><friction><ode><mu>{e["mu"]}</mu><mu2>{e["mu"]}</mu2></ode></friction></surface>' if e['mu'] else ''
                s.append(f'        <collision name="c{i}">{pose}{geo}{fr}</collision>')
            if not e['visual']:
                continue
            r, g, b = e['color']
            tr = f'<transparency>{e["transp"]}</transparency>' if e['transp'] is not None else ''
            s.append(f'        <visual name="v{i}">{pose}{geo}<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>{tr}</visual>')
        s += ['      </link>', '    </model>']
        return '\n'.join(s)

    def commit(self, in_layout=True):
        models.append(self.xml())
        if in_layout:
            X, Y, Z = self.pose
            boxes = []
            for e in self.el:
                px, py, pz, pr, pp, pw = e['pose']
                a = aabb((px + X, py + Y, pz + Z, pr, pp + self.pitch, pw), e['size'])
                boxes.append([round(v, 3) for v in a])
            layout.append(dict(name=self.name, group=self.group, boxes=boxes, collision=any(e['collision'] for e in self.el)))


def slab(name, x0, x1, y0, y1, top, color=C['slab'], mu=0.9, group='structure'):
    m = Model(name, (x0 + x1) / 2, (y0 + y1) / 2, top - T / 2, group)
    m.box((x1 - x0, y1 - y0, T), (0, 0, 0, 0, 0, 0), color, True, mu)
    m.commit(); return m


def include(uri_name, name, x, y, z, yaw=0.0, group='prop', dims=None, strip=(), link='link'):
    """Fuel include. `strip`: names of collision shapes removed via <experimental:params> (the visual mesh is untouched). Static-scenery collision cost is per shape (~2.5 us/step each),
    so multi-shape props (cinder block 5, large crate 25) are reduced to a few envelope shapes - see the performance note in docs/construction_site_design.md."""
    par = ''.join(f'\n        <collision element_id="{link}::{c}" action="remove"/>' for c in strip)
    par = f'\n      <experimental:params>{par}\n      </experimental:params>' if strip else ''
    includes.append(f'    <include>\n      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/{uri_name}</uri>\n      <name>{name}</name>\n      <static>true</static>\n'
                    f'      <pose>{f(x)} {f(y)} {f(z)} 0 0 {f(yaw)}</pose>{par}\n    </include>')
    sx, sy, sz = dims
    if abs(math.sin(yaw)) > 0.7: sx, sy = sy, sx
    layout.append(dict(name=name, group=group, boxes=[[round(x - sx / 2, 3), round(y - sy / 2, 3), round(z, 3), round(x + sx / 2, 3), round(y + sy / 2, 3), round(z + sz, 3)]], collision=True, fuel=uri_name))


# ------------------------------------------------------------------ Floor-1 extension + ramp 2 + Floor 2
slab('floor1_ext_slab', F1_EAST, FLOOR2_X1, -EXT_Y, EXT_Y, F1_TOP)

RAMP1 = [('fillet_bottom_1', 28.2987, 0.1279, -0.0931, 0.6), ('fillet_bottom_2', 28.8922, 0.2113, -0.1862, 0.6), ('fillet_bottom_3', 29.4754, 0.3495, -0.2793, 0.6),
         ('mid', 34.4735, 1.7827, -0.2793, 9.799), ('fillet_top_1', 39.4780, 3.1887, -0.1862, 0.6), ('fillet_top_2', 40.0715, 3.2721, -0.0931, 0.6), ('fillet_top_3', 40.6702, 3.3000, 0.0, 0.6)]
ramp2_profile = []                                                      # (x, z) of the top-face centreline, for rails / supports
for nm, x, z, p, L in RAMP1:
    m = Model('ramp2_' + nm, x + DX, 0, z + RISE, 'ramp2', pitch=p)
    m.box((L, 4.5, T), (0, 0, 0, 0, 0, 0), C['ramp'], True, 0.45)
    m.commit()
    xa, za = -L / 2 * math.cos(p) + (T / 2) * math.sin(p), L / 2 * math.sin(p) + (T / 2) * math.cos(p)   # top-face back edge (local -> world offsets)
    xb, zb = L / 2 * math.cos(p) + (T / 2) * math.sin(p), -L / 2 * math.sin(p) + (T / 2) * math.cos(p)
    ramp2_profile += [(x + DX + xa, z + RISE + za), (x + DX + xb, z + RISE + zb)]


def ramp_top_z(x):
    pts = sorted(ramp2_profile)
    for (xa, za), (xb, zb) in zip(pts[:-1], pts[1:]):
        if xa - 1e-6 <= x <= xb + 1e-6 and xb > xa + 1e-6:
            return za + (zb - za) * (x - xa) / (xb - xa)
    return pts[0][1] if x < pts[0][0] else pts[-1][1]


# Floor 2 slab: 4 boxes leaving the 4 x 4 m stair-core opening (x 79-83, y -7.4..-3.4); the whole robot lane (|y| <= 3.4) is ONE box (no seam in the lane)
CORE_X0, CORE_X1, CORE_Y0, CORE_Y1 = 79.0, 83.0, -7.4, -3.4
slab('floor2_slab_main', FLOOR2_X0, FLOOR2_X1, CORE_Y1, FLOOR2_Y, F2_TOP, group='floor2')
slab('floor2_slab_sw', FLOOR2_X0, CORE_X0, -FLOOR2_Y, CORE_Y1, F2_TOP, group='floor2')
slab('floor2_slab_se', CORE_X1, FLOOR2_X1, -FLOOR2_Y, CORE_Y1, F2_TOP, group='floor2')
slab('floor2_slab_s', CORE_X0, CORE_X1, -FLOOR2_Y, CORE_Y0, F2_TOP, group='floor2')

# ------------------------------------------------------------------ helpers: guardrails, rebar starters, scaffolds
def rail_run(name, pts, z0, group='edge', gaps=(), zfun=None, net=False, side=1, breaks=None):
    """Guardrail along a polyline of axis-aligned segments at surface height z0 (or zfun(x) for a sloped x-run). gaps: list of (a,b) intervals (in the run's own axis coordinate) left open.
    Performance-lean: posts every <= POST_SP, but the top rail / midrail / toeboard are ONE continuous member per straight run (per open interval), not one per bay.
    Sloped runs: rails are chords between the x-values in `breaks` (ramp element joints; chord deviation from the filleted surface < 6 mm). Posts are visual-only, mid/toe rails carry collision."""
    m = Model(name, 0, 0, 0, group)
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        horiz_x = abs(y1 - y0) < 1e-9
        L = abs(x1 - x0) if horiz_x else abs(y1 - y0)
        d = 1 if (x1 - x0 if horiz_x else y1 - y0) > 0 else -1
        a0 = x0 if horiz_x else y0                                          # run coordinate at t=0
        # open intervals (in t) after removing the gaps
        cuts = sorted(((g0 - a0) * d, (g1 - a0) * d) if d > 0 else ((g1 - a0) * d, (g0 - a0) * d) for g0, g1 in gaps)
        ivs, t = [], 0.0
        for g0, g1 in cuts:
            g0, g1 = max(g0, 0.0), min(g1, L)
            if g1 <= 0 or g0 >= L: continue
            if g0 > t + 1e-6: ivs.append((t, g0))
            t = max(t, g1)
        if t < L - 1e-6: ivs.append((t, L))
        def coord(t):
            return (x0 + (x1 - x0) * t / L, y0 + (y1 - y0) * t / L)
        for ta, tb in ivs:
            n = max(1, math.ceil((tb - ta) / POST_SP - 1e-9)); step = (tb - ta) / n
            for i in range(n + 1):                                             # posts
                x, y = coord(ta + i * step)
                zb = zfun(x) if zfun else z0
                m.box((0.06, 0.06, GR_TOP), (x, y, zb + GR_TOP / 2, 0, 0, 0), C['yellow'], False)
            xa, ya = coord(ta); xb, yb = coord(tb)
            if horiz_x and zfun:
                xs = sorted(set([xa, xb] + [q for q in (breaks or []) if min(xa, xb) + 1e-6 < q < max(xa, xb) - 1e-6]))
                for u, v in zip(xs[:-1], xs[1:]):
                    za, zb2 = zfun(u), zfun(v); pitch = -math.atan2(zb2 - za, v - u); ln = math.hypot(v - u, zb2 - za)
                    xc, zc = (u + v) / 2, (za + zb2) / 2
                    m.box((ln, 0.05, 0.05), (xc, ya, zc + GR_TOP - 0.025, 0, pitch, 0), C['yellow'], False)
                    m.box((ln, 0.05, 0.04), (xc, ya, zc + GR_MID, 0, pitch, 0), C['yellow'], False)
                    m.box((ln, 0.02, GR_TOE), (xc, ya, zc + GR_TOE / 2, 0, pitch, 0), C['timber'], True)
            else:
                xc, yc = (xa + xb) / 2, (ya + yb) / 2
                sx, sy = (abs(xb - xa), 0.05) if horiz_x else (0.05, abs(yb - ya))
                tsx, tsy = (abs(xb - xa), 0.02) if horiz_x else (0.02, abs(yb - ya))
                m.box((sx, sy, 0.05), (xc, yc, z0 + GR_TOP - 0.025, 0, 0, 0), C['yellow'], False)
                m.box((sx, sy, 0.04), (xc, yc, z0 + GR_MID, 0, 0, 0), C['yellow'], False)
                m.box((tsx, tsy, GR_TOE), (xc, yc, z0 + GR_TOE / 2, 0, 0, 0), C['timber'], True)
                if net:                                               # safety mesh: toeboard to top rail, vertical screen (1926.451(h) / 1926.105 style), visual only
                    nx, ny = (abs(xb - xa), 0.005) if horiz_x else (0.005, abs(yb - ya))
                    m.box((nx, ny, GR_TOP - GR_TOE), (xc + (0 if horiz_x else 0.03 * side), yc + (0.03 * side if horiz_x else 0), z0 + (GR_TOP + GR_TOE) / 2, 0, 0, 0), C['net'], False, None, 0.55)
    m.commit()


def starter_bars(name, cx, cy, ztop, group='rebar', kicker=True):
    """Column stub (kicker) + 4 corner starter bars D25, lap 36 d = 0.9 m exposed, each with an orange impalement cap (OSHA 1926.701(b))."""
    m = Model(name, cx, cy, 0, group)
    if kicker:
        m.box((0.4, 0.4, 0.3), (0, 0, ztop + 0.15, 0, 0, 0), C['conc_dk'], False)
    for k, (dx, dy) in enumerate([(-.15, -.15), (.15, -.15), (-.15, .15), (.15, .15)]):
        m.box((0.025, 0.025, 0.9), (dx, dy, ztop + 0.3 + 0.45, 0, 0, 0), C['rebar'], False)
        m.box((0.08, 0.08, 0.04), (dx, dy, ztop + 0.3 + 0.9 + 0.02, 0, 0, 0), C['cap'], False)
    m.commit()


def scaffold(name, x0, y0, z0, bays, depth, lifts, netting_side=0, group='scaffold'):
    """Frame scaffold on the slab at z0: bays of 2.44 m (8 ft plank), lifts of 1.83 m (6 ft frame), plank decks, guardrails 1.0 m (OSHA 38-45 in) + toeboard 0.10 m on the OUTER face
    (the inner face is the building side, gap to the work <= 14 in), one X-brace per bay, optional debris netting. Rails/decks are continuous across bays (lean scene: fewer elements)."""
    BAY, LIFT = 2.44, 1.83
    m = Model(name, 0, 0, 0, group)
    H = lifts * LIFT; W = bays * BAY
    out_y = y0 + depth if netting_side > 0 else y0                     # outer (guarded) face
    in_y = y0 if netting_side > 0 else y0 + depth
    for i in range(bays + 1):
        for yy in (y0, y0 + depth):
            m.box((0.05, 0.05, H + 1.0), (x0 + i * BAY, yy, z0 + (H + 1.0) / 2, 0, 0, 0), C['galv'], False)          # standards
    for L in range(1, lifts + 1):
        zd = z0 + L * LIFT
        m.box((W, depth, 0.045), (x0 + W / 2, y0 + depth / 2, zd, 0, 0, 0), C['plank'], False)                      # plank deck (continuous)
        m.box((W, 0.04, 0.04), (x0 + W / 2, out_y, zd + 1.0 - 0.02, 0, 0, 0), C['galv'], False)                       # top rail 1.0 m above deck
        m.box((W, 0.04, 0.04), (x0 + W / 2, out_y, zd + 0.5, 0, 0, 0), C['galv'], False)                              # midrail
        m.box((W, 0.02, 0.10), (x0 + W / 2, out_y, zd + 0.05, 0, 0, 0), C['timber'], False)                            # toeboard >= 0.089 m
    for i in range(bays):
        pitch = -math.atan2(lifts * LIFT, BAY) * (1 if i % 2 == 0 else -1)
        m.box((math.hypot(BAY, lifts * LIFT), 0.03, 0.03), (x0 + i * BAY + BAY / 2, out_y, z0 + H / 2, 0, pitch, 0), C['galv'], False)   # X-brace on the outer face, full height
    m.box((W, depth, 0.3), (x0 + W / 2, y0 + depth / 2, z0 + 0.15, 0, 0, 0), C['galv'], True, visual=False)          # base-frame envelope: the only collision (the robot never enters a scaffold)
    if netting_side:
        yy = y0 + depth + 0.04 if netting_side > 0 else y0 - 0.04
        m.box((W, 0.006, H + 1.0), (x0 + W / 2, yy, z0 + (H + 1.0) / 2, 0, 0, 0), C['net'], False, None, 0.55)          # debris netting (visual only)
    m.commit()


# ------------------------------------------------------------------ edge protection
# Floor 1 extension edges (north/south/east); Floor 2 edges with the ramp-2 opening (5.5 m) in the west edge; netting on Floor 2's north edge
rail_run('floor1_ext_guardrail_north', [(F1_EAST + 0.5, EXT_Y - 0.05), (FLOOR2_X1 - 0.05, EXT_Y - 0.05)], F1_TOP, 'edge')
rail_run('floor1_ext_guardrail_south', [(F1_EAST + 0.5, -EXT_Y + 0.05), (FLOOR2_X1 - 0.05, -EXT_Y + 0.05)], F1_TOP, 'edge')
rail_run('floor1_ext_guardrail_east', [(FLOOR2_X1 - 0.05, -EXT_Y + 0.05), (FLOOR2_X1 - 0.05, EXT_Y - 0.05)], F1_TOP, 'edge')
rail_run('floor2_guardrail_west', [(FLOOR2_X0 + 0.06, -FLOOR2_Y + 0.05), (FLOOR2_X0 + 0.06, FLOOR2_Y - 0.05)], F2_TOP, 'edge', gaps=[(-2.9, 2.9)])
rail_run('floor2_guardrail_north', [(FLOOR2_X0 + 0.06, FLOOR2_Y - 0.05), (FLOOR2_X1 - 0.05, FLOOR2_Y - 0.05)], F2_TOP, 'edge', net=True, side=1)
rail_run('floor2_guardrail_east', [(FLOOR2_X1 - 0.05, FLOOR2_Y - 0.05), (FLOOR2_X1 - 0.05, -FLOOR2_Y + 0.05)], F2_TOP, 'edge')
rail_run('floor2_guardrail_south', [(FLOOR2_X1 - 0.05, -FLOOR2_Y + 0.05), (FLOOR2_X0 + 0.06, -FLOOR2_Y + 0.05)], F2_TOP, 'edge')
for sgn, nm in ((1, 'north'), (-1, 'south')):                                           # ramp-2 edge protection: rails ON the ramp, 0.05 m inside its edge (open side up to 3.2 m above Floor 1)
    rail_run(f'ramp2_guardrail_{nm}', [(F1_EAST + 0.6, sgn * (LANE - 0.05)), (FLOOR2_X0 - 0.4, sgn * (LANE - 0.05))], 0.0, 'ramp2', zfun=ramp_top_z, breaks=[ramp2_profile[6][0], ramp2_profile[7][0]])

# ramp-2 supports (timber/steel bents standing on the Floor-1 extension): 4 bents, 2 posts + a transverse beam under the slab
for i, xb in enumerate((58.8, 61.9, 65.0, 67.8)):
    zt = ramp_top_z(xb) - 0.1 - 0.05
    m = Model(f'ramp2_support_{i + 1}', xb, 0, 0, 'ramp2')
    for yy in (-1.8, 1.8):
        m.box((0.2, 0.2, zt - F1_TOP), (0, yy, F1_TOP + (zt - F1_TOP) / 2, 0, 0, 0), C['conc_dk'], False)
    m.box((0.25, 4.0, 0.2), (0, 0, zt - 0.1 + 0.0, 0, 0, 0), C['conc_dk'], False)
    m.commit()

# ------------------------------------------------------------------ structural columns (6 grid columns, 6 m grid along the long edges, open plan between them) + starter bars above Floor 2
COLS = [(x, y) for x in (71.94, 77.94, 83.94) for y in (-7.4, 7.4)]
for k, (x, y) in enumerate(COLS):
    m = Model(f'floor2_column_{k + 1}', x, y, 0, 'structure')
    m.box((0.4, 0.4, RISE - T), (0, 0, F1_TOP + (RISE - T) / 2, 0, 0, 0), C['conc_dk'], True)
    m.commit()
    starter_bars(f'floor2_column_starters_{k + 1}', x, y, F2_TOP)

# ------------------------------------------------------------------ stair tower core (scenery only): walls + two 8-riser flights + landing, enclosed opening in Floor 2
RI, TR = 0.2, 0.28                                   # riser 200 mm (7.9 in <= 9.5 in), tread 280 mm (11 in) -> 35.5 deg, inside the 30-50 deg rule; 16 risers = 3.2 m
cw = 0.2
core = Model('stair_core_walls', 0, 0, 0, 'stair')
def wall(x0, x1, y0, y1, z0, z1):
    core.box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, 0, 0, 0), C['conc'], True)
TOP = F2_TOP + 2.4
wall(CORE_X0, CORE_X1, CORE_Y0, CORE_Y0 + cw, F1_TOP, TOP)                       # south wall (full)
wall(CORE_X0, CORE_X0 + cw, CORE_Y0 + cw, CORE_Y1 - cw, F1_TOP, TOP)             # west wall
wall(CORE_X1 - cw, CORE_X1, CORE_Y0 + cw, CORE_Y1 - cw, F1_TOP, TOP)             # east wall
# north wall: lower doorway (x 79.3-80.3, z 3.35-5.45 = 2.1 m high) and upper doorway (x 81.7-82.7, z 6.55-8.65)
n0, n1 = CORE_Y1 - cw, CORE_Y1
wall(CORE_X0, 79.3, n0, n1, F1_TOP, TOP); wall(80.3, 81.7, n0, n1, F1_TOP, TOP); wall(82.7, CORE_X1, n0, n1, F1_TOP, TOP)
wall(79.3, 80.3, n0, n1, F1_TOP + 2.1, F2_TOP); wall(79.3, 80.3, n0, n1, F2_TOP, TOP)                    # lintel over the lower door + solid above Floor 2
wall(81.7, 82.7, n0, n1, F1_TOP, F2_TOP); wall(81.7, 82.7, n0, n1, F2_TOP + 2.1, TOP)                     # sill under / lintel over the upper door
core.commit()
st = Model('stair_core_stairs', 0, 0, 0, 'stair')
yA = CORE_Y1 - cw - 0.1                                   # flight A starts at the lower door (north wall inner face - 0.1)
for k in range(1, 9):                                     # flight A: x 79.3-80.3, climbing toward -y, steps 1-8 (top at z = 3.35 + 0.2 k)
    ya, yb = yA - TR * k, yA - TR * (k - 1)
    st.box((1.0, yb - ya, RI * k), (79.8, (ya + yb) / 2, F1_TOP + RI * k / 2, 0, 0, 0), C['stair'], False)
zl = F1_TOP + RI * 8
land_y1, land_y0 = yA - TR * 8, CORE_Y0 + cw               # landing: from the last tread to the south wall
st.box((3.4, land_y1 - land_y0, zl - F1_TOP), (81.0, (land_y0 + land_y1) / 2, F1_TOP + (zl - F1_TOP) / 2, 0, 0, 0), C['stair'], True)      # landing (visual + collision)
for k in range(9, 17):                                    # flight B: x 81.7-82.7, climbing toward +y, steps 9-16
    ya, yb = land_y1 + TR * (k - 9), land_y1 + TR * (k - 8)
    st.box((1.0, yb - ya, RI * k), (82.2, (ya + yb) / 2, F1_TOP + RI * k / 2, 0, 0, 0), C['stair'], False)


def flight_collision(x, y0, z0, y1, z1):
    """one collision slab per flight, top face on the nosing line (scenery only: the robot never uses the stairs; 16 step-shaped collisions would cost more than the whole ramp chain)"""
    dy, dz = y1 - y0, z1 - z0; r = math.atan2(dz, dy); th = 0.3; ln = math.hypot(dy, dz)
    st.box((1.0, ln, th), (x, (y0 + y1) / 2 + th / 2 * math.sin(r), (z0 + z1) / 2 - th / 2 * math.cos(r), r, 0, 0), C['stair'], True, visual=False)


flight_collision(79.8, yA, F1_TOP, yA - TR * 8, F1_TOP + RI * 8)
flight_collision(82.2, land_y1, F1_TOP + RI * 8, land_y1 + TR * 8, F1_TOP + RI * 16)
st.commit()
rail = Model('stair_core_rails', 0, 0, 0, 'stair')


def sloped_rail(m, x, y0, z0, y1, z1, colr):
    """rail along the flight in the y-z plane from (y0,z0) to (y1,z1) at fixed x (roll about x)"""
    dy, dz = y1 - y0, z1 - z0
    roll = math.atan2(dz, dy) if dy > 0 else math.atan2(-dz, -dy)
    m.box((0.05, math.hypot(dy, dz), 0.05), (x, (y0 + y1) / 2, (z0 + z1) / 2, roll, 0, 0), colr, False)


# OSHA 1926.1052: stair rail >= 36 in (0.914 m): 0.95 m above the nosing line, on the open inner side of each flight
sloped_rail(rail, 80.3, yA, F1_TOP + 0.95, yA - TR * 8, F1_TOP + RI * 8 + 0.95, C['yellow'])
sloped_rail(rail, 81.7, land_y1, F1_TOP + RI * 8 + 0.95, land_y1 + TR * 8, F1_TOP + RI * 16 + 0.95, C['yellow'])
rail.commit()
for k, (x, y) in enumerate(((CORE_X0 + 0.25, CORE_Y1 - 0.25), (CORE_X1 - 0.25, CORE_Y1 - 0.25), (CORE_X0 + 0.25, CORE_Y0 + 0.25), (CORE_X1 - 0.25, CORE_Y0 + 0.25))):
    starter_bars(f'stair_core_starters_{k + 1}', x, y, TOP - 0.3, 'stair', kicker=False)     # bars project from the wall tops for the next lift

# ------------------------------------------------------------------ unfinished CMU infill wall between columns 2 and 3 on the north column line: 7 courses x 0.203 m = 1.42 m (< 2.44 m -> no bracing required, OSHA 1926.706), racked (stepped) east end
WALL_X0, WALL_X1, WALL_Y = 78.14, 83.74, 7.4
cmu = Model('cmu_infill_wall', 0, 0, 0, 'masonry')
for c in range(7):
    xe = WALL_X1 - 0.2175 * c
    cmu.box((xe - WALL_X0, 0.203, 0.203), ((WALL_X0 + xe) / 2, WALL_Y, F2_TOP + 0.203 * c + 0.1015, 0, 0, 0), C['cmu'], False)
xt = WALL_X1 - 0.2175 * 6
cmu.box((WALL_X1 - WALL_X0, 0.203, 0.203), ((WALL_X0 + WALL_X1) / 2, WALL_Y, F2_TOP + 0.1015, 0, 0, 0), C['cmu'], True, visual=False)     # collision: full-length bottom course
cmu.box((xt - WALL_X0, 0.203, 1.218), ((WALL_X0 + xt) / 2, WALL_Y, F2_TOP + 0.203 + 0.609, 0, 0, 0), C['cmu'], True, visual=False)          # + the full-height part of the racked wall
cmu.commit()
scaffold('mason_tower_scaffold', 80.0, 5.75, F2_TOP, 1, 1.2, 2, 0, 'scaffold')                # 1 bay x 2 lifts, standing on Floor 2, on the wall's working (south) side
scaffold('facade_scaffold_north', 72.0, 8.3, F1_TOP, 5, 1.2, 3, +1, 'scaffold')               # 5 bays x 3 lifts on the Floor-1 apron along Floor 2's north facade, netted on the outer face

# ------------------------------------------------------------------ formwork / rebar staging (authored: nothing suitable on Fuel)
def plywood_stack(name, x, y, yaw90=False):
    m = Model(name, x, y, 0, 'staging')
    sx, sy = (2.44, 1.22) if not yaw90 else (1.22, 2.44)
    for k, (dx, dy) in enumerate(((-0.9, 0), (0, 0), (0.9, 0)) if not yaw90 else ((0, -0.9), (0, 0), (0, 0.9))):
        m.box((0.089, 1.22, 0.089) if not yaw90 else (1.22, 0.089, 0.089), (dx, dy, F2_TOP + 0.0445, 0, 0, 0), C['timber'], False)        # 4x4 dunnage
    m.box((sx, sy, 0.36), (0, 0, F2_TOP + 0.089 + 0.18, 0, 0, 0), C['ply'], True)                                          # 20 sheets of 18 mm film-faced plywood, 4x8 ft
    m.commit()


def rebar_bundle(name, x, y, length=6.0):
    m = Model(name, x, y, 0, 'staging')
    for dx in (-(length / 2 - 0.4), 0, length / 2 - 0.4):
        m.box((0.089, 0.3, 0.089), (dx, 0, F2_TOP + 0.0445, 0, 0, 0), C['timber'], False)                                       # dunnage
    m.box((length, 0.26, 0.26), (0, 0, F2_TOP + 0.089 + 0.13, 0, 0, 0), C['rebar'], True)                                    # one bundle of D16 bars (collision = envelope)
    m.commit()


def steel_forms(name, x, y):
    m = Model(name, x, y, 0, 'staging')
    for k in range(4):                                                           # steel column-form panels 0.4 x 2.4 m leaning on a rail
        m.box((0.4, 0.04, 2.4), (0.45 * k - 0.7, 0.0, F2_TOP + 1.25, 0.12, 0, 0), C['steel'], False)
    m.box((1.9, 0.08, 0.08), (0, -0.25, F2_TOP + 0.04, 0, 0, 0), C['timber'], False)
    m.box((1.9, 0.5, 2.5), (0, 0, F2_TOP + 1.25, 0, 0, 0), C['steel'], True, visual=False)          # envelope collision of the leaning panels
    m.commit()


def timber_stack(name, x, y):
    m = Model(name, x, y, 0, 'staging')
    for r in range(3):
        m.box((1.22, 0.18, 0.089), (0, 0, F2_TOP + 0.0445 + 0.089 * r, 0, 0, 0), C['timber'], False)                    # course of four 2x4 x 4 ft bearers
    m.box((1.22, 0.18, 0.267), (0, 0, F2_TOP + 0.1335, 0, 0, 0), C['timber'], True, visual=False)       # envelope collision
    m.commit()


plywood_stack('formwork_plywood_stack_1', 75.2, 4.4)
plywood_stack('formwork_plywood_stack_2', 75.2, 6.0)
timber_stack('formwork_timber_stack_1', 73.3, 3.55)
rebar_bundle('rebar_bundle_1', 72.9, 6.85, 4.8)
steel_forms('steel_column_forms_1', 71.4, -4.6)
plywood_stack('formwork_plywood_stack_3', 74.2, -4.4)

# ------------------------------------------------------------------ Fuel-sourced props (real dimensions verified, see fuel_model_dimensions.txt)
DIMS = dict(pallet=(0.795, 1.2, 0.144), block=(0.435, 0.203, 0.203), jersey=(4.066, 0.813, 1.143), drum=(0.594, 0.594, 0.862), spool=(1.0, 1.0, 0.6), crate=(0.76, 0.53, 0.49))


envelopes = Model('floor2_prop_envelopes', 0, 0, 0, 'staging')       # one collision shape per stack / crate (the Fuel meshes keep their visuals, their multi-shape collisions are stripped)
CB = ('left', 'right', 'front', 'middle', 'back')


def block_pallet(tag, x, y, layers):
    include('Euro pallet', f'floor2_pallet_{tag}', x, y, F2_TOP, 0.0, 'staging', DIMS['pallet'])
    for L in range(layers):
        for ix, dx in enumerate((-0.203, 0.0, 0.203)):
            for iy, dy in enumerate((-0.2175, 0.2175)):
                include('Cinder Block', f'floor2_block_{tag}_{L}{ix}{iy}', x + dx, y + dy, F2_TOP + 0.144 + 0.203 * L, math.pi / 2, 'staging', DIMS['block'], strip=CB)
    envelopes.box((0.609, 0.87, 0.203 * layers), (x, y, F2_TOP + 0.144 + 0.203 * layers / 2, 0, 0, 0), C['cmu'], True, visual=False)


block_pallet('a', 70.7, 3.6, 3)
block_pallet('b', 72.0, 3.6, 3)
block_pallet('c', 80.4, 4.2, 2)                                          # mason's stock beside the tower scaffold
CRATE_STRIP = tuple(f'Box{i:03d}' for i in range(7, 32))
for k, (cx, cy) in enumerate(((76.6, -4.3), (76.6, -5.4))):
    include('Large Crate', f'floor2_crate_{k + 1}', cx, cy, F2_TOP, 0.0, 'staging', DIMS['crate'], strip=CRATE_STRIP)
    envelopes.box((0.76, 0.53, 0.49), (cx, cy, F2_TOP + 0.245, 0, 0, 0), C['timber'], True, visual=False)
include('55gal_Drum', 'floor2_drum_1', 77.0, 6.2, F2_TOP, 0.0, 'staging', DIMS['drum'], link='base_link')
include('55gal_Drum', 'floor2_drum_2', 77.0, 5.5, F2_TOP, 0.0, 'staging', DIMS['drum'], link='base_link')
include('Cable_Spool', 'floor2_cable_spool', 85.4, 6.2, F2_TOP, 0.0, 'staging', DIMS['spool'], strip=('spool_cable_low_collider_cylinder.001', 'spool_cable_low_collider_cylinder.002'), link='base_link')
include('Jersey Barrier', 'floor2_jersey_barrier_north', 69.75, 4.9, F2_TOP, math.pi / 2, 'barrier', DIMS['jersey'], strip=('base2', 'base3', 'left-angle', 'right-angle'))      # segregate the ramp-2 exit lane from the staging zone
include('Jersey Barrier', 'floor2_jersey_barrier_south', 69.75, -4.9, F2_TOP, math.pi / 2, 'barrier', DIMS['jersey'], strip=('base2', 'base3', 'left-angle', 'right-angle'))
envelopes.commit(in_layout=False)

# ------------------------------------------------------------------ DELIVERY / DROP-OFF POINT (marker only this session; attach/detach is the next session)
ZX, ZY, ZS = 79.5, 0.0, 3.0
dz = Model('floor2_delivery_zone', ZX, ZY, F2_TOP, 'delivery')
dz.box((ZS, ZS, 0.004), (0, 0, 0.002, 0, 0, 0), C['green'], False)                                      # painted 3 x 3 m pad (visual only, no collision: no step for the wheels)
for sx, sy, px, py in ((ZS, 0.15, 0, ZS / 2 - 0.075), (ZS, 0.15, 0, -ZS / 2 + 0.075), (0.15, ZS, ZS / 2 - 0.075, 0), (0.15, ZS, -ZS / 2 + 0.075, 0)):
    dz.box((sx, sy, 0.006), (px, py, 0.003, 0, 0, 0), C['yellow'], False)                                # yellow border
dz.commit()
mk = Model('floor2_delivery_marker', ZX - ZS / 2 - 0.35, ZY + ZS / 2 + 0.2, F2_TOP, 'delivery')
mk.box((0.08, 0.08, 2.2), (0, 0, 1.1, 0, 0, 0), C['yellow'], True)                                      # marker post, 2.2 m
mk.box((0.9, 0.03, 0.45), (0.45, 0, 1.95, 0, 0, 0), C['green'], False)                                   # green "DELIVERY" panel
mk.commit()

# ------------------------------------------------------------------ DELIVERY DOCK (phase 8, side-chute redesign): a low shelf beside the pad, positioned to catch the
# chute's own discharge trajectory, not a flush top-of-chassis transfer.
# robot.urdf.xacro's payload_chute_joint (-20 deg incline, final geometry after four rejected iterations - see that file's own chute comment for the full timeline:
# secured payload moved to y=0.28 because a 100 kg payload further out tips the whole robot over; tilt eased from -30 to -20 deg because the payload's own tilted
# corner, not the plate surface, was the real obstacle-LiDAR clearance driver) discharges the payload at its outboard edge: y=0.646, z=-0.068 rel. base_link, at a real
# measured exit speed of ~0.54 m/s along the slope. With the robot at rest its base_link origin is 0.12 m (wheel_radius) above the floor, so the discharge point is at
# world Z = F2_TOP + 0.12 - 0.068 = F2_TOP + 0.052, at world Y ~= robot_y+0.646 (~0.65 when the robot sits on the pad centreline). The dock top sits 20 mm below that
# (DECK_H=0.032): a short final drop off the chute's own edge onto the receiving shelf (not a flush hand-off like the old top-mount design, since the chute itself
# already does the sideways+downward work), with the shelf's own span (1.6 x 0.6 m, centred y=0.6) covering the discharge point plus margin for the payload's own exit
# momentum to carry it a bit further before friction (same mu=0.3 as the payload's own collision) stops it. Still clear of the wheels (outer face y=0.39) and the
# 360 deg obstacle LiDAR's scan plane (z=0.5 rel. base_link, far above this shelf).
DECK_H = 0.032
dock = Model('floor2_delivery_dock', ZX, 0.6, F2_TOP, 'delivery')
dock.box((1.6, 0.6, DECK_H), (0, 0, DECK_H / 2, 0, 0, 0), C['steel'], True, 0.3)      # mu 0.3: matches the payload's own collision friction (payload_box.sdf.xacro)
dock.box((1.6, 0.6, 0.01), (0, 0, DECK_H + 0.005, 0, 0, 0), C['yellow'], False)                          # yellow edge-marking plate on the top (visual only)
dock.commit()

# ------------------------------------------------------------------ write into the world file
HEADER = """    <!-- FLOOR2:BEGIN (generated by validation/phase8/floor2/generate_floor2.py - do not hand-edit; re-run the generator)
         FLOOR 2 EXTENSION (two-floor under-construction building). Floor 1 = elevated_level (top z 3.35). Floor-1 extension slab -> ramp 2 (ramp 1's validated
         7-element chain translated +27.97 m in x, +3.2 m in z; 16 deg max, 4.5 m wide, mu 0.45) -> Floor 2 (top z 6.55, x 68.94-86.94, y +/-8).
         Contents: edge protection (OSHA 1926.502/451), 6 grid columns with capped starter bars (1926.701(b)), scenery stair tower (1926.1052; the robot never uses it),
         racked CMU wall + tower scaffold, netted facade scaffold, formwork/rebar/block staging near the ramp-2 exit, Fuel props, and the DELIVERY / DROP-OFF POINT marker
         (floor2_delivery_zone / floor2_delivery_marker: marker only, no attach/detach logic).
         PERFORMANCE NOTE: static collision shapes cost ~2.5 us of sim step each, so scenery uses envelope collisions (thin rails collide only at the toeboard, stair flights are one
         pitched slab, Fuel multi-shape collisions are stripped via <experimental:params> and replaced by one envelope shape in floor2_prop_envelopes). Visual meshes are complete. -->"""
block = '\n'.join([HEADER] + models + includes + ['    <!-- FLOOR2:END -->'])
s = open(WORLD).read()
if '<!-- FLOOR2:BEGIN' in s:
    s = re.sub(r'    <!-- FLOOR2:BEGIN.*?<!-- FLOOR2:END -->', lambda m: block, s, flags=re.S)
else:
    s = s.replace('  </world>\n</sdf>', block + '\n  </world>\n</sdf>')
# east perimeter fence: the site is extended east by 31 m (Floor 1 extension + ramp 2 + Floor 2); the ONLY existing model edited
old_east = '<pose>59.150 0.000 1.200 0 0 0</pose>'
if old_east in s:
    s = s.replace(old_east, '<pose>90.150 0.000 1.200 0 0 0</pose>', 1)
    s = s.replace('    <model name="perimeter_east">', '    <!-- FLOOR 2 EXTENSION: perimeter_east moved from x=59.15 to x=90.15 (site extended 31 m east for Floor 2); north/south fences extended by\n         perimeter_north_ext / perimeter_south_ext at the end of this file. Same fence, same height (2.4 m); nothing on the Floor-1 route changed. -->\n    <model name="perimeter_east">', 1)
FENCE = """    <model name="perimeter_{d}_ext">
      <static>true</static>
      <pose>74.800 {y:.3f} 1.200 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>31.000 0.300 2.400</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>31.000 0.300 2.400</size></box></geometry>
          <material><ambient>0.75 0.72 0.68 1</ambient><diffuse>0.75 0.72 0.68 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
ext = ''.join(FENCE.format(d=d, y=sgn * 16.15) for d, sgn in (('north', 1), ('south', -1)))
if '<model name="perimeter_north_ext">' not in s:            # (the explanatory comment above also contains the name, so test for the model element)
    s = s.replace('    <!-- FLOOR2:BEGIN', ext + '    <!-- FLOOR2:BEGIN', 1)
if 'xmlns:experimental' not in s:
    s = s.replace('<sdf version="1.9">', '<sdf version="1.9" xmlns:experimental="http://sdformat.org/schemas/experimental">', 1)      # needed by the Fuel includes' <experimental:params> (collision stripping)
if 'gz-sim-apply-link-wrench-system' not in s:      # world-level actuator plugin - no longer used by the live mission (phase 8's side chute releases under gravity alone,
    # see robot.urdf.xacro's payload_chute_joint), kept because validation/phase8/delivery/{mechanism_test,rtf_state_run}.py and the historical payload_unloader.py
    # still use EntityWrench for their own earlier top-mount-era diagnostics
    s = s.replace('    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>\n', '    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>\n    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>\n', 1)
open(WORLD, 'w').write(s)
json.dump(dict(f2_top=F2_TOP, models=layout), open(LAYOUT, 'w'), indent=1)
print(f'wrote {len(models)} authored models + {len(includes)} Fuel includes into the world; layout -> {LAYOUT}')
