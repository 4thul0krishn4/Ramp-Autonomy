# Chassis mesh swap: cosmetic-only, verified structurally identical (MiR100, 2026-09-30)

Public-showcase pass: replace the robot's placeholder box/cylinder `<visual>` geometry with a real chassis mesh, with a hard constraint verified before AND
after - every `<link>` name, `<joint>` name/type/parent/child/origin/axis, `<collision>` block, `<inertial>` block, and `gz_frame_id` in the rendered URDF must
be byte-identical. Only `<visual>` elements changed.

**Supersedes an earlier Husky A200 mesh swap done in this same session.** That work is fully removed (meshes, attribution, docs) and replaced by this one -
see "Why MiR100, not the originally-requested MiR250" below for why the robot identity changed, not just the mesh source.

## 1. Mesh source

DFKI-NI's `mir_robot` (BSD 3-Clause, Copyright (c) 2018 DFKI GmbH), community-maintained ROS/ROS2 description of MiR's mobile robots, branch `rolling` @
commit `7d9c72942d512b191f2b326ca89c066710db4e34` (fetched 2026-09-30 via raw GitHub content; license confirmed via the repo's own `LICENSE` file). Files
taken (geometry only - STL carries no material/color data, unlike the previous Husky swap's `.dae` files):

| file | source path | used for |
|---|---|---|
| `mir_100_base.stl` | `mir_description/meshes/visual/mir_100_base.stl` | chassis (visual on `base_link`) |
| `wheel.stl` | `mir_description/meshes/visual/wheel.stl` | wheel (visual on each of the 4 `*_wheel_link`s) |

No URDF structure, xacro macros, control/actuation plugins, sensor mounts, or collision geometry was taken from `mir_robot` - only the two mesh files.
Installed under `ramp_autonomy_description/meshes/mir100/`, with `THIRD_PARTY_NOTICE.md` and the upstream `LICENSE-BSD-3-Clause-DFKI.txt` alongside them.
Since STL carries no embedded color, the real MiR100's own chassis/wheel colors are reproduced as two plain URDF `<material>` definitions in
`robot.urdf.xacro` (`mir_white` 1/1/1, `mir_dark_grey` 0.3/0.3/0.3 - matching MiR's own `material_white`/`material_dark_grey` conventions), not part of the
mesh files themselves.

### Why MiR100, not the originally-requested MiR250

The task asked for a MiR250 chassis. `DFKI-NI/mir_robot` (the source used for this swap) only publishes MiR100 meshes; no other established,
properly-licensed open-source MiR250-specific mesh package could be found (several personal repos exist but ship no `LICENSE` file, so they are not safely
redistributable - checked, not assumed). This was raised explicitly rather than silently substituted: the user was asked how to proceed and chose "use the
MiR100 meshes instead, labeled honestly" over alternatives (waiting on a MiR250 source, or keeping the Husky visuals). Every artifact from this swap - this
file, `THIRD_PARTY_NOTICE.md`, `robot.urdf.xacro`'s own comments, screenshots - says **MiR100**, never MiR250. Same manufacturer (Mobile Industrial Robots),
same general AMR chassis family, different specific model.

## 2. What changed in `robot.urdf.xacro` - visual only

- New properties: `mir_base_x/y/z` = 0.9 / 0.58 / 0.3 m (the source xacro's own `box_inertial` dimensions - MiR100's real chassis footprint/height),
  `mir_base_bottom_gap` = 0.05 m (`mir_100_base.stl`'s own local origin sits this far below the chassis box's bottom face, unlike the Husky meshes which
  were bottom-anchored at their own origin - derived from the source xacro's own `box_inertial` origin z=0.20, half-height 0.15 -> box bottom at local
  z=+0.05), `mir_wheel_r/w` = 0.0625 / 0.032 m (MiR100's real actuated-wheel radius/width).
- `base_link`'s `<visual>` (was: one grey box) -> one `<visual>` using `mir_100_base.stl`, `<origin xyz="0 0 ${-base_height/2 -
  mir_base_bottom_gap*(base_height/mir_base_z)}"/>` (shifts the mesh down by half this project's own collision-box height, plus the scaled bottom gap, so
  it sits flush with the collision box's bottom face) and `scale="${base_length/mir_base_x} ${base_width/mir_base_y} ${base_height/mir_base_z}"` - a
  per-axis scale from MiR100's own real dimensions down to this project's existing collision box (`base_length`/`base_width`/`base_height`: 0.8 / 0.6 /
  0.25 m), computed as xacro `${...}` expressions from the same properties the collision geometry already uses; rendered scale 0.8889 / 1.0345 / 0.8333.
  `material="mir_white"`.
- Each wheel's `<visual>` (was: a grey/black cylinder) -> `wheel.stl`, `<origin xyz="0 0 0" rpy="0 0 0"/>` (the mesh is already authored rolling about y,
  same as this project's own `axis xyz="0 1 0"` on every wheel joint - no extra rotation needed, unlike the Husky wheel mesh) and `scale="${wheel_radius/
  mir_wheel_r} ${wheel_width/mir_wheel_w} ${wheel_radius/mir_wheel_r}"` - this project's own `wheel_radius`/`wheel_width` (0.12 / 0.08 m) scaled against
  MiR100's real actuated-wheel dimensions; rendered scale 1.92 / 2.5 / 1.92 (x and z scaled equally to keep the wheel circular, y - the axle direction -
  scaled independently). `material="mir_dark_grey"`.
- `<collision>` and `<inertial>` blocks: untouched, byte-for-byte (see the verification below).
- `$(find ramp_autonomy_description)` (xacro's own package locator, resolved to an absolute installed path AT XACRO-RENDER TIME) is used for every mesh
  `filename`, not a `package://` URI - this project spawns the robot by publishing the xacro-rendered URDF text on a ROS topic (`robot_description`) rather
  than loading a file from disk into gz-sim, so there is no in-simulator `package://` resolution step to depend on.

## 3. Verification the swap is genuinely cosmetic

**Structural diff, at the rendered-URDF level** (`diff_nonvisual.py`; what the stack actually consumes, not the xacro source): rendered the xacro BEFORE
(Husky-visual state) and AFTER (MiR100-visual state) for all 4 argument combinations this project spawns with (`payload_mass` 0/100, `detachable_payload`
false/true), stripped every `<visual>` element, every XML comment, and every top-level `<material name="...">` color definition (all three carry zero
functional weight for any consumer that isn't rendering - confirmed by grepping every `<material>` occurrence in the rendered file and checking none appear
inside `<collision>`/`<inertial>`/`<joint>`/sensor/plugin content; the tool's sensitivity was re-verified after adding this pass with a deliberately renamed
joint, a deliberately resized collision box, and a bogus tag hidden right after `<collision>` - all still correctly flagged), and compared what's left
byte-for-byte.

**Result, all 4 combinations: byte-identical outside `<visual>`/comments/materials.** Every link name, every joint's (name, type, parent, child, origin,
axis) tuple, every `gz_frame_id`, every `<collision>` and `<inertial>` block, every Gazebo plugin/sensor block - unchanged.

**Full construction-site round trip, loaded, complete stack live** (EKF, stall detector, MPPI, safety gates - `arc_turn_followpath_test_ekf.py`, guarded,
headless): **PASS**, matches the established baseline exactly on the number that should never move if physics is untouched, and stays inside range on the
numbers that always vary run to run:

| metric | this run (MiR100 mesh) | established baseline |
|---|---|---|
| min stability_margin (overall) | **0.463** | **0.463** (bit-identical - this floor has read exactly 0.463 in every loaded run across every session of this project) |
| total HALT events | 9 (4 on the descent leg) | established range |
| leg 1 / leg 2 / leg 4 ground-truth-vs-EKF disagreement | 0.023 / 0.021 / -0.02 m, all `ok: True` | same pattern, same gate |
| descent max ground-truth lateral \|Y\| | 4.078 m | matches previously logged runs |
| final ground-truth pose | (1.99, 1.33) | inside the documented loaded leg-4 drift distribution |
| RTF (headless) | mean 0.932, min 0.915 | ~0.92-0.94 on this world |

Nothing about the physical behavior moved - exactly what a visual-only change should produce. Full data: `mir100_swap/round_trip/`.

## 4. Screenshots (`mir100_swap/shots/`, copied to `docs/images/mesh_*.png`)

Live GUI, loaded config, full stack: `gate_closeup` (stationary, chassis/wheel detail - white chassis, dark grey wheels), `ramp_climb` (mid-ascent on ramp
1, gz z=1.54), `elevated_site` (nearing Floor 1, robot visible with Floor 2's ramp-2/guardrail structure beyond). A fourth close-up angle
(`elevated_site_close`) was captured but discarded - the robot was out of frame. These files replace the previous Husky-mesh screenshots of the same names.

## Files

`diff_nonvisual.py` (the structural-identity checker, now also stripping `<material>` definitions), `before_*.urdf`/`after_*.urdf` (the 4 rendered
baselines - "before" = the prior Husky-visual state, "after" = MiR100), `mir100_swap/round_trip/` (the guarded regression check), `mir100_swap/shots/` (the
3 final PNGs used). Mesh files and attribution live in `src/ramp_autonomy_description/meshes/mir100/` (installed via the package, not here). The old
`before_*.err`/`after_*.err`/`shots_v1_flatground_mislabeled/`/`mesh_screenshot_tour.sh`/`simple_drive.py` files are leftover from the Husky session and no
longer represent the current mesh state; kept for the historical diff baseline only, not otherwise referenced.
