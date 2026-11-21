# LiDAR visual floating bug: root-caused and fixed (2026-10-02)

Both LiDAR sensor visuals (the forward ramp-profiling `lidar_link` and the centred 360° `obstacle_lidar_link`) appeared detached/floating above the
chassis in live GUI captures, confirmed from a screenshot showing both sensor meshes suspended clear of the real MiR100 body. Root-caused and fixed;
no joint, collision, inertial, or sensor-frame content changed - verified, not assumed.

## 1. Reproduce and instrument

Spawned the current robot (MiR100 mesh, loaded config, `detachable_payload:=true`) and queried both sensors' actual simulated pose two ways:

- `gz model -m ramp_robot -l`: `lidar_sensor` at (0.35, 0, 0.295) and `obstacle_lidar_sensor` at (0, 0, 0.62), both relative to `base_footprint` -
  exactly `base_height/2+0.05` and `0.5` above `base_link` respectively, i.e. exactly the URDF-specified fixed-joint offsets with zero deviation.
- Triggered a live payload detach (`ros2 topic pub /payload/detach std_msgs/msg/Empty`) and re-queried: **both sensor poses were bit-for-bit
  identical before and after.**

Both results are expected given how URDF/SDF handles a chain of `type="fixed"` joints: sdformat lumps them into their parent link at load time (that's
why `gz model -l` reports `lidar_sensor`/`obstacle_lidar_sensor` as children of `base_footprint`, not as separate dynamic links) - there is no runtime
degree of freedom for a fixed joint to drift on, and the detachable payload's own joint is a completely separate kinematic chain (`base_footprint` ->
`payload_box`) with zero coupling to the sensor mounts.

## 2. Root cause: (b), not (a) - confirmed, not assumed

**(a) TF/physics issue: ruled out by the evidence above.** The joints are correct and static; nothing about them moved, with or without a payload
event.

**(b) Visual/rendering mismatch: confirmed.** `lidar_joint`'s z (`base_height/2 + 0.05`) and `obstacle_lidar_joint`'s z (`0.5`, chosen to clear the
payload box's own top at 0.425) were both computed from `base_height/2` (0.125 m) - the OLD placeholder BOX's own half-height, back when the chassis
visual *was* a box. The real MiR100 mesh is a lower, rounded-top chassis that does not fill that same bounding box. Measured directly from
`mir_100_base.stl`'s own raw vertices (small neighbourhoods around each sensor's mount x/y, scaled and offset identically to how the chassis visual
itself is placed - `validation/phase8/lidar_mount_fix/mesh_query.py`):

| Sensor | Old assumed surface height (rel. base_link) | Real mesh surface height (rel. base_link) | Resulting float |
|---|---|---|---|
| `lidar_link` (forward) | 0.125 m | **0.095 m** | ~0.08 m gap below the drawn cylinder |
| `obstacle_lidar_link` (centre) | 0.125 m | **0.071 m** | ~0.43 m gap below the drawn cylinder |

## 3. The fix

`lidar_joint` and `obstacle_lidar_joint` themselves are **unchanged** - those joints also position each sensor's functional ray-casting frame (the
`<gazebo><sensor>` block is a sibling of `<visual>` in the same link), and phase 4's own measured accuracy numbers were validated against the
originally-specified mount height. Moving the joint would be a real, untested change to sensor geometry, not a cosmetic one - exactly the distinction
the task asked to preserve.

Instead, only each link's own `<visual>` geometry moved, via a local `<origin>` offset inside the link (independent of the link/joint/sensor frame):

- **`lidar_link`**: the drawn cylinder drops by `(base_height/2+0.05) - (mir_lidar_surface_z+0.05) = 0.030 m` to rest on the real mesh surface.
- **`obstacle_lidar_link`**: its sensor height (0.5 m) is a genuine functional requirement, not a visual guess - it has to clear the payload box's own
  top (0.425 m, present even when `payload_mass=0`) for an unobstructed 360° view, and lowering it would reopen the exact occlusion gap it was added
  to close. It keeps its own visual cylinder unchanged in place and gains a second, new, purely cosmetic mounting-mast visual spanning from the real
  chassis surface (0.071 m) up to just under the sensor body, so the required height has something visibly holding it up instead of floating in
  mid-air.

Both changes live in `src/ramp_autonomy_description/urdf/robot.urdf.xacro`, alongside the `mir_lidar_surface_z`/`mir_obstacle_lidar_surface_z`
property comment explaining the measurement and the joint-vs-visual distinction in full.

**A real near-miss caught before shipping the fix**: `robot.urdf.xacro`'s sensors are `type="gpu_lidar"`, which in gz-sim ray-casts against the
rendered scene, not collision geometry - new visual-only geometry placed in a sensor's own field of view genuinely can corrupt its scan. The first
version of this fix was checked for exactly that (section 4 below) before being treated as done; it came back clean, but it was checked, not assumed.

## 4. Verification

**Structural diff** (`validation/phase8/mesh_swap/diff_nonvisual.py`, all 4 `payload_mass`/`detachable_payload` combinations): every link, joint
(name/type/parent/child/origin/axis), collision block, inertial block, and `gz_frame_id` is byte-identical before and after; only `<visual>` content
differs (one repositioned cylinder, one new mast visual).

**Visual confirmation**, matched before/after camera angles, `shots/`:

| | Before | After |
|---|---|---|
| Forward LiDAR | `front_lidar_before.png` - visible gap between the sensor and the chassis hull | `front_lidar_after.png` - sits flush on the real surface |
| Obstacle LiDAR | `obstacle_lidar_before.png` - sensor floats clear above the payload with nothing connecting it | `obstacle_lidar_after.png` - a visible mast connects it down to the payload/chassis surface |

**Sensor data regression check** - same stationary pose (gate, `construction_site`), reverted-vs-fixed xacro, 5 samples each of `/ramp_perception`:

| | `breakpoint_distance_m` | `confidence` | `slope_deg` (5 samples) |
|---|---|---|---|
| Before | 0.028-0.031 m | 0.90-0.95 | -9.9, -2.8, 0.04, 0.3, 13.0 |
| After | 0.028-0.034 m | 0.90-0.94 | -7.8, -6.3, -1.4, 0.1, 6.4 |

Statistically indistinguishable - same mean breakpoint distance, same confidence band, same wide natural slope-noise pattern (expected: at this
near-zero breakpoint distance the onset-fit window is a tiny, inherently noisy cluster, a pre-existing property of being this close to real nearby
gate structure, present identically in both states). `/scan_360` sanity-checked post-fix: 82 of 360 valid returns, 3.2-11.9 m range, no self-detection
artifacts from the new mast.

**Full delivery mission rerun**, with the fix in place (`guarded_run.sh`, `detachable_payload:=true`, loaded): `ok: True`, all 6 legs `ok: True`
(ground-truth-vs-odometry disagreement -0.001 to -0.089 m, all inside the 0.5 m gate), payload released and landed on the dock
(`on_dock: True`), 20 HALTs. Host was moderately loaded during this run (mean RTF 0.86, below the harness's 0.90 clean-host purity bar, flagged
`STARVED` and not treated as a trusted quantitative baseline) - but the mission itself completed correctly end to end, which is what this check was
actually verifying. `mission_rerun/mission.json` has the full leg-by-leg record.

## Files

`mesh_query.py` (the STL surface-height measurement), `shots/` (the 4 before/after screenshots), `robot.urdf.xacro.fixed` (a saved copy of the fixed
file, used to cleanly toggle between before/after states for this verification without re-deriving the fix each time), `mission_rerun/` (the
post-fix delivery mission data).
