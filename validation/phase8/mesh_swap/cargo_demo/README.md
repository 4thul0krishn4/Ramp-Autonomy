# Unsecured cargo physics demo (`cargo_mode:=loose`) - opt-in, isolated from every validated result (2026-09-30)

Design-doc section: `docs/construction_site_design.md` "Unsecured cargo demo mode". This directory is the **only place in the whole project** that ever
passes `cargo_mode:=loose` - confirmed by grepping `src/` and `validation/` for `cargo_mode` (see that doc section's "Isolation, confirmed").
`validation_test_ekf_groundtruth.py`, every construction-site round-trip script, and every regression test never pass `cargo_mode` at all, so they all
keep the verified byte-identical `secured` default.

## 1. What `cargo_mode` adds

New xacro/launch arg `cargo_mode:=secured|loose`, default `secured`:

- **`secured`**: `payload_link` fixed-jointed to `base_link`, exactly today's behavior. Rendered URDF verified byte-for-byte identical (all 4
  `payload_mass`/`detachable_payload` combinations) whether `cargo_mode` is omitted or passed explicitly as `secured`.
- **`loose`**: the payload is a genuine separate free rigid body (`payload_box`, spawned the same way as the existing detachable-payload mechanism),
  resting on the chassis under gravity/contact friction, never joint-constrained - `separate_payload = detachable or loose` gates `payload_link` the
  same way `detachable_payload` alone always has; `loose` also suppresses the DetachableJoint plugin even if `detachable_payload:=true` is passed too
  (there is nothing to detach - it started free).

Implementation: `src/ramp_autonomy_description/urdf/robot.urdf.xacro` (the `cargo_mode` arg + `loose`/`separate_payload` properties),
`src/ramp_autonomy_gazebo/launch/ramp_world.launch.py` and `src/ramp_autonomy_bringup/launch/ramp_autonomy_bringup.launch.py` (both pass `cargo_mode`
through, default `secured`).

## 2. The demo: loaded ramp descent, secured vs. loose, same spawn/path/conditions

`record_cargo_demo.sh <secured|loose> <out_dir>` - live GUI, H.264 screen capture (`gui_recordings/{secured,loose}/cargo_{secured,loose}_raw.mp4`), both
runs spawned at the identical point at the top of the elevated platform (`spawn_x=47.9702 spawn_y=0.0 spawn_z=3.48`, `--from-top` mode in
`cargo_descent_demo.py`) and driving the identical demo leg: closed-loop FollowPath arc-turn on the flat elevated deck -> taper onto the ramp centreline
-> descend ramp 1, loaded (100 kg). Isolates the descent leg (the "most dynamically interesting leg") rather than risking the recording ending during an
earlier ascent HALT-storm (see section 4 - loose cargo can already destabilize the robot during ascent from the gate, a separate finding).

`cargo_descent_demo.py` reuses `construction_site_descent/arc_turn_followpath_test_ekf.py`'s `Test` class **unchanged** (same closed-loop FollowPath
goal, same ground-truth-vs-EKF gate, same HALT/margin/slip bookkeeping) and adds one thing: a background thread polling the payload's own ground-truth
world pose (`gz topic -e -t /world/construction_site/dynamic_pose/info`) at ~5 Hz throughout, logged relative to the chassis's own ground-truth pose.

### Result

| | secured | loose |
|---|---|---|
| descent leg ok (FollowPath + verified-progress gate) | **True** | **False - SAFETY ABORT** |
| min stability_margin (descent) | 0.463 | **0.097** |
| max \|pitch\| (descent) | 16.00 deg (= the ramp's own geometric incline, exactly) | **33.40 deg** |
| max wheel_slip (descent) | 5.3-10.7 | **28.0** |
| HALT events | 4-7 | 8 (run ends in an aborted goal) |
| payload ground-truth samples | **0 - no separate body exists to poll.** `payload_box` is never spawned in `secured` mode; the payload is structurally part of the single chassis rigid body, not merely "measured to stay still". | 344 samples over 156.4 s |

Both `secured` runs (one headless, one this GUI recording) landed on the *same* min-margin/max-pitch numbers as every other validated loaded run in this
project (0.463 / 16.00 deg) - the recording did not change the underlying physics.

### Loose cargo's actual relative-offset trajectory (`payload_gt.csv`, `gz_z` offset starts at +0.395 m)

- **t = 0-128 s (the whole flat-platform arc-turn):** essentially fixed - a mild rock within rel_x +0.10 to -0.13 m, rel_y -0.08 to +0.12 m as
  centripetal force changes direction through the turn, but **rel_z locked at exactly +0.395 m for the entire 128 seconds.** The turn alone does not
  dislodge the cargo.
- **t = 128-142 s (taper onto ramp, near-flat):** rel_x drifts to -0.16; rel_z barely moves (+0.395 -> +0.386).
- **t = 142-150 s (real descent, chassis pitch exactly 16.00 deg, margin steady 0.463 - matching the secured run at this same point):** slow slide
  begins, rel_x -0.16 to -0.24, rel_z +0.386 to +0.34.
- **t = 150-156 s (final ~6 s):** sudden, accelerating slide - rel_x -0.244 -> **-0.483 m**, rel_z +0.340 -> **+0.178 m**. This is the exact window in
  which the chassis's own logged pitch jumps from a steady 16.00 deg to 31.31 deg and margin collapses from 0.463 to 0.147, ending in
  `SAFETY ABORT: margin=0.147 pitch=31.3 - cancelling FollowPath goal`. Total relative displacement: **~0.62 m**, almost all of it in the final 6
  seconds.

**Reading (observed correlation - the demo shows what happened, this is not an independent first-principles derivation)**: the payload rides essentially
motionless through the turn and the first ~8 s of descent - matching the secured run almost exactly up to that point - then, once the slope component
of gravity overcomes static friction, it slides backward (toward the uphill end of the chassis deck) and down in an accelerating ~6-second event, right
as the chassis's own pitch shoots past the ramp's 16 deg geometric incline to over 33 deg and stability margin collapses, ending in the safety stack's
own abort. A shifting rear-loaded mass making the chassis pitch further nose-down than the ramp geometry alone would explain is consistent with what
was logged; not independently verified beyond that correlation.

## 2b. Verified: genuine rigid-body dynamics, not a contact/numerical artifact (2026-09-30)

Before using the clip publicly, the dramatic result above (pitch 16.00 -> 31.31 deg while still nominally on a 16 deg ramp, margin collapse to 0.147,
and the section-4 ascent run's margin-to-0.000) was checked for three failure modes: contact tunneling/interpenetration, a coordinate-frame illusion
(apparent "sliding" that's actually just the chassis rotating under a payload that never moved locally), and solver/numerical instability. **All three
checks came back clean - this is genuine dynamics.**

**Method**: added an opt-in, zero-impact `contact_diag:=true` launch arg (`payload_box.sdf.xacro`/`ramp_world.launch.py` - a `<sensor type="contact">`
on the payload's existing collision geometry; confirmed byte-identical rendered SDF when omitted, so every validated call site including the delivery
session's `detachable_payload:=true` - which spawns the same file - is untouched) and re-ran the identical secured-vs-loose descent scenario headless
with full orientation (quaternion) logging for both bodies plus live contact-penetration-depth polling (`contact_diag_run.py`,
`contact_diag/diag1_{payload,contact}.csv`). This is a NEW diagnostic run, not a re-analysis of the original GUI clip (which wasn't instrumented for
contact depth) - it reproduced the same event (SAFETY ABORT at margin=0.148 pitch=31.3, vs. the GUI clip's margin=0.147 pitch=31.3 - matching almost
exactly), which is itself evidence against a one-off glitch.

**(a) Contact penetration depth - real, not tunneling.** Across the entire run, including through the event, max penetration depth was **0.0005 mm at
rest, peaking at 0.0191 mm during the event** (about 1/15,000th of the box's own height) - negligible, consistent with ODE's normal constraint-slop, not
tunneling. Contact point count stayed at **4 (all four corners) for the entire run**, right up to the last sample before the abort - the payload never
lost contact outright, it kept touching throughout, just increasingly off-center.

**(b) Position relative to the chassis footprint - a real slide past the support edge, correcting for chassis rotation.** The original GUI clip's
world-frame offsets are confounded by the chassis's own pitch (a payload sitting at a perfectly FIXED point on a tilting chassis still traces a
changing world-frame offset, purely from rotation - checked for exactly this before trusting the raw numbers). Un-rotating by the chassis's own
ground-truth orientation (quaternion, this diagnostic run) into the chassis's local frame:

  - **t = 0-88 s (flat-platform arc-turn, pitch~0 so local = world, no rotation confound possible):** local offset oscillates genuinely within about
    +0.12 to -0.15 m (turn-induced rocking, not a coordinate artifact) - bounded, not runaway.
  - **t = 88-99 s (pitch climbs 0 -> 16 deg, the ramp's own nominal incline):** local_x grows smoothly from -0.15 to -0.38 m, local_z drops smoothly
    from the resting +0.395 to +0.23 m - a slow, steady slide while the chassis itself still tracks the ramp's own geometry exactly (margin 0.463,
    matching the secured run).
  - **The chassis's own half-length is 0.4 m and the payload's own half-length is 0.25 m, so the payload's trailing edge reaches the chassis's
    trailing edge once local_x <= -0.15 m** - crossed smoothly around t=88-90 s, while pitch was still near-nominal and margin was still 0.463. From
    that point the payload is genuinely overhanging, not merely "measured to be far away".
  - **t = 99.2-100.0 s (final <1 s):** local_x -0.38 -> **-0.52 m**, local_z +0.23 -> **-0.13 m** (crossing below the chassis's own origin height) -
    exactly the geometric signature of a box tipping/rotating around an edge it has already overhung, not a discontinuous jump: every intermediate
    sample lies smoothly on the curve, nothing teleports.
  - **Cross-check**: this ground-truth-quaternion-derived pitch (16.00 deg steady, 31.03 deg at the end) matches the robot's own independent
    IMU-derived pitch log (`contact_diag/diag1.csv`: 16.00 deg steady, 31.3 deg at the abort) to within a fraction of a degree and a fraction of a
    second - two independent measurement paths agreeing rules out a parsing/sign error in either.

**(c) Solver/numerical behavior - a smooth buildup, not an instability.** The world's physics step is fixed at 1 ms (ODE, sim-clock-driven - confirmed
unaffected by the GUI recording's degraded wall-clock RTF, which only changes how much wall-clock time each fixed sim-step takes, not the step itself)
with moderate payload friction (mu=0.3, not an extreme/destabilizing value). The margin collapse itself is smooth and monotonic over ~0.85 s
(0.463 -> 0.428 -> 0.148 in the diagnostic run's own log), not a discontinuous spike or an oscillating/diverging value - the signature of a continuous
torque buildup as the payload's lever arm grows, not a solver breakdown.

**One caveat, disclosed rather than hidden**: the ORIGINAL GUI clip's payload log has one single isolated outlier sample (t=150.13 s, reverts cleanly
on the very next sample 0.45 s later) that does not fit the otherwise-smooth curve. The clip's own two position reads (robot, then payload) are two
SEPARATE, non-atomic `gz topic` subprocess calls, and host load during that GUI+encoding recording was heavy (load1 8-10+); this one-sample glitch is
best explained as read-timing skew in this diagnostic's OWN polling method under load, not a simulator artifact - the fresh headless diagnostic run
(much lower host load) reproduced the same overall event with no comparable outlier.

**Verdict: genuine dynamics, safe to use publicly as recorded.** The event is reproducible (two independent runs land on nearly identical numbers),
contact stays physically realistic throughout (sub-0.02 mm depth, contact never lost), the "slide" is real sliding past a real geometric support
threshold (not a rotation-frame illusion), two independent pitch measurements agree, and the margin collapse is smooth rather than glitchy.

**Lighter pass on the separate ascent-run's margin-to-0.000 finding (section 4, existing data, no re-run needed)**: the payload's relative trajectory
around that event (`headless_test/loose_headless_payload.csv`, t=129-136 s) is smooth and monotonically accelerating (each consecutive ~0.4 s sample's
displacement is consistent with gravity-driven acceleration, e.g. the single largest step, 0.56 m over 0.43 s, is the right order of magnitude for
free fall over that interval) - no teleports. The robot's OWN ground-truth trajectory over the same window is equally smooth: it visibly slides
backward down the ramp (z: 3.16 -> 2.75 m, x: 38.06 -> 36.78 m) before coming to rest and freezing - a real loss-of-traction event, not a glitch. This
also fully explains the run's reported 41 m EKF-vs-ground-truth disagreement: the EKF assumed forward progress from commanded wheel motion while the
robot actually slid backward under the shifting load - consistent with this project's own already-documented "`/odom` drift under heavy wheel slip"
limitation (see the main README's "Known gaps"), not a new issue.

## 3. Analysis reproduction

```
cd validation/phase8/mesh_swap/cargo_demo
python3 - <<'PY'
import csv
def load(path):
    rows = list(csv.reader(open(path)))[1:]
    rows = [r for r in rows if all(x != '' for x in r)]
    return [(float(r[0]), float(r[4])-float(r[1]), float(r[5])-float(r[2]), float(r[6])-float(r[3])) for r in rows]
for t, dx, dy, dz in load('gui_recordings/loose/data/payload_gt.csv'):
    print(t, dx, dy, dz)
PY
```

## 4. A second, separate, earlier finding (full gate-to-descent, headless only - not this GUI-recorded pair)

`headless_test/loose_headless*` - a full ascent-from-gate attempt with `cargo_mode:=loose`: unsecured cargo destabilized the robot **during the
ascent**, before the descent leg was ever reached. A sudden ~1.35 m relative backward slide drove stability_margin to its absolute floor (0.000),
triggered 29 HALT events, and the robot never recovered for the rest of a 500 s run (the payload settled far below/behind the frozen chassis). Also
isolated, also not part of any validated result - documented here for completeness since it's the same mechanism (unsecured mass sliding backward
relative to the chassis on an incline), just a different, more severe scenario than the GUI-recorded descent-only comparison above.

`headless_test/loose_fromtop3*` - an earlier, truncated from-top descent-only headless run (data cut short mid-slide at t=91.78 s) showed the same
qualitative pattern (flat-platform stability, then descent-triggered sliding) with less-complete data; superseded by the full GUI-recorded run in
section 2, which captured the complete slide-to-abort event.

## 5. Why `--no-rtf-gate`

The live GUI client alone (before any video encoding) measures RTF ~0.66-0.70 on this host; encoding drops it further to ~0.5-0.55. This is well below
the headless-validation harness's strict `MIN_RTF=0.95` gate (`harness_guard.py`), but a low RTF only means wall-clock time lags sim-clock time -
gz-sim's physics step is fixed-size and sim-clock-driven, so it doesn't affect the physics/contact results themselves. `cargo_descent_demo.py` accepts
`--no-rtf-gate` (GUI recording only, via `record_cargo_demo.sh`; never used by any headless/validated invocation of the script) to log this measurement
rather than abort the recording on it. The GUI-recorded secured run's own numbers (margin 0.463, pitch exactly 16.00 deg, matching the strict-gated
headless secured baseline) suggest the reduced RTF did not distort the physics.

## Files

- `cargo_descent_demo.py` - the demo driver (subclasses `arc_turn_followpath_test_ekf.Test`, adds payload ground-truth polling + `--from-top`/
  `--no-rtf-gate` flags).
- `record_cargo_demo.sh` - GUI-recorded run (secured or loose), the script referenced in section 2 above.
- `guarded_run.sh` - headless guarded run (used for `headless_test/`).
- `headless_test/` - headless validation data: `secured_fromtop*` (this comparison's secured baseline, headless), `loose_fromtop3*` (truncated
  descent-only loose, superseded by the GUI recording), `loose_headless*` (the separate full-ascent finding, section 4).
- `gui_recordings/{secured,loose}/` - the actual GUI-recorded demo pair: `cargo_{secured,loose}_raw.mp4` (video) + `data/` (CSVs, logs, health).
- `contact_diag_run.py`, `contact_diag/guarded_diag_run.sh`, `contact_diag/diag1_{payload,contact}.csv` - the genuine-dynamics-vs-artifact verification
  (section 2b): orientation quaternions for both bodies + live contact-penetration-depth telemetry from a `contact_diag:=true` instrumented re-run.
