# Crest freeze diagnosis and fix (2026-09-20) - the loaded-robot "cannot clear the crest" gap was a 0.05 m geometry authoring error

**Verdict: the same bug class as the construction-site freeze (a box-center-vs-top-surface authoring step at a slab join), in different geometry - `ramp_world_*.sdf`, which was never fixed when `construction_site.sdf` was.**
Fixing it, the loaded robot completes the ascent AND the descent in **60 of 60** guarded reverification runs (four loaded cells x 12, two unloaded regression cells x 6; ground-truth verified); the loaded-ascent completion rate is **48 / 48
(Wilson 95% 93-100%)** versus 0 / 10 (curved 0.45), 6 / 8 (curved normal) and 0 / 8 per cell (sharp loaded, every recorded run) before. Pre-registration: `PREREGISTRATION.md`.

## 1. Same bug class or different mechanism? (step 1)
Exact world-frame corner math (`seam_geometry.py`, output in the pre-registration and `data/`): the top-face height at every seam between consecutive ramp slabs, in every ramp world file. All seams match within 1.4 mm
(the same tolerance the fixed construction site shows: <= 1.1 mm) EXCEPT two per world:

| World | seam | step (ORIGINAL) | after fix |
|---|---|---|---|
| curved (and curved_mu045) | `fillet_top_3` -> `zone_c_exit` (the crest -> platform join) | **+0.0500 m** | +0.0000 |
| curved (and curved_mu045) | `zone_a_approach` -> `fillet_bottom_1` (the ramp foot) | **-0.0502 m** | -0.0002 |
| sharp (and its 4 TESTONLY variants) | `zone_b_ramp` -> `zone_c_exit` | **+0.0502 m** | -0.0018 |
| sharp (and variants) | `zone_a_approach` -> `zone_b_ramp` | **-0.0537 m** | -0.0017 |

Cause: the ramp fillets/slab were authored with heights that treat 0.05 (the centre of the 0.1 m thick `zone_a` slab) and 1.05 (the centre of `zone_c_exit`) as if they were the SURFACES (0.10 / 1.10): the ramp is 0.05 m too low. Independent evidence for
the design intent: the sharp world's visual breakpoint markers sit at z 0.15 and 1.15 (surfaces 0.10 / 1.10 plus 0.05).

Wheel-level instrumentation of a fresh repro (`crest_diag.py`: per-wheel joint velocity, `/wheel_slip`, the detector's signals, full-orientation ground truth; `analyze_freeze.py`; `before_freeze_analysis.txt`), curved / 0.45 / loaded, two runs:
* The chassis froze at the **identical pose in both runs: gz x = 8.018 m, z = 1.157 m, pitch -3.6 deg**, for 40-50 s.
* All four wheels turn (mean surface speed 0.089-0.095 m/s) while the chassis moves 0.0003 m/s. `/wheel_slip` mean 0.012 (max 4.4 in transients): **blind**, because it compares wheel to command, never chassis to wheel.
* The EKF x ran **+4.5 m / +3.8 m** ahead of ground truth in the window (`/odometry/filtered` vx 0.09 m/s) - the "EKF runs away" signature.
* Exact contact math: front wheel centre (8.297, 1.175); step top corner (8.395, 1.100); distance 0.1233 m vs wheel radius 0.12 m -> **touching within 3 mm**, contact normal 38.7 deg from vertical. The wheel is pressed against the 0.05 m step and cannot roll over it.
* Independent check on the OLD sharp-world loaded freeze (`loaded_payload_diagnosis/ground_truth_pose_trace.csv`, frozen at gz x = 7.119): `sharp_old_freeze_check.py` -> front wheel centre (7.388, 1.147), distance to the crest step corner (7.500, 1.100) = 0.1211 m vs 0.12 m
  (**1.1 mm gap**). So the sharp-world "loaded payload cannot climb" finding was the SAME step.

## 2. The fix and its causal test (step 2)
Ramp slabs (fillets, ramp, rails) raised: curved worlds +0.050 m, sharp worlds +0.052 m; `zone_a` and `zone_c` untouched (spawn height and platform-z references assume 0.10 / 1.10). All 7 `ramp_world_*.sdf` files (originals: `data/*.ORIGINAL`); each edited file carries a comment. Seams after:
curved <= 1.4 mm, sharp <= 1.8 mm (table above). Causal test, same robot, same procedure, only the geometry differs (curved / 0.45 / loaded): original geometry **0 / 10** reach the platform (7 earlier corrected runs + 3 recorded), corrected geometry **3 / 3** in the first recorded check and 12 / 12 in the sample below.
A/B control with the final instrument (`control_origgeom_logs/`; world `ramp_world_curved_mu045_origgeom_TESTONLY.sdf` = the unfixed geometry, kept to show the failure on demand): same loaded robot, same defaults, same detector - **original geometry 0 / 4** reach the platform
(chassis advanced 0.02-0.04 m of the 2.4 m drive; 6-12 HALTs), **fixed geometry 12 / 12** (section 3). Fisher exact p = 0.0006 for 0/4 vs 12/12 (p ~ 1e-7 for 0 / 14 vs 12 / 12 counting all original-geometry corrected-procedure runs of this cell).

## 3. Reverification at sample size, corrected turn procedure, unscoped fixed detector (48625b76) (`reverify_*_logs/`, `reverify_analysis.txt`, `analyze_reverify.py --runs`)
Guarded (by-PID cleanup, idle-host wait, one sim at a time, per-trial retry on a guard stop; none starved). Completion = ascent PASS (ground-truth verified) AND the descent travelled >= 4.5 m of the 6 m leg on ground truth AND max |y| <= 1.5 m.

| Cell | Completed | Wilson 95% | ascent verdicts | HALTs (median; range; >= 8) | max \|y\| | min RTF | before the fix |
|---|---|---|---|---|---|---|---|
| curved / 0.45 / loaded | **12 / 12** | 76-100% | PASS 12 | 2; 0-9; 2 | 0.030 m | 0.990 (8/8 at >= 0.995 complete) | 0 / 10 reached the platform |
| curved / normal / loaded | **12 / 12** | 76-100% | PASS 12 | 4; 0-7; 0 | 0.020 m | 0.997 | 6 / 8 reached it |
| sharp / normal / loaded | **12 / 12** | 76-100% | PASS 12 | 2; 1-6; 0 | 0.030 m | 0.996 | 0 / >=8 (DID_NOT_CLIMB, every recorded run) |
| sharp / 0.45 / loaded | **12 / 12** | 76-100% | PASS 12 | 5; 1-10; 1 | 0.020 m | 0.995 | 0 / >=8 (DID_NOT_CLIMB, every recorded run) |
| curved / normal / unloaded (regression) | 6 / 6 | 61-100% | PASS 6 | 2; 2-8; 1 | 0.020 m | 0.998 | (0 / 18 failures earlier; no regression) |
| sharp / normal / unloaded (regression) | 6 / 6 | 61-100% | PASS 6 | 3; 1-6; 0 | 0.030 m | 0.997 | (earlier: robot never descended) |

**All four loaded cells: 48 / 48 completions (Wilson 93-100%)**; across all 60 runs the descent travelled 5.57-5.81 m (ground truth). Disclosed instrument amendment: the pre-registered completion test used a final-position "foot" criterion (x <= 5.0 m and z <= 0.5 m)
calibrated on the pre-fix leg end (x 4.2-4.5); after the ramp was raised 0.05 m the leg (whose end is set by odometry) ends at x 5.0-5.2, z 0.44-0.48 - the same physical descent - so that criterion read 41/60 purely on a 0.02-0.18 m offset. It was replaced by ground-truth
descent travel >= 4.5 m (all 60 runs travelled >= 5.57 m, so any sensible threshold gives the same answer). Both counts are in `reverify_analysis.txt`.
Note the sharp / normal / unloaded regression cell's first pre-registered count (5/6) and the curved-loaded 4/12 and 3/12 are that artifact, not failures.

## 4. Final corrected matrix, one run per cell, all cells (step 5) (`matrix_geometry_logs/`)
Corrected turn procedure (default), fixed geometry, unscoped fixed detector 48625b76, quiet host (min RTF 0.994-0.999). Verdict (HALTs; ground-truth max |y|; descent travel):

| Cell | Previous final matrix (corrected turn, ORIGINAL geometry: run A / run B) | **Final** |
|---|---|---|
| sharp / normal / unloaded | MARGINAL (4) / FAIL HALT_THRASH (9) | **MARGINAL** (6 HALTs; 0.03 m; 5.61 m) |
| sharp / normal / loaded | FAIL DID_NOT_CLIMB (7) / FAIL DID_NOT_CLIMB (13) | **FAIL: HALT_THRASH** (8 HALTs; ascent PASS; 0.02 m; 5.62 m) - completes; the FAIL is the HALT-count rule only (the 12-run sample: median 2, max 6, 0/12 >= 8) |
| sharp / 0.45 / loaded | FAIL DID_NOT_CLIMB (12) / FAIL DID_NOT_CLIMB (14) | **MARGINAL** (4; 0.02 m; 5.79 m) |
| curved / normal / unloaded | FAIL HALT_THRASH (11) / FAIL HALT_THRASH (11) | **MARGINAL** (1; 0.02 m; 5.71 m) |
| curved / 0.45 / unloaded | FAIL HALT_THRASH (8) / MARGINAL (5) | **MARGINAL** (4; 0.03 m; 5.79 m) |
| curved / normal / loaded | MARGINAL (3) / FAIL DID_NOT_REACH_PLATFORM (11) | **PASS** (0 HALTs; 0.02 m; 5.59 m) |
| curved / 0.45 / loaded | FAIL edge dep. (timed-drive artifact) / FAIL DID_NOT_REACH_PLATFORM (9) | **MARGINAL** (5; 0.02 m; 5.79 m) |

Every one of the 7 cells now has a ground-truth-verified ascent AND a full descent with the robot on the centerline (max |y| <= 0.03 m): **1 PASS, 5 MARGINAL, 1 FAIL (HALT count only)**. The earlier verdict history (prior / old / fixed / scoped detectors, both earlier corrected runs) is in `../path_structural/README.md` 7.4.

## 5. Ascent goal-completion gate (step 4)
* **Finding: ASCENT had no physical-reality gate.** The validation test called the ascent complete when `/ramp_state` reached EXIT (a pitch-based FSM state); the construction-site round-trip driver's ascent legs (1, 2) ended when the fused EKF x crossed a target. Only the descent (leg 3)
  had `/verified_progress_m` goal-verification. Same bug class as the descent goal-check the project already fixed once: the client trusted an estimate that a frozen chassis with spinning wheels keeps advancing.
* **Why the descent's pattern alone is insufficient (measured, not assumed):** during the freeze `/verified_progress_m` grew **4.32 m and 3.69 m in 50 s and 40 s (96% and 97% of the raw wheel distance)** while the chassis moved 0.00 m: the stall detector latches only after ~5 s of
  commanded-but-unchanged scans, then HALTs, and every resume re-accumulates about 5 s of spinning before the next latch. The documented claim that its biases are "in the safe direction" does not hold for cyclic freezes; a gate built on it would have passed the frozen robot.
  Retroactively revoking the pre-latch credit is not viable either: the detector false-latches ~12 times per construction-site session on a MOVING robot, which would strip real progress from leg 3.
* **Fix pattern applied ("refuse completion unless independent evidence agrees", with the strongest independent evidence the harness has - Gazebo ground truth):** ground-truth chassis displacement vs the odometry/EKF displacement over the ascent, tolerance 0.5 m (calibrated on 24 healthy
  leg ends from 8 earlier round trips: <= 0.09 m; in the reverification the disagreement was a steady -0.14 to -0.16 m over 60 ascents, and <= 0.035 m on every round-trip leg).
  `validation_test_ekf_groundtruth.py`: `FAIL: ASCENT_POSITION_UNVERIFIED` (new) and, for the platform drive, `FAIL: DID_NOT_REACH_PLATFORM` (last session). `arc_turn_followpath_test_ekf.py`: legs 1, 2 and 4 each checked; an unverified ascent stops the run before leg 3, and the per-leg checks (with `/verified_progress_m` recorded, not gated) are printed.
* **What the gate does NOT do:** it uses ground truth, which only the simulation harness has. The stack itself still has no on-robot physical ascent check; the on-robot options are limited by the same physics as the stall detector (an exteroceptive floor-motion sensor would be needed) - documented, not solved.
* **Positive control of the gate** (original-geometry control world, 4 runs): all 4 were caught - `FAIL: DID_NOT_REACH_PLATFORM` (ground-truth advance 0.02-0.04 m of 2.4 m) although `/ramp_state` reached EXIT in every one. Honest detail: the ascent-position comparison AT EXIT read only -0.152 to -0.154 m
  (inside the 0.5 m tolerance), because the chassis is still moving with the EKF when EXIT is declared - the freeze begins at the crest afterwards. So in the validation test it is the ground-truth platform drive that catches this freeze, not the EXIT-time disagreement. In the round-trip driver, leg 2 keeps driving past the crest until the EKF x
  reaches its target, so a crest freeze there shows up as a metres-large ground-truth shortfall at the end of leg 2 (not exercised: the construction site's geometry is already fixed).

## 6. Construction-site round trips with the ascent gate (`../construction_site_descent/roundtrip_geometry_*`, unscoped fixed detector)
| Payload | Ascent legs physically verified (GT vs EKF displacement) | Leg 3 | Verified | HALTs (total / descent) | Min margin | Final leg-4 pos (gz) | Min RTF |
|---|---|---|---|---|---|---|---|
| loaded | leg 1 +0.016, leg 2 +0.031, leg 4 -0.035 m: all ok | ok | 103% | 13 / 3 | 0.463 | (1.99, +1.56) | 0.970 |
| unloaded | leg 1 +0.027, leg 2 +0.014, leg 4 -0.033 m: all ok | ok | 103% | 31 / 15 | 0.872 | (1.95, -0.14) | 0.965 |
Both complete, no abort. The loaded leg-4 drift (1.56 m) is inside the known distribution (clean-host loaded n = 12: mean 0.93 m, max 1.572 m); the construction site's own geometry was fixed earlier and is unchanged. RTF 0.965-0.970 in these two runs is above the 0.95 guard but under the requested 0.995 (single runs, none starved).

## 7. Corrections to the earlier record
* README "The loaded payload cannot climb the sharp-world ramp at all - root-caused: a genuine physical/design constraint (breakover angle / zero ground clearance), not fixed by design" is **refuted**: that diagnosis located the freeze (gz x 7.119, at the seam) correctly but the cause was the 0.05 m authoring step; with it
  removed, the same 100 kg / +0.1 m-CG robot climbs the sharp world 24 / 24 times. `DID_NOT_CLIMB` for the two sharp loaded cells in every earlier matrix was this bug.
* Last session's "loaded curved cells often cannot clear the crest (a robot limitation the old harness hid)" was the same bug: 0 / 10 and 6 / 8 before, 24 / 24 now. The old harness's ascent PASS (FSM reached EXIT) still hid it, which is why the ascent gate matters.
* The old "friction floor" test results (mu 0.25 / 0.30 DID_NOT_CLIMB on the unloaded sharp world) are a genuine slope-traction result (tan 16 deg = 0.287) and are not touched by this finding; they were run on the stepped geometry, so their exact margin near the floor is not re-established.

## 8. Limits
* Not re-run: the breakpoint-detection accuracy scoring and the obstacle-LiDAR tests (their worlds changed by 0.05 m of ramp height; the FSM transitions worked in every one of the 60+ runs). The friction-floor experiments (mu 0.25-0.35) were not repeated on the fixed geometry.
* Single-run HALT verdicts remain noisy (the sharp / normal / loaded matrix cell reads FAIL at 8 HALTs while its 12-run sample has a maximum of 6).
* The ascent gate needs ground truth (harness only). RTF: reverification min 0.990-0.997 per cell (a few runs under the requested 0.995, none under the 0.95 guard, none starved); round trips 0.965-0.970.
* Why turning in place on the curved crest displaces the chassis (last session's optional item) is still not investigated; with the crest step removed it may no longer matter, which was not tested.

## Files
`seam_geometry.py` (exact seam math), `crest_diag.py` + `run_cell_crest.sh` (wheel-level recorder), `analyze_freeze.py`, `before_freeze_analysis.txt`, `sharp_old_freeze_check.py/.txt`, `data/*.ORIGINAL` (original worlds) and `data/*BEFORE_ASCENTGATE*` (pre-change test scripts),
`run_geometry_reverify.sh`, `run_cell_repeat_resumable.sh`, `analyze_reverify.py`, `reverify_*_logs/`, `matrix_geometry_logs/`, `before_logs/`, `after_logs/`, `control_origgeom_logs/`, `PREREGISTRATION.md`.
