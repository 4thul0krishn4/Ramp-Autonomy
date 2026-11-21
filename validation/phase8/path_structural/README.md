# Structural path experiment for the curved-world descent - final report (closes the curved-world investigation)

Question asked: can a structural change to the path source (densified waypoints with explicit lateral constraints, then an explicit
cross-track cost) close the `curved / normal / unloaded` descent edge departure that every tuning attempt left open?
Answer: **No - and the reason is not path tracking at all.** The departure is caused by where the test turns the robot around, and by
a localization blind spot the path cannot see. With the turn done on the flat platform, the *unchanged* MPPI, 0.4 m path and critics
descend the curved ramp cleanly (max |y| 0.02-0.05 m, 6 of 6 and again 6 of 6 runs), and the stall-detector workaround is not needed.

All runs: guarded clean host (by-PID cleanup, preflight, one sim at a time), same instrument, scoped detector installed unless stated.
Pre-registration (rules fixed before data, four dated amendments, all disclosed): `PREREGISTRATION.md`. Raw logs: `*_logs/`.
Every ground-truth (E1) failure = max |y| > 1.5 m. E2 = the robot actually descends (odom leg >= 2.5 m of the 6 m path).

## 1. Isolated effect of path densification (step 2 of the request)

Arm A: waypoint spacing 0.4 m -> 0.05 m (121 points, straight, every waypoint pinned to y = 0), nothing else changed. n = 12.

| | n | E1 failures (Wilson 95%) | max \|y\| mean | genuine descents (E2) | median leg | HALT_THRASH (>= 8) |
|---|---|---|---|---|---|---|
| original detector (baseline) | 6 | 0 (0-39%) | 0.90 m | 0 / 6 | 0.48 m | - |
| unscoped fixed detector | 10 | 10 (72-100%) | 2.49 m | 0 / 10 | 0.8 m | - |
| scoped detector, 0.4 m path (baseline) | 14 | 5 (16-61%) | 1.32 m | 0 / 14 | 0.5 m | - |
| **Arm A: dense path** | 12 | **0 (0-24%)** | **0.42 m** | **0 / 12** | **0.10 m** | 5 / 12 |

Against the scoped baseline: failure-rate difference -36 points (95% CI -61 to -5), Fisher p = 0.043; mean max |y| -0.90 m (bootstrap
CI -1.31 to -0.51). Against the original: 0 points (CI -39 to +24), p = 1.0. Against fixed: -100 points, p < 0.0001.
Strictly RTF >= 0.995 runs only: 0 / 7. **Read plainly: the "improvement" is the robot not moving.** All 12 runs end at Nav2's 15 s
"Failed to make progress" abort with the robot still at the crest (travel 0.05-0.12 m of 6 m; ground truth x 7.85-7.99, y moved by at most 0.45 m). E1 is met trivially; there is no descent. Effect of densification on genuine descent: none (0/12 vs 0/14).

## 2. Arm B: dense path + explicit cross-track critic (step 1's second half)

`ramp_autonomy_critics/RampCrossTrackCritic` (new C++ MPPI plugin, built and loaded - the controller log confirms it): every sampled
trajectory point's distance to the nearest path segment (no arc-length matching, no furthest-point gating) weighted 30, plus a
corridor hinge (+/-0.5 m, weight 300). Added to the critic list for the arm, then removed (params restored to sha 55f005aa).
n = 12: **E1 0 / 12 (0-24%), E2 0 / 12, median leg 0.05 m, HALT_THRASH 10 / 12.** Same as Arm A (robot static at the crest) and noisier
(more HALTs). Versus scoped baseline: -36 points (CI -61 to -5), p = 0.043 - again by not moving. Strict RTF >= 0.995: 0 / 4.

## 3. Why: what the recorded runs show (`descent_diag.py`: ground-truth pose/yaw + EKF + commands, passive, test unmodified)

| Procedure (n) | yaw at descent start | true y offset at start (EKF says 0.00) | outcome |
|---|---|---|---|
| open-loop turn, as the harness always did (17 recorded runs, all 3 arms) | 83-101 deg (should be 180) | +0.13 to +0.46 m | robot faces the panel edge, MPPI commands almost no rotation (mean wz 0.03-0.10 vs wz_max 0.6), creeps sideways; leg ends by the 15 s abort or the robot falls off |
| closed-loop turn on the crest, Arm H (6) | 176 deg (correct) | **+1.0 to +1.3 m** (edge at 1.5) | **E1 failed 5/6** (max \|y\| 2.19-2.31, off the panel); E2 1/6 |
| **closed-loop turn on the flat platform, Arm P (6)** | 176 deg | **+0.01 to +0.02 m** | **E1 0/6 (max \|y\| 0.02-0.04); E2 6/6; full descent to the ramp foot (ground truth x ~9.9 -> 4.2, z 0.23)** |
| Arm P with the UNSCOPED fixed detector (6) | 176 deg | +0.01 to +0.02 m | **E1 0/6 (0.03-0.05), E2 6/6, HALT_THRASH 0/6** |

Mechanism, from the timelines (`mechanism_table.txt`, `analyze_diag.py`):
1. The harness's 189-degree open-loop turn (0.5 rad/s x 6.6 s) achieves only ~90 degrees on the curved crest, so the robot starts
   the descent facing +y, toward the panel edge. The path is built from raw wheel-odometry yaw, which believes the turn finished.
2. Whatever the turn, the chassis **translates sideways** while it spins on the crest (roughly 0.4-0.7 m per 90 degrees of rotation), and the EKF cannot
   see it: it fuses wheel FORWARD velocity only, so its y stays exactly 0.00 through the whole turn. The controller believes it is on
   the centerline while it is 0.2-1.3 m off; the panel edge is 1.5 m from the centerline.
3. In the open-loop case MPPI then commands almost no rotation and the robot creeps toward the edge until the 15 s progress abort ends
   the leg (a "PASS") or it gets there first (a "FAIL"). So the pass/fail outcome of this cell has been a race between creep and abort.
4. Densifying the path, or adding a lateral cost, acts on lateral error in the EKF's frame, where the robot appears centered. They
   cannot see, and cannot fix, the displacement that already happened. That is why every path/critic tuning attempt failed.
5. Where the turn is done is what matters: on the flat platform the displacement is 0.02 m and the untouched controller tracks the
   path to within 5 cm for the whole ramp.

## 4. What this changes in the earlier record (corrected in the README and docs)
* The "second mechanism" in `crosstrack_diagnosis/README.md` (a stumble at the `fillet_top_1/2/3` seam cluster) and the "unchecked
  crosstrack growth" reading were never tested as causes; the recorded runs show the robot sitting at the crest (x ~ 7.4-8.4) because of
  its heading and displacement, not tracking a path. The `offset_from_furthest` fix is real (the critic really was disabled) but its
  measured "2.85 -> 2.03 m" improvement was one run vs one run on a distribution with sd 0.6-0.7 m.
* The ground-truth PASS on curved cells at normal friction (unloaded AND loaded) has meant "did not leave the panel before the leg ended",
  not "descended" - in every recorded run at normal friction the robot travelled 0.1-2 m of the 6 m path. The one cell that reaches the
  ramp foot in the matrix, `curved / 0.45 / unloaded`, does it by REVERSING down the ramp (its turn achieves 0 degrees).
* The in-DESCENT stall pause that "helps" is consistent with this: a pause shortens the creep. It is a delay of the departure, not a fix.

## 5. Verdict against the request
* Step 2 (densify alone): no effect on descent; the E1 numbers above are a stationary robot.
* Step 3 (with the scoped detector): done for two arms at n = 12 each, compared with all three baselines (tables above), with CIs.
* Step 4 (remove the scoping, rerun matrix + round trips): **not done in round 1, deliberately (superseded: DONE in round 2, section 7).** The path fixes did not close the gap on any
  reading that involves a descent (E2 0/12, 0/12). The gap IS closed by moving the turn onto the platform (Arm P and Arm P-fixed, both 6/6
  clean, the scoping unnecessary), but that is a change to the curved-cell test PROCEDURE, not to the stack; the stack still cannot turn
  on the crest without unseen displacement. Removing the scoping and re-running the matrix would only be valid together with that
  procedure change, which redefines the instrument for every curved cell - left to you. The scoped detector stays installed (ea1bcee5).
* Step 5: documented - but the "structural limitation of MPPI" framing is refuted for this cell: MPPI tracks the path correctly when the
  robot starts on it. The real limitation is localization: no absolute lateral position reference, so slip-induced displacement is invisible.

## 6. Limits of this evidence
Arm P / P-fixed are n = 6 each (0/6, Wilson 0-39%); they show clean descents in every run but cannot bound a rare failure rate. RTF: many
runs of Arms A/B/P dipped to 0.985-0.994 (under the 0.995 bar; none under the 0.95 guard, none starved); the strict subsets are reported
above and give the same picture. The raw-odom leg metric reads 2.6 m for Arm P's descents while ground truth shows ~5.5 m; E2 is met
on both. The reason the turn displaces the chassis on the crest was not investigated (crest geometry / pivoting on tilted panels is
a guess, not a finding). The unscoped detector under the platform-turn procedure was only run at normal friction, curved, unloaded.

## Files
`PREREGISTRATION.md`, `analyze_arm.py`, `analyze_diag.py`, `descent_diag.py`, `run_cell_diag.sh`, `run_diag_batch.sh`, `mechanism_table.txt`,
`dense005_*`, `armB_*`, `armH_*`, `armP_*`, `armPfixed_*`, `diag_*`; code: `ramp_autonomy_nav/path_source.py` (opt-in, unused by default),
`src/ramp_autonomy_critics/` (built, NOT loaded: params restored), `validation_test_ekf_groundtruth.py` (three opt-in switches,
defaults reproduce the historical run: `PATH_SPACING`, `CLOSED_LOOP_TURN`, `PLATFORM_TURN_M`; pre-change copies kept here).

---

# 7. Round 2 (project owner's decision: the corrected turn procedure is the standard for every cell) - FINAL, TRUSTWORTHY STATE

Decision rules and all disclosed amendments: `PREREGISTRATION_ROUND2.md`. Nothing below used the historical crest open-loop turn.

## 7.1 The corrected procedure is now the default
`validation_test_ekf_groundtruth.py`: drive 2.4 m further onto the flat top platform (distance measured on Gazebo ground truth, 60 s cap), then turn CLOSED-LOOP on the EKF yaw to 180 degrees, then descend.
Default for EVERY world (override for the historical procedure: `CLOSED_LOOP_TURN=0 PLATFORM_TURN_M=0`). Sharp world checked first (`sharp_diag_logs/`, 3 runs): its open-loop crest turn achieves only 62-67 of 180
degrees (little sideways displacement, 0.07-0.08 m) so the robot never descended there either (0.05-0.12 m of 6 m); the pre-registered adoption rule (yaw < 150 deg OR unseen y >= 0.15 m) was met, so sharp worlds
use the corrected procedure too. Historical scripts that still use the crest turn (`validation_test.py`, `crosstrack_instrument.py`, `curved_descent_diagnosis.py`) are marked UNCONFIRMED at the top.
Two instrument corrections found by the re-run itself (amendments 5 and 6): (a) a TIMED platform drive is not enough - a loaded robot can be stuck at the crest, so the drive is distance-based on ground truth;
(b) if the chassis never reaches the platform the run is `FAIL: DID_NOT_REACH_PLATFORM` (an ascent failure) and no turn/descent is attempted; matrix test cap 220 -> 360 s.

## 7.2 Larger samples (curved / normal / unloaded, guarded clean host, defaults, 12 new runs per arm + the 6 earlier of the identical procedure)
E1 = max |y| <= 1.5 m; E2' = reached the ramp foot (final ground-truth x <= 5.0 m and z <= 0.5 m). A run fails if either fails. (`round2_analysis.txt`, `analyze_round2.py --runs`)

| Detector | New 12 | Earlier 6 | **Pooled n = 18** | Wilson 95% | max \|y\| (mean / worst) | HALT_THRASH (>= 8) | strict RTF >= 0.995 subset |
|---|---|---|---|---|---|---|---|
| Arm P: scoped (ea1bcee5) | 0/12 fail | 0/6 | **0 / 18** | **0-18%** | 0.035 / 0.056 m | 8 / 18 | 0 / 10 |
| Arm P-fixed: unscoped fixed (48625b76) | 0/12 fail | 0/6 | **0 / 18** | **0-18%** | 0.037 / 0.048 m | 3 / 18 | 0 / 13 |

All 36 runs reached the foot of the ramp. Scoped vs unscoped: Fisher p = 1.0 on failures. The one visible difference is HALT count: the scoped detector (the workaround) produced HALT_THRASH in 8/18 runs vs 3/18
unscoped (Fisher p = 0.146, not significant, but the direction matches the workaround's known extra-HALT cost). Min RTF over all runs 0.983-0.999 (several under the requested 0.995, none under the 0.95 guard,
none starved); the strict subsets are also 0 failures.

## 7.3 Scoping decision: REMOVED
Pre-registered rule (P-fixed <= 1 failure in 18): met with 0/18. The unscoped fixed detector (sha 48625b76, the rotation check active in every state, no `ROTATION_CHECK_EXCLUDED_STATES`, no `/ramp_state` dependency) is installed
and is what `src/` contains; the scoped version is kept as `stall_detector_control/data/motion_stall_detector_node.SCOPED.py` (ea1bcee5). The removal steps were followed:
* 10+ unscoped curved-cell runs: 18 (section 7.2).
* Positive control (construction site, 6 normal + 6 freeze sessions, `stall_detector_control/data/final_*`): **forced freezes detected 18/18 (Wilson 82-100%; straight 6/6, arc 6/6, taper 6/6, median latency ~5 s), false latches while rotating 0
  (all remaining false latches are straight travel, the known physical limit), leg 3 completed 6/6 in 131 s mean, 26.8% of leg-3 time stall-latched.** Original validation of this identical file: 18/18, 123 s, 27.4%.
* Full matrix and both round trips (7.4, 7.5).

## 7.4 Corrected 7-cell matrix - old vs new verdict for every cell
"Old" columns are the crest-open-loop-turn procedure with each detector (detail in the validation README section 4); their curved-cell ground-truth PASSes meant a robot that never left the crest. "New" = corrected procedure, unscoped detector.
Run A = first corrected matrix (`matrix_final_logs/`, quiet host, RTF 0.995-0.999; its platform drive was TIMED, which failed only for curved / 0.45 / loaded); Run B = final instrument (`matrix_final2_logs/`, RTF 0.972-0.994, load 6-10).
Format: verdict (HALTs; GT max |y|).

| Cell | Prior trusted | Old detector | Fixed | Scoped | **New run A** | **New run B (final instrument)** | What it now means |
|---|---|---|---|---|---|---|---|
| sharp / normal / unloaded | PASS (0) | MARGINAL (2; 0.10) | MARGINAL (3; 0.08) | MARGINAL (6; 0.09) | MARGINAL (4; 0.02) | FAIL: HALT_THRASH (9; 0.02) | now a real descent (final ground-truth x 4.6 / 3.8 m, the ramp foot); previously a robot sitting at the lip facing sideways. HALT verdict flips 4 vs 9 (single-run noise) |
| sharp / normal / loaded | FAIL DID_NOT_CLIMB | FAIL (7) | FAIL (6) | FAIL DID_NOT_CLIMB (11) | FAIL DID_NOT_CLIMB (7) | FAIL DID_NOT_CLIMB (13) | unchanged - never reaches the turn |
| sharp / 0.45 / loaded | FAIL DID_NOT_CLIMB | FAIL (8) | FAIL (9) | FAIL DID_NOT_CLIMB (7) | FAIL DID_NOT_CLIMB (12) | FAIL DID_NOT_CLIMB (14) | unchanged - never reaches the turn |
| **curved / normal / unloaded** | FAIL edge dep. (4; 2.68) | FAIL HALT_THRASH (11; 1.01 = stuck at crest) | FAIL (14; GT FAIL 2.49) | FAIL HALT_THRASH (11; 0.93 = stuck) | FAIL HALT_THRASH (11; **0.01**) | FAIL HALT_THRASH (11; **0.02**) | **ground truth clean and a real full descent (18/18 in the larger sample, worst 0.048 m)**; the overall FAIL is HALT count only (matrix runs 11, 11; the 18-run sample has median 6, 3/18 >= 8) |
| **curved / 0.45 / unloaded** | MARGINAL (1; 0.36) | MARGINAL (3; 0.31 = reversing) | MARGINAL (4; 0.23) | FAIL HALT_THRASH (13; 0.14) | FAIL HALT_THRASH (8; 0.05) | MARGINAL (5; 0.04) | real forward descent now (was a reverse descent); HALT verdict flips 8 vs 5 |
| **curved / normal / loaded** | MARGINAL (4; 0.97 = stuck) | MARGINAL (4; 0.94 = stuck) | MARGINAL (3; 1.08 = stuck) | MARGINAL (5; 0.95 = stuck) | MARGINAL (3; **0.02**, real descent) | FAIL DID_NOT_REACH_PLATFORM (11 HALTs) | **stochastic**: with 6 repeats, 6 of 8 corrected runs got onto the platform and descended cleanly (max \|y\| 0.02 m, Wilson 41-93%), 2 of 8 were stuck at the crest |
| **curved / 0.45 / loaded** | FAIL HALT_THRASH (9; 0.93 = stuck) | MARGINAL (3; 0.93 = stuck) | MARGINAL (2; 1.07 = stuck) | FAIL HALT_THRASH (8; 1.02 = stuck) | FAIL edge dep. 2.21 (10 HALTs) - **artifact of the timed drive** (turned at the crest) | FAIL DID_NOT_REACH_PLATFORM (9 HALTs) | **0 / 7 reach the platform** (Wilson 0-35%): the loaded robot on mu 0.45 cannot clear the crest (ground truth advances 0.02-0.04 m in 60-90 s, wheels spin, EKF runs away to x = 14 m). Robot limitation like the sharp loaded cells; every previous "PASS" was the same stuck robot |

Curved-world ground truth is now clean in EVERY corrected-procedure curved run in which the robot reached the platform: 46 runs (36 of the larger samples, 5 loaded-normal repeats, 1 loaded-normal matrix run and 4 unloaded matrix runs), max |y| <= 0.056 m.
Between the two matrices the ascent and ground-truth outcomes agree in 6 of 7 cells (the exception, curved / normal / loaded, is the stochastic crest clearing; curved / 0.45 / loaded's run A was the timed-drive artifact); the HALT verdict flips in 2
of the cells with comparable outcomes (sharp / normal / unloaded 4 vs 9, curved / 0.45 / unloaded 8 vs 5). RTF: run A 0.995-0.999; run B 0.972-0.994 (guard bar 0.95 met, requested 0.995 not).

## 7.5 Construction-site round trips (`construction_site_descent/roundtrip_final_*`, unscoped fixed detector; their turn is the arc on the flat platform, unchanged by this decision)
| Payload | Detector | Leg 3 | Verified | HALTs (total / descent) | Min margin | Final leg-4 pos (gz) | Min RTF |
|---|---|---|---|---|---|---|---|
| loaded | old | ok | 102% | 24 / 12 | 0.463 | (2.00, +1.57) | - |
| loaded | fixed (earlier) | ok | 103% | 14 / 4 | 0.463 | (2.02, +1.17) | - |
| loaded | scoped | ok | 103% | 15 / 4 | 0.463 | (1.98, +1.42) | - |
| loaded | **unscoped, final** | **ok** | **103%** | **22 / 9** | 0.463 | (2.08, +0.42) | 0.972 |
| unloaded | old | ok | 102% | 40 / 26 | 0.872 | (1.96, -0.10) | - |
| unloaded | fixed (earlier) | ok | 103% | 30 / 15 | 0.872 | (2.06, +0.00) | - |
| unloaded | scoped | ok | 102% | 31 / 15 | 0.872 | (2.04, +0.01) | - |
| unloaded | **unscoped, final** | **ok** | **102%** | **36 / 19** | 0.872 | (2.01, -0.07) | 0.973 |
Both complete, no abort; single runs each, so HALT counts vary (loaded 22 vs the earlier 14 for the same detector file, unloaded 36 vs 30); leg-4 drift 0.42 m loaded / -0.07 m unloaded, the smallest loaded value recorded. (Descent "max |y|" of 3.9-4.1 m in these logs is the 2 m-radius arc turn itself, by design.)

## 7.6 What is now settled, and what is not
Settled: the curved-world descent works on the unchanged controller when the robot turns on the platform; the DESCENT-scoped stall-detector workaround was compensating for a test-procedure artifact and is removed;
the positive control and round trips are unchanged by the removal; every earlier curved-world verdict is superseded by 7.4. Not settled / new limits found: (1) loaded curved cells: a loaded robot cannot reliably clear the crest
(curved / normal / loaded 6 of 8 corrected runs reach the platform; curved / 0.45 / loaded 0 of 7) - a capability limit, invisible to the old harness because /ramp_state reaches EXIT while the chassis is still on the crest;
(2) single-run HALT verdicts are noisy in both directions; (3) OPTIONAL future work, not needed for anything above: why turning in place on the curved crest displaces the chassis 0.2-1.3 m sideways (crest geometry / pivoting
on tilted panels is a guess, not a finding), and an absolute lateral position reference for the EKF (its y is blind to slip).

**Correction (2026-09-20, `../crest_freeze/README.md`):** the "loaded curved cells cannot reliably clear the crest (a capability limit)" finding in 7.4 and 7.6 was a 0.05 m geometry authoring step at the crest / ramp foot in `ramp_world_*.sdf` (the same bug class as the construction-site freeze), not a robot limit. With the ramp slabs raised, the four loaded cells complete the ascent and the descent 48 / 48. The final matrix is in the crest_freeze README section 4 and the main README.
