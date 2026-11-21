# Round 2 pre-registration - larger samples for the corrected turn procedure, scoping decision, corrected matrix (written BEFORE any round-2 data)

Decision taken by the project owner: the corrected standard for ALL curved-world cells is "closed-loop turn to 180 degrees on the EKF yaw, done
2.4 m further onto the flat top platform" (Arm P's procedure); it is now the DEFAULT in `validation_test_ekf_groundtruth.py` for curved worlds
(and, after the sharp-world check below, for sharp worlds too). Override for the historical procedure: `CLOSED_LOOP_TURN=0 PLATFORM_TURN_M=0`.

## 1. Larger samples (curved / normal / unloaded, guarded clean host, one sim at a time, defaults - no env switches)
* Arm P  = scoped detector (sha ea1bcee5) installed: 12 NEW runs. Arm P-fixed = unscoped fixed detector (sha 48625b76): 12 NEW runs.
* The 6 + 6 runs of the earlier session used the identical procedure and detector files; results are reported new-only AND pooled (n = 18 each).
* Endpoints per run. E1: ground-truth max |y| <= 1.5 m. E2 (descent actually happened; redefined here on ground truth because the raw-odom leg
  metric under-reads this procedure, 2.6 m for a ~5.5 m descent): final ground-truth x <= 5.0 m AND z <= 0.5 m (the robot reached the foot of the ramp).
  A run "fails" if E1 or E2 fails. Also reported: HALT count / HALT_THRASH (>= 8), min RTF (requested bar 0.995; strict subsets given).
* Statistic: Wilson 95% interval on the failure proportion of the pooled sample (0/18 -> upper bound 17.6%; 1/18 -> 27%; 2/18 -> 35%).

## 2. Scoping decision rule (fixed now)
* The corrected procedure is "trusted" only if Arm P (scoped) itself has <= 1 failure in 18. If not, stop and report; nothing else is removed.
* If Arm P-fixed (unscoped) has <= 1 failure in 18 (Wilson upper bound <= 27%): follow the documented removal - install the unscoped fixed detector
  (sha 48625b76; i.e. the scoping machinery removed entirely, `ROTATION_CHECK_EXCLUDED_STATES` gone), re-run the positive control (6 normal + 6 freeze
  sessions, construction site, same protocol and scoring as the original validation: false latches while rotating 0, forced freezes detected),
  the full 7-cell matrix and both construction-site round trips under the final detector.
* If Arm P-fixed has >= 2 failures in 18: keep the scoped detector, re-run the matrix and round trips with it, and report why plainly.
* The 18 unscoped runs also satisfy the "10+ unscoped curved-cell runs" step.

## 3. Sharp-world check (before adopting the default there)
Three recorded sharp/normal/unloaded runs (the only sharp cell that reaches the turn; the two loaded sharp cells fail DID_NOT_CLIMB before it) with the passive
recorder: yaw achieved by the open-loop turn, and the chassis displacement (ground-truth y minus EKF y) at the descent start. Adopt the corrected procedure for
sharp worlds only if the displacement / heading shortfall is comparable to the curved-crest values (yaw < 150 degrees or |unseen y| >= 0.15 m); otherwise leave it.

## 4. Final matrix
7 cells x 1 run, corrected procedure for curved cells, under the detector chosen in section 2; reported old vs new for every curved cell (prior trusted /
old detector / fixed / scoped columns already exist) with the new procedure's E1/E2 and HALT count. Both round trips re-run (their turn is the arc on the flat
platform and is unchanged by this decision).

## Result of the sharp-world check (section 3), recorded before any other round-2 data
3 recorded sharp/normal/unloaded runs (`sharp_diag_logs/`): the open-loop turn achieved 62-67 degrees (yaw at descent start 62-67, should be 180); unseen lateral
displacement small (-0.07 to -0.08 m, below the 0.15 m bar); descent leg 0.05-0.12 m of 6 m in all three (the robot sits at the lip facing sideways; 15 s Nav2 abort;
its ground-truth PASS 0.08-0.12 m is a stationary robot). The pre-registered adoption rule is an OR (yaw < 150 deg or |unseen y| >= 0.15 m) and the yaw condition
is met, so the corrected procedure is adopted for sharp worlds too: default on for every world. (Different mechanism from the curved crest - heading shortfall, not
displacement - same consequence: the historical sharp-world descent verdicts also did not test a descent.) The sharp top platform (x 7.5-11.5) fits the 2.4 m drive.

## Amendment 5 (after the first corrected matrix and the 5-run follow-up of curved / 0.45 / loaded; before any re-run)
Positive control (unscoped fixed detector, 6 normal + 6 freeze sessions): forced freezes 18/18 detected (Wilson 82-100%), false latches while rotating 0, leg 3 completed 6/6,
26.8% of leg-3 time stall-latched - the same as the original validation of this detector file (18/18; 27.4%). Both round trips completed (ok=True, no abort).
First corrected matrix (`matrix_final_logs/`): 6 of 7 cells reached a genuine descent or (sharp loaded cells) the same DID_NOT_CLIMB; `curved / 0.45 / loaded` FAILED ground truth
(max |y| 2.21 m) 6 of 6 times (matrix run + 5 recorded repeats) - but not during the descent: the robot was already at gz y = -2.2 m (off the platform) when the descent goal was sent.
Recorded timeline: the loaded low-friction robot stalled/HALTed at the crest (gz x 8.02) for ~10 s during the TIMED 2.4 m platform drive, so the timed drive ended with the
robot still on the crest, and the closed-loop turn then happened there - reproducing the EKF-invisible crest displacement (-1.1 m per ~90 degrees) and driving it off the edge.
This is a defect in how the corrected procedure was implemented (a timed drive cannot guarantee the robot reached the platform), not a robot verdict. In the other 5 cells and
in all 36 P / P-fixed runs the timed drive worked (turn started at gz x 9.5-10.6). Change (instrument, decided before re-running): the platform drive is now DISTANCE-based on Gazebo
ground truth (advance >= 2.4 m or 90 s timeout; wheel odometry cannot measure it because it counts wheel rotation while the chassis is blocked). Where the timed drive succeeded
the new drive is equivalent (same ~2.4 m). Re-run: 5 repeats of curved / 0.45 / loaded, then the FULL 7-cell matrix again (instrument changed, so no cell is carried over).
Interpretation rule: if curved / 0.45 / loaded still fails after the drive really reaches the platform, that is reported as a physical / robot result for that cell.

## Amendment 6 (during the follow-up; before the matrix re-run started)
The distance-based platform drive showed the real situation for `curved / 0.45 / loaded`: with ground truth as the yardstick the loaded robot advances 0.02 m in 60-90 s of driving from the crest (gz x stays 8.02 m for
the whole time while wheel odometry / the EKF run away to x = 14 m: wheels spinning, chassis blocked). It never gets onto the platform. In every earlier record of this cell (old procedure) it also sat at gz x = 8.01; the
harness called the ascent a PASS because /ramp_state reached EXIT (pitch-based), so those "descent" results were a robot stuck at the crest. Instrument change: if the platform drive advances < 80% of the requested distance, the
run is reported as `FAIL: DID_NOT_REACH_PLATFORM` (an ascent failure, like DID_NOT_CLIMB) and no turn/descent is attempted (turning at the crest would only reproduce the displacement and fake an edge departure);
platform-drive timeout 60 s; matrix test cap raised 220 -> 360 s. The 5 recorded repeats before this change (r1-r5 in `curved_loaded_mu045_fix_logs/`) show the stuck robot and are kept as evidence; r2-r4 were cut off by the old 220 s cap.
