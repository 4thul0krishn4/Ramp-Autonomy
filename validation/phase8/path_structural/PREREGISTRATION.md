# Structural path experiment - pre-registration (written BEFORE any run of the new arms)

Date: 2026-09-19. Question: does a structurally better path source close the `curved / normal / unloaded` ramp-descent
edge departure that every tuning attempt left open?

## Instrument (unchanged)
`validation_test_ekf_groundtruth.py` on world `curved`, payload 0, guarded clean host (by-PID cleanup, preflight idle host,
one sim at a time, RTF monitored), scoped stall detector installed (`ROTATION_CHECK_EXCLUDED_STATES = ('DESCENT',)`,
sha256 `ea1bcee5`), watchdog original. The only variable per arm is the descent path / controller critic set.
Outcome = ground truth: max |y| > 1.5 m (panel half-width) => FAIL. RTF >= 0.995 is the requested bar; every result is
reported for all runs AND for the strict (min RTF >= 0.995) subset.

## Baselines (same instrument, already collected)
original detector 0/6 fail (Wilson 0-39%) | unscoped fixed detector 10/10 fail | scoped detector 5/14 fail (36%, Wilson 16-61%).

## Arm A - densify only (isolation)
Change: descent path waypoint spacing 0.4 m -> **0.05 m** (a single value, chosen a priori and NOT tuned: it is the
spacing at which the 3-waypoint `offset_from_furthest` window of `descent_mode` is 0.15 m, well inside the ~1 m the
MPPI horizon can reach at 0.35 m/s; 0.4 m spacing gives ~2 waypoints of reach). Path is otherwise identical: straight,
every waypoint pinned to y = 0, same endpoints. No MPPI parameter, critic or detector change. n = 12.
Effect size reported vs the scoped baseline (its own arm) and vs the other two baselines.

## Arm B - only if Arm A does not close the gap: explicit cross-track critic on the dense path (n = 12).

## Decision rules
"Closes the gap" = **at most 1 failure in 12** (point estimate <= 8%; Wilson upper bound 35%, at or below the original
detector's own 0/6 upper bound of 39%) AND no significant worsening vs the original (Fisher two-sided p > 0.05). The
literal wording "statistically indistinguishable from the original" is NOT used as the bar by itself: with the
original at n = 6 the scoped detector (5/14) already satisfies it (p = 0.26) while clearly not matching it.
Failures 2-3 of 12 = partial improvement (reported with CI, gap not closed); >= 4/12 = no meaningful effect.
HALT count (matrix rule: >= 8 = HALT_THRASH) is reported as a secondary cost.
If a fix closes the gap with the scoped detector, the scoping is removed ONLY after the same criterion is also met with
the unscoped detector (n = 12) - the documented removal condition is that the descent stays on the panel WITHOUT the
accidental in-DESCENT stall pause, so the scoped-detector result alone cannot license removal. If unmet, the scoped
workaround is left in place and the limitation documented.

## Amendment 1 (written after the first 4 of 12 Arm-A runs had finished; disclosed as data-informed)
The pre-registered outcome (ground-truth max |y| <= 1.5 m) can be passed by a robot that never leaves the crest. While
Arm A was running, a tabulation of ALL 85 curved-world result logs showed that at normal friction the robot travels only
0.1-2 m of the 6 m descent path in essentially every run (median odom leg distance 0.45 m; `n=15-29` ground-truth samples =
the leg ends by Nav2's 15 s "Failed to make progress" abort while the robot sits at the crest, gz x ~ 7.4-8.4). The first 4 dense-path runs
show the same, more extremely: leg distance 0.05-0.12 m, y unchanged. So a ground-truth PASS on this cell has never
meant "descended and stayed on the panel"; it has meant "did not slide off the crest before the leg was aborted".
Therefore every arm is reported on TWO endpoints:
  (E1) ground truth max |y| <= 1.5 m (the original criterion and the one the request compares against the baselines), and
  (E2) genuine descent: odom leg distance >= 2.5 m (of the 6 m path) in that run.
Two tiers of claim, decided before Arm A finishes:
  * "closes the E1 gap" (the criterion in the request) = at most 1 of 12 runs with E1 violated, judged against the baselines.
  * "genuinely fixes the descent" = additionally at most 1 of 12 runs failing E2. NOTE the original detector (0/6 on E1) itself
    would fail E2 in every run, so E1 alone cannot show a descent; a result that passes E1 while E2 stays at 0/12 is reported
    as "E1 met without descending - not evidence of a working descent", and the stall-detector scoping is NOT removed on
    that basis (the removal condition is that the descent works without the accidental pause).
E2's 2.5 m threshold is set (before Arm A finishes, and before any Arm B run) so that every run of the
`curved / 0.45 / unloaded` reference cell qualifies: that cell does descend, reaching 2.8-3.8 m in its 90 s window (its deadline, not
the path, ends those runs), so it is the yardstick for what a genuine descent looks like on this instrument.
(An earlier draft of this amendment said 3.0 m; that would have failed the reference cell's own 2.83 m run, so it was corrected.)

## Amendment 2 (after the diagnostic batch `diag_logs/`, 6 recorded runs; before Arm H or Arm B)
Recorded ground truth vs EKF yaw vs commands (descent_diag.py, analyze_diag.py). Findings, not hypotheses:
  1. The test's open-loop "180 degree turn" (0.5 rad/s x 6.6 s = 189 deg commanded) achieves only 85-101 deg of rotation at normal
     friction (5 recorded runs) - about half - so the descent leg starts with the robot facing ~+y (yaw 85-101 deg), i.e. ~90 deg
     off the -x path and pointing at the panel edge, 1.1-1.3 m away. EKF yaw equals ground-truth yaw to 1 deg (the controller knows).
     (The test builds the path from raw WHEEL-odometry yaw, which believes the turn completed.)
  2. Given that heading MPPI commands almost no rotation (mean wz 0.03-0.10 rad/s against wz_max 0.6) and vx 0.02-0.06 m/s, so the
     robot creeps along its heading toward +y. In the one recorded departure it fell off at t = 16 s; in the others the 15 s
     "Failed to make progress" abort ended the leg first. Outcome = a race between creep and abort, not a tracking error.
  3. At mu 0.45 the "turn" achieves 0 deg (wheels slip, no rotation) and the cell "descends" by REVERSING down the ramp at
     vx -0.04 m/s facing up the ramp. So the one "genuine descent" reference is a reverse descent, not a forward one.
Consequence for the design: waypoint density / a cross-track cost act on lateral error (0.2-0.4 m), not on a 90 degree heading
error, so neither is expected to matter; Arm A confirmed no effect on descent. Two additions, decided before their data:
  * Arm H (causal test of finding 1-2): identical cell/detector/0.4 m path, but the 180 degree turn is closed-loop on EKF yaw
    (opt-in `CLOSED_LOOP_TURN=1`, default off), n = 6. Prediction if the mechanism is correct: E1 and E2 both pass in nearly all
    runs. Prediction if it is wrong: it looks like the baseline.
  * Arm B is still run as requested (explicit cross-track critic on the dense path, n = 12) and scored on E1/E2 with the same rules.

## Amendment 3 (after Arm H and Arm B; before Arm P) - results so far and one diagnostic experiment
Arm H (closed-loop turn, 0.4 m path, n=6) REFUTED the "heading alone" prediction: E1 failed 5/6 (max |y| 2.19-2.31 m, robot off the
panel at gz z=0.40), E2 met 1/6. Recorded cause: during the 180 degree turn on the curved crest the chassis translates sideways
by ~0.2-0.4 m (open-loop, ~90 deg of rotation) to 1.0-1.3 m (closed-loop, 180 deg) while the EKF's y stays exactly 0.00 (its
position integrates wheel FORWARD velocity only; pivot/slip translation is invisible), so the robot starts the descent
1.0-1.3 m off the centerline (edge at 1.5 m) believing it is on it. Arm B (dense path + explicit cross-track critic, n=12)
behaved like Arm A: E1 0/12 failures, E2 0/12 descents (median leg 0.05 m), robot static at the crest, HALT_THRASH 10/12.
Arm P (diagnostic only, not a fix; n=6): identical to Arm H (closed-loop turn) but the robot first drives 2.4 m further onto
the FLAT top platform and turns there. Prediction if the crest turn is what displaces the robot: little or no unseen lateral
offset at the descent start, and E1+E2 both met in most runs. Prediction if not: like Arm H. Decided before the data.

## Amendment 4 (after Arm P; before Arm P-fixed) - result of Arm P and the last experiment
Arm P (scoped detector installed; closed-loop turn on the flat platform; n=6): unseen lateral offset at descent start +0.01..+0.02 m
(vs +0.2..+0.4 m open-loop and +1.0..+1.3 m closed-loop on the crest); E1 0/6 failures (max |y| 0.02-0.04 m); genuine full descents
6/6 (ground-truth x 9.9 -> 4.2 m, to the foot of the ramp). The prediction ("little unseen offset, E1+E2 met") held.
Arm P-fixed (decided before data): the same procedure with the UNSCOPED fixed detector temporarily installed (sha 48625b76,
the validated rotation-check-everywhere version), n=6, then restore the scoped detector (sha ea1bcee5) and verify. It answers
only "is the DESCENT-scoped workaround still needed once the robot starts the descent on the centerline with the right heading";
prediction: not needed (E1+E2 met, comparable to Arm P). No other change.
