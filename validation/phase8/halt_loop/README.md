# HALT/recovery self-reinforcement loop: diagnosed with live evidence and fixed (2026-09-26)

Carried forward from `validation/phase8/fsm_multiramp/README.md` 7.4: a single HALT could sustain itself (one 75-HALT thrash in 16 sharp/unloaded trials, a 24-HALT run in the original code).
It had to be resolved before the attach/detach plugin so a new mechanism does not stack on an unexplained feedback loop.

Tools: `start_stop_repro.py` (start-stop reproduction + instrumentation; `INJECT=1` adds an on-demand optimizer reset per leg), `analyze_halts.py`, `slip_recorder.py`, `slip_episodes.py`,
`cell_with_recorder.sh`, `persist_ab.sh`, `ab_summary.py`, `synthetic_slip_control.py`, `blocked_wheels_control.py`, `fsm_slip_persist_test.py`, `full_regression.sh`. Backups: `state_machine_node.PRE_PERSIST.py`
(sha 4db40158, before the fix), `state_machine_node.EXP_SLIPENV.py` and `nav2_param_switcher_node.EXP_NOHALTPUSH.py` (experiment copies, never left installed).

## 1. Diagnosis: what the loop is made of

**Reproduction** (`run1_baseline/`): 24 Nav2/MPPI legs on the flat laydown with a closed-loop 180 deg turn between them (every leg is a start from rest, every leg end a stop; the 75-HALT event
also began at a start from rest after a turn), full stack, final code 4db40158 + 3020997a, wheel command / 4 wheel speeds / slip metric / states / EKF twist recorded at message rate.
Unloaded: 10 HALT entries. Loaded (100 kg): **75 HALT entries, including a natural 17-HALT loop**. Classified by what the robot was doing when the slip metric crossed 0.5:

| context | loaded | what the data show |
|---|---|---|
| onset from rest | 24 | metric exactly **1.00**: the command has started (e.g. a turn command `w = 0.15-0.5 rad/s`), the wheels and the chassis are still at 0 - the wheels lag the command, they are not slipping |
| decel / stop | 20 (10 of 10 unloaded) | command 0.05-0.09 m/s (a small denominator), wheels 0.08-0.16 m/s still spinning down, chassis moving at least as fast as the wheels: ratio 0.6-1.9 |
| loop continuation (< 0.6 s after a HALT exit) | 31 | the restart after a HALT is itself an onset from rest |

**0 of 85 HALTs were physical slip** (wheels turning >= 0.1 m/s while the chassis moves < 50 % of wheel speed); every one is the ratio `|commanded - actual| / |commanded|` (stability_monitor_node.py) being
evaluated while the command is starting, stopping or small. The turn loop happened while MPPI was idle (raw `cmd_vel_nav`), so it does not need optimizer resets or profile pushes.

**The artefacts are one or two samples long** - measured with the slip HALT DISABLED (`FSM_SLIP_HALT=1e9`, experiment FSM copy) so nothing truncates them (`run2_noslip/`, `run3_realslip/`,
`slip_episodes.py`): 213 episodes above 0.5 in 14 runs (the loaded start-stop run; loaded descents on the sharp mu 0.45, curved mu 0.45 and curved normal worlds, 3 each; the sharp world at ramp friction
0.25 and 0.12, 2 each): onset from rest 116 (longest 0.06 s, peak exactly 1.00), decel/stop 60 (longest 0.10 s, peaks 1.4-14), moving 37 (longest 0.09 s). **Nothing longer than 0.10 s, nothing physical.**

**Finding that changes the picture of the slip HALT itself:** even where the robot really slips (ramp friction 0.25 and 0.12, below tan 16 deg), the metric never showed a sustained episode (longest 0.06 s,
0.6-0.9 % of samples above 0.5); a robot driven into an obstacle likewise produced only 0.08 s episodes (the stall detector caught it). The wheels are velocity-controlled and keep tracking the command
whatever the traction, so `|commanded - actual|` does not measure traction loss in this simulator. The phase-6 "live test" of the slip HALT (readings up to 2.08 at mu 0.12) was, on this evidence, the same
onset/decel artefact. So today's slip HALT fires only on transients, never on a real physical condition, and stops the robot each time.

## 2. Which fix: evidence

Both candidates from the hand-over were tested on the same reproduction, loaded, 24 legs, with one on-demand optimizer reset injected per leg (`run4_arms/`, 2 runs per arm; HALT entries):

| arm | run 1 | run 2 |
|---|---|---|
| base (final code before this fix) | 44 | 39 |
| **B: HALT recovery pushes no profile** (switcher ignores HALT; candidate fix 2) | 42 | 29 |
| **fix: persistence-gated slip HALT** (candidate fix 1, below) | **0** | **0** |

B does not remove the loop (the artefacts exist with no push at all, section 1); it only trims what the pushes add. The push chain seen in the 75-HALT thrash (84 `halt_mode` / `flat_mode` pairs) is a
consequence of each HALT, not its cause. Candidate 1 is the correct one - but not as a change of the metric: `/wheel_slip` is also the input of `descent_brake_node` (band 0.08-0.25 of its tuning), so the
metric is left exactly as it is and the artefact is rejected where the HALT decision is made.

## 3. The fix

`state_machine_node.py` (sha a3a9ddd4): the slip HALT needs the metric to stay above 0.5 continuously for `SLIP_HALT_PERSIST_S = 0.25 s` (2.5x the longest artefact ever recorded, 0.10 s). Margin HALT, stall HALT and the
watchdog are untouched and immediate. Decision-function test (`fsm_slip_persist_test.py`, `slip_persist_unit_test.txt`, fake clock): 0.05 s spike, 0.10 s spike, 20 back-to-back onset artefacts and a
0.20 s episode -> no HALT; 0.30 s and 2 s sustained -> HALT exactly 0.25 s after the metric crossed; margin and stall HALT still immediate. Live positive control on the running stack
(`synthetic_slip_control.py`, real stability monitor stopped, synthetic 20 Hz slip): original code HALTs at +0.03-0.09 s on everything >= 0.10 s; fixed code ignores 0.05, 0.10 and 0.20 s and HALTs at +0.28 s (0.3 s
episode) and +0.33 s (2 s episode), releasing when the metric clears.

## 4. Verification

* **Loop propensity** (section 2): 44 / 39 -> 0 / 0 HALTs; the raw metric still crosses 0.5 (2021 samples in one run) - it just no longer stops the robot.
* **Interleaved A/B, PRE (4db40158) vs FIX (a3a9ddd4), fresh launch per trial, arms alternating** (`persist_ab.sh`, `ab_summary.py`), HALT entries per trial:

| cell | PRE | FIX |
|---|---|---|
| sharp / normal / unloaded (8+8; the cell with the 75-HALT thrash) | 1, 5, 6, 3, 3, 4, 3, 8 (1 FAIL) | 1, 1, 1, 1, 1, 1, 1, 1 |
| sharp / normal / loaded (6+6) | 4, 5, 5, 7, 9, 2 (1 FAIL) | 1, 1, 1, 1, 1, 1 |
| sharp / 0.45 / loaded (6+6; the 24-HALT run) | 6, 6, 6, 2, 3, 8 (1 FAIL) | 1, 1, 1, 1, 1, 1 |
| curved / 0.45 / unloaded (6+6) | 0, 3, 2, 0, 5, 1 (2 PASS) | 0, 0, 0, 0, 0, 1 (5 PASS) |
| curved / normal / loaded (6+6) | 4, 3, 3, 0, 2, 20 (1 FAIL, 1 PASS) | 0, 0, 0, 0, 1, 0 (5 PASS) |

  Every descent is reached and ground-truth max \|y\| <= 0.033 m in all 60 trials. The single HALT left in every sharp trial is the motion-stall detector's 5 s HALT at the ramp foot (t ~ 12.7 s), unchanged and
  not part of this item.
* **Full regression with the fix** (`run9_regression/`, guarded, `HG_LOW_RTF=0.85`): 7-cell matrix all ascend + descend (GT max \|y\| <= 0.023 m): **4 PASS, 3 MARGINAL (1 HALT each: the stall HALT), 0 FAIL** (before: 0 PASS,
  5 MARGINAL, 2 FAIL); curved/normal/loaded went `ASCENT > EXIT` in that run (the intended ASCENT fallback when the crest trigger did not fire; the descent was normal). Floor-1 round trips: loaded 108 % goal
  verification / 13 HALTs / margin **0.463** / final GT (2.12, +0.33); unloaded 108 % / 13 / **0.872** / (2.03, +0.22). Full Nav2/MPPI two-ramp trips (5/5 legs verified, GT-vs-EKF <= 0.19 m, 0 mislabelled
  samples, live profile = state's profile in every stable sample incl. `ramp_mode` 124-126/124-126): HALTs unloaded 10, loaded 8 (13 / 12 before). Ramp-2 traversal PASS (6 HALTs, final GT z 6.675).

## 5. What this does and does not change

* Removes the artefact HALTs (onset, decel, loop) that stopped the robot, and with them the HALT/restart loop, in every cell tested. It also removes the 2-8 spurious HALTs per trial the sharp and loaded cells used to log.
* The slip HALT now only fires on a metric sustained for 0.25 s, which in this simulator never happened in any test - **no physical slip or wheel-blocking scenario I could construct is detected by that path**.
  Physical protection comes from the stability margin, the motion-stall detector (which did catch the obstacle push) and the descent brake. If detecting real traction loss matters (e.g. for a payload that can
  slide), the metric needs a different definition (wheel speed vs measured chassis speed) - a separate item, not attempted here.
* Not addressed: with a command held into an obstacle the stall detector itself flaps HALT/EXIT every 0.1 s (`run7_blocked/`): a stall-HALT recovery loop that is inherent to re-issuing the command; unrelated to the slip metric.
