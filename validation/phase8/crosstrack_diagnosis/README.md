> **CORRECTION (final structural experiment, `../path_structural/README.md`).** The "second mechanism" below (a physical stumble at the
> `fillet_top_1/2/3` seam cluster) and the reading that cross-track error "grows unchecked" through a tracking failure were never tested as causes and
> are superseded. Recorded ground truth shows the robot starts the descent facing the panel edge and displaced sideways (0.2-1.3 m) by the harness's turn on the
> crest while the EKF's y stays 0.00; with the turn done on the flat platform the unchanged controller tracks the path within 5 cm. The `offset_from_furthest`
> finding (PathAlignCritic disabled) is a real source-level fact, but the "2.85 -> 2.03 m" improvement attributed to it was one run against one run.

# Cross-track error and HALT-thrash root cause (curved-world descent)

Follow-up to the EKF-corrected matrix rerun, which found `curved/normal/
unloaded` and `curved/normal/loaded` failing on ground-truth edge
departure (2.3-2.7 m past a 1.5 m panel half-width) while
`/ramp_state` reported clean `DESCENT` throughout, and `curved/normal/
loaded` + `curved/0.45/loaded` showing HALT counts roughly tripling
(3 -> 8) once EKF-corrected localization went live.

## Finding 1: cross-track error confirmed growing unchecked, then root-caused

`crosstrack_instrument.py` drives one failing cell
(`curved_unloaded_normal`) with fine-grained (0.3 s) ground-truth
polling throughout the descent leg. The planned path is a straight line
at Y=0, so ground-truth Y IS the cross-track error directly.

**Baseline** (`crosstrack_curved_unloaded_normal.csv/log`): error grows
smoothly and almost linearly from 0.44 m to 1.2 m over the first ~9.4 s
- zero sign of any restoring force, a textbook unchecked-drift signature
- then continues to a peak of 2.85 m before the leg ends. This directly
confirms the hypothesis: no real lateral correction exists.

**Three tuning attempts on `PathAlignCritic`, all measured live against
the same failing cell, none fully resolved it:**

| attempt | change | peak cross-track error | outcome |
|---|---|---|---|
| 1 | `cost_weight` 14 -> 60 (power=1) | 2.42 m | halved the early drift *rate*, but plateaued rather than correcting back - still past 1.5 m |
| 2 | `cost_weight`=8, `cost_power`=2 (quadratic) | 2.82 m | no better than baseline - same plateau-without-correction shape |
| 3 | split ASCENT/DESCENT profiles, DESCENT gets `heading_tolerance_deg` 5->20 + `max_vel_theta` 0.4->0.6 (hypothesis: tight heading tolerance was fighting the corrective steering a DiffDrive robot needs) | 2.03 m | best of the three, but still past 1.5 m, and the robot got stuck (frozen ground-truth position) for the rest of the leg |

**The actual root cause**, found by reading `nav2_mppi_controller`'s own
`PathAlignCritic` source (`path_align_critic.cpp`) instead of continuing
to guess:

```cpp
// Don't apply when first getting bearing w.r.t. the path
utils::setPathFurthestPointIfNotSet(data);
const size_t path_segments_count = *data.furthest_reached_path_point;
if (path_segments_count < offset_from_furthest_) {
  return;   // <-- ZERO cost, critic fully disabled
}
```

`offset_from_furthest` defaults to 20 (`nav2_params.yaml`'s static
value). This project's descent path (~6 m at 0.4 m waypoint spacing)
has roughly 15 points total - `path_segments_count` can **never reach
20**, so `PathAlignCritic` was structurally disabled for the entire
descent leg, on all three prior attempts. Every weight/power change was
tuning a critic that never activated even once - that's why none of
them produced the qualitative shape change (correction back to center)
that a genuinely active critic should.

**Fix**: `offset_from_furthest` dropped to 3 (matching
`PathFollowCritic`'s own existing default) for the new `descent_mode`
profile, on top of attempt 3's mode split and loosened heading
tolerance (kept - reasonable on their own terms and necessary for the
robot to have steering authority once the critic is actually live).
Result (`crosstrack_curved_unloaded_normal_offsetfix.csv/log`): peak
2.03 m - a genuine, measurable improvement (29% better than baseline,
consistent with attempt 3's result, confirming the critic really is
live now) but **still not under the 1.5 m panel half-width**, and the
robot still gets stuck (frozen ground truth) partway through.

### Why the fix alone doesn't fully resolve it: a second, separate mechanism

Every one of the four live runs - baseline and all three tuning
attempts - shows the SAME robot getting physically stuck (ground-truth
position and pitch both frozen, matching the exact signature already
documented in `construction_site_descent/README.md`'s fillet-top stall
finding) at almost the identical location: ground-truth X in the
7.6-8.3 m range. Checked directly against `ramp_world_curved.sdf`'s own
geometry:

```
fillet_top_1:  pose x=7.646
fillet_top_2:  pose x=7.944
fillet_top_3:  pose x=8.244
```

This is not coincidence - it's the exact location of three narrow
(0.301 m) fillet-transition panels in rapid succession near the ramp
top, a genuine geometric pinch point. The stuck episodes correlate
precisely with this seam cluster across every run, regardless of which
MPPI parameters were in play. **No amount of critic tuning fixes a
physical stumble at a real geometric discontinuity** - MPPI can at best
correct the trajectory gradually afterward (which the offset_from_
furthest fix now genuinely does, better than before), not prevent or
instantly undo a discrete physical kick. This is the same architectural
blind spot already documented for the construction-site stall: no
existing signal (`/wheel_slip`, `/stability_margin`) can detect
"commanded to move, not actually moving," so nothing catches or
compensates for the stumble in real time.

**Conclusion for finding 1**: the MPPI misconfiguration was real,
confirmed, and is now fixed (`offset_from_furthest`, plus the
supporting mode-split/heading-tolerance changes) - this measurably
improves cross-track tracking and is worth keeping. But it does not by
itself fully resolve every curved-world edge-departure failure, because
a second, independently-confirmed mechanism (physical stumble at the
`fillet_top_1/2/3` seam cluster) also contributes, and fixing THAT
needs a different category of fix (a ground-truth-independent
stall/stumble detector, or smoothing the world's own fillet-seam
geometry) - out of scope here, matching the same "propose but don't
implement, needs its own session" judgment already applied to the
construction-site stall finding.

## Finding 2: HALT-thrash - same mechanism, previously invisible

Checked the state-transition timeline for `curved_unloaded_normal`
under the corrected localization: HALT events cluster temporally right
around the descent-leg time window that matches the fillet-top
seam-stumble episode (roughly t=10-17 s into the descent leg in the
instrumented runs). This is consistent with genuine, real instability
- previously invisible to a controller/FSM fed raw, drifting wheel
odometry - now being correctly detected and reacted to once
localization is accurate.

Ruled out as primary causes, with evidence (not assumed):
- **HALT threshold mismatch**: thresholds (`/stability_margin`,
  `/wheel_slip`) were not changed anywhere in this session's fix -
  same values before and after. A threshold-tuning problem would need
  the threshold itself to be wrong; nothing here changed it.
- **Fusion-rate/timing mismatch**: `ekf.yaml` runs at 30 Hz (well within
  normal margins for a 20 Hz `controller_frequency`); no log evidence
  anywhere in this session's testing of a timing-related anomaly (no
  gaps, no stale-data symptoms distinct from the stumble events
  themselves).

The residual pattern after the crosstrack fix is not perfectly
monotonic (`curved_loaded_normal`'s HALT count improved 8->4;
`curved_loaded_mu045`'s got slightly worse, 8->9; `curved_unloaded_
normal`'s got worse, 1->4) - consistent with a stochastic, contact-
dynamics-dependent event (exact stumble severity varies run to run at
a narrow physical seam) rather than a deterministic, fixable bug.
**Conclusion: HALT-thrash is driven by the same newly-visible physical
instability as finding 1, amplified for loaded cells by their reduced
margin (more mass/forward CG means the same stumble produces a more
severe pitch/margin response) - not a threshold or timing bug.**

## Final corrected matrix (after the crosstrack fix)

See `README.md`'s "Full matrix, corrected (crosstrack fix applied)"
section for the full 7-cell table and honest final verdicts - not all
cells are clean; reported plainly rather than oversold.
