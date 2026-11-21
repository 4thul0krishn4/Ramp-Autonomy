#!/usr/bin/env python3
"""
ramp_classifier_node.py  [PHASE 4]

Consumes /scan (2D LiDAR) and estimates, ahead of the robot rather than
reactively:
    - slope angle of the terrain beyond the nearest detected break point
    - distance to that break point
    - a confidence score

Publishes RampPercept (ramp_autonomy_msgs) on /ramp_perception so
state_machine_node.py can pre-empt its bootstrap IMU-pitch-threshold
trigger with a proper look-ahead.

===================== Sensor geometry this depends on =====================
robot.urdf.xacro's lidar_joint is rolled 90 deg (rpy="1.5708 0 0") rather
than mounted level. A single-plane 2D LiDAR mounted LEVEL can never
produce ground-return points on flat approach ground: every forward beam
at mount height sails straight over flat terrain and only ever hits
something once it rises into the beam's path - at that point you're
seeing the rising face as a wall, not sampling "the ground ahead" at a
range of distances the way a ground-plane fit needs. Rolling the sensor
turns its native horizontal (yaw) sweep into a vertical fore-aft fan
instead: angle=0 in the scan still points straight ahead (+X), negative
angles tip down toward the ground, positive angles tip up. For a ray at
scan angle a with returned range r, the point in the LiDAR's own frame is
simply (x, y) = (r*cos(a), r*sin(a)) - ordinary polar-to-Cartesian, same
as processing any 2D LiDAR scan - which now means x = forward distance
and y = height relative to the sensor. For flat ground at (constant)
height -h below the sensor, different downward angles hit it at
different forward distances but the SAME y = -h, i.e. flat ground is a
zero-slope line in this (x, y) space; an inclined surface is a line
whose slope is exactly tan(incline angle). That's what the fit below
relies on. (Trade-off, documented in the README: the same /scan feeds
Nav2's local_costmap obstacle_layer, which loses its 360 deg horizontal
obstacle ring as a result - a single 2D LiDAR can't serve both roles at
once.)

===================== Algorithm: piecewise (multi-segment) least squares =====
Take the forward cone (mostly downward, a bit of upward margin - see
FORWARD_CONE_MIN/MAX_DEG), convert every valid return to (x, y), sort by
x, and fit up to MAX_SEGMENTS ordinary-least-squares line segments to it
via dynamic programming (see _best_piecewise_fit/_segment_fit), keeping
however many segments (2..MAX_SEGMENTS) the data actually justifies.
That directly gives us both outputs we need - where the current terrain
ends (the first split) and the TRUE slope immediately beyond it (segment
1's own slope, not whatever the last segment converges to) - which is
exactly a change-point problem, not an outlier-rejection problem.

That's also why this isn't RANSAC: RANSAC is built to robustly fit ONE
model while discarding a minority of outlier points, but here the
"second population" (points on the other side of a break point) is the
signal we're looking for, not noise to reject. A single global RANSAC
(or plain OLS) fit would average across every break point and blur
exactly the thing we're trying to localize.

Originally a fixed two-segment search (O(n^2) over the cone's ~100
points). Generalized to a proper multi-segment DP after finding live
that two segments alone converges to the wrong slope on
ramp_world_curved.sdf's filleted (multi-panel, not single-corner) break
points - see _best_piecewise_fit's own docstring for the full evidence
and why MAX_SEGMENTS is set the way it is. The DP is O(MAX_SEGMENTS *
n^2) using O(1)-per-segment cost lookups (_prefix_sums/_segment_fit),
still trivially cheap at 10 Hz for ~100 points.

If even a 2-segment fit isn't meaningfully better than one global line
(no consistent split with enough points on both sides and a real slope
difference), that means there's no break point in view right now -
report breakpoint_distance_m = -1.0 and slope_deg from the single-line
fit instead (e.g. "currently on a uniform incline/flat with nothing
changing ahead"), rather than reporting a split driven by noise.
"""
import math

import rclpy
from rclpy.node import Node
from ramp_autonomy_msgs.msg import RampPercept
from sensor_msgs.msg import LaserScan

# Forward cone, in degrees, of the (post-roll) vertical scan plane to use.
# angle=0 is straight ahead; negative angles look down-and-forward (where
# ground returns come from), positive look up-and-forward. The wide
# downward reach (-80 deg) samples flat ground close to the robot too.
# Widened from the original 20 deg to 45 deg while root-causing the
# ASCENT-stuck bug (see MIN_SEGMENT_POINTS below for the actual fix) -
# tested in isolation and confirmed this alone does NOT fix that bug
# (with MIN_SEGMENT_POINTS left at its old value, 45 deg collapsed at
# the exact same distance 20 deg did), so it is NOT being credited as
# the fix. Kept anyway as a reasonable, verified-harmless margin (no
# accuracy regression on the stationary Zone A baseline below) that
# gives a few extra near-corner points a chance to register, which
# task 2's multi-segment curved-world fit benefits from more directly
# (closely-spaced fillet segments need to see slightly further upward
# to resolve as distinct).
FORWARD_CONE_MIN_DEG = -80.0
FORWARD_CONE_MAX_DEG = 45.0

# Each side of a candidate split needs at least this many points. This
# WAS the actual cause of the ASCENT-stuck bug at its default of 5 -
# see the long comment at _best_two_segment_split for the full,
# live-instrumented evidence chain. Short version: right at a sharp
# break point, the "before the corner" side of the point cloud
# genuinely only has 1-3 valid returns (a real, physical gap in the
# scan, not sensor noise) - MIN_SEGMENT_POINTS=5 made that cluster
# categorically too small to ever be tried as a candidate segment, so
# the search was forced into splits deep inside the single uniform
# surface beyond it, correctly (but unhelpfully) concluding "no break
# point here" earlier than the true break point actually disappears
# from view. Lowered to 3 - the smallest value that still leaves OLS a
# real 1-degree-of-freedom fit rather than a 2-point line - which lets
# the search use that genuine near-corner cluster once it exists,
# confirmed live to extend valid, still-decreasing detections down to
# ~0.2 m from the corner (previously collapsed near ~0.5 m), well
# inside BREAKPOINT_TRIGGER_DISTANCE_M's 0.75 m (state_machine_node.py).
#
# RE-TUNED to 4 (phase 8, ramp LiDAR corner relocation - see
# validation/phase8/corner_mount/README.md): re-validating break-point
# accuracy after moving lidar_joint to a front-left corner mount found a
# real regression at 3 specific to the top (ascent-end) break point -
# the "closer" stop's error went from the baseline's -4 cm to -76 cm,
# and the ASCENT->TRANSITION trigger fired off a spurious near-zero
# reading instead of a genuine ~0.5-0.7 m detection. At the new mount's
# slightly different corner-cluster point density, 3 points was once
# again too permissive and let a small noise-driven cluster get accepted
# as a real segment split, the same failure mode this constant exists to
# prevent, just re-appearing on the opposite side of its tuned range.
# Raising to 4 fixed it live: "closer" error back to -11 cm, "imminent"
# to -10 cm, ASCENT->TRANSITION firing at a sensible ~0.66 m - without
# reopening the original ASCENT-stuck bug at the bottom break point
# (re-confirmed: bottom-approach and curved-world accuracy unchanged or
# improved at 4). The already-known-fragile far-range top approach
# (documented in validation/phase8/perception_regression/README.md as a
# pre-existing weak-signal zone even before this mount change) is still
# unreliable at 4 - outside BREAKPOINT_TRIGGER_DISTANCE_M's 0.75 m use,
# so not a functional regression, but reported plainly rather than
# hidden.
MIN_SEGMENT_POINTS = 4
MIN_SLOPE_DIFF_DEG = 3.0     # near/far slope must differ by at least this to call it a real break point
# Size of the fixed local window (points, not metres - point density
# already falls off with range) used for the reported onset slope - see
# _on_scan's onset_slope comment for why this is a separate, smaller,
# local fit rather than segment[1] of the piecewise DP. Same floor as
# MIN_SEGMENT_POINTS (need a real fit, not 1-2 points) with a bit of
# headroom for stability; kept small deliberately so it stays LOCAL to
# just-past-the-break, matching "onset" rather than drifting into
# whatever's further out.
ONSET_WINDOW_POINTS = 5
# No real surface in either world (sharp or curved) exceeds ~16 deg -
# see _on_scan's plausibility-guard comment for why the onset window
# above needs this backstop against a known near-corner artifact.
PLAUSIBLE_SLOPE_DEG_MAX = 20.0
# Tuned against a live run (world:=sharp, robot stationary in Zone A,
# ~2.6 m from the break point): a clean, accurate detection there (3 cm
# breakpoint-distance error against ground truth) had ~0.039 m RMS
# residual, not the ~0.005-0.01 m a noise-only synthetic scan predicts -
# real gpu_lidar ray noise plus small ray-angle discretization effects
# add up to more scatter than the idealized model. 0.05 was the original
# placeholder guess and scored that good detection at only ~0.22
# confidence; 0.10 was chosen so a residual in that observed range still
# reads as a moderately-confident (~0.6) detection while genuinely
# scattered fits (debris, an edge, too few points) still score low.
RESIDUAL_CONFIDENCE_SCALE = 0.10  # metres of RMS residual at which confidence bottoms out at 0
NO_BREAKPOINT = -1.0

# Max segments the piecewise fit will consider - see _best_piecewise_fit
# for why 2 segments alone converges to the wrong slope on ramp_world_
# curved.sdf's filleted transitions, and _on_scan's k-selection loop for
# how this ceiling is actually used (it's a ceiling, not a fixed target -
# fewer segments are used whenever fewer are justified by the data).
# ramp_world_curved.sdf's own geometry sets this number: each break
# point there is built from 3 discrete flat fillet panels (not 1 sharp
# corner), so resolving "the current segment" + "the true next-onset
# segment" distinctly from "everything further, still blended" needs at
# least 3 segments - a 4th is allowed as headroom (e.g. if two fillet
# panels are both clearly resolvable at once) without being required.
MAX_SEGMENTS = 4
# k-selection uses BIC (Bayesian Information Criterion), not a fixed
# RSS-improvement ratio - found live that a ratio-based bar (originally
# tried: require each extra segment to cut RSS by >=15%) was NOT strict
# enough: with only ~0.01-0.04 m of real scan noise (see
# RESIDUAL_CONFIDENCE_SCALE's own tuning note) and candidate segments as
# small as MIN_SEGMENT_POINTS=3 points, the DP could always find SOME
# partition of flat, featureless ground that trims RSS by 15%+ just by
# fitting a few points' worth of noise as a fake tiny-slope "segment" -
# confirmed live: a spurious ~2-point segment reading -1 to -2 deg
# appeared BETWEEN the real flat segment and the real fillet-onset
# segment while the robot was still well out on flat ground, nowhere
# near the fillet. A flat percentage bar can't tell "real structure" from
# "a good noise fit" apart, because both just look like "RSS went down."
# BIC can: it charges each extra segment's 2 extra free parameters
# (slope, intercept) a fixed cost in log-likelihood units (BIC_PENALTY *
# ln(n) per segment, the standard BIC form, not re-derived or tuned
# here), so an extra segment only wins if it explains materially more
# variance than 2 more free parameters would explain by chance alone -
# exactly the model-selection question "how many real change points are
# in this data," which is what a fixed RSS ratio was only approximating.
# Confirmed live this correctly rejects the noise-driven segment above
# while still accepting the real 3-segment fillet structure (see README).
BIC_PENALTY = 1.0


def _ols_fit(points):
    """Ordinary least squares line fit y = slope*x + intercept.

    Returns (slope, intercept, rms_residual). points is a list of (x, y).
    Falls back to a flat (slope=0) fit if x has ~zero variance (can't
    determine a slope from a single forward distance).
    """
    n = len(points)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x_mean = sum(xs) / n
    y_mean = sum(ys) / n
    var_x = sum((x - x_mean) ** 2 for x in xs)
    if var_x < 1e-6:
        slope = 0.0
    else:
        cov_xy = sum((x - x_mean) * (y - y_mean) for x, y in points)
        slope = cov_xy / var_x
    intercept = y_mean - slope * x_mean
    residual_sq_sum = sum((y - (slope * x + intercept)) ** 2 for x, y in points)
    rms_residual = math.sqrt(residual_sq_sum / n)
    return slope, intercept, rms_residual


def _confidence_from_residual(rms_residual: float) -> float:
    return max(0.0, min(1.0, 1.0 - rms_residual / RESIDUAL_CONFIDENCE_SCALE))


def _prefix_sums(points):
    """Running sums of 1, x, y, x^2, xy, y^2 - lets any contiguous
    segment's OLS fit be computed in O(1) instead of O(segment length),
    which is what makes the piecewise DP below O(MAX_SEGMENTS * n^2)
    instead of O(MAX_SEGMENTS * n^3)."""
    n = len(points)
    px = [0.0] * (n + 1)
    py = [0.0] * (n + 1)
    pxx = [0.0] * (n + 1)
    pxy = [0.0] * (n + 1)
    pyy = [0.0] * (n + 1)
    for i, (x, y) in enumerate(points):
        px[i + 1] = px[i] + x
        py[i + 1] = py[i] + y
        pxx[i + 1] = pxx[i] + x * x
        pxy[i + 1] = pxy[i] + x * y
        pyy[i + 1] = pyy[i] + y * y
    return px, py, pxx, pxy, pyy


def _segment_fit(prefix, i, j):
    """OLS slope/intercept/RSS (residual SUM of squares, not RMS - the
    DP below needs an additive cost) for points[i:j], from precomputed
    prefix sums. Same slope=0 fallback as _ols_fit for near-zero x
    variance. Standard identity used here: for the least-squares line,
    RSS = Syy - slope*Sxy (Syy, Sxy = sums of squared/cross deviations),
    which needs each segment's stats only, not the individual points -
    that's what makes this O(1) per call.
    """
    px, py, pxx, pxy, pyy = prefix
    cnt = j - i
    sx = px[j] - px[i]
    sy = py[j] - py[i]
    sxx = pxx[j] - pxx[i]
    sxy = pxy[j] - pxy[i]
    syy = pyy[j] - pyy[i]
    x_mean = sx / cnt
    y_mean = sy / cnt
    var_x = sxx - x_mean * sx
    var_y = syy - y_mean * sy
    if var_x < 1e-6:
        slope = 0.0
        rss = var_y
    else:
        cov_xy = sxy - x_mean * sy
        slope = cov_xy / var_x
        rss = max(0.0, var_y - slope * cov_xy)
    intercept = y_mean - slope * x_mean
    return slope, intercept, rss


class RampClassifier(Node):
    def __init__(self):
        super().__init__('ramp_classifier')
        self._percept_pub = self.create_publisher(RampPercept, '/ramp_perception', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.get_logger().info(
            f'Ramp classifier running: forward cone [{FORWARD_CONE_MIN_DEG}, {FORWARD_CONE_MAX_DEG}] deg'
        )

    def _on_scan(self, msg: LaserScan):
        points = self._extract_cone_points(msg)

        percept = RampPercept()
        percept.header = msg.header

        if len(points) < 2 * MIN_SEGMENT_POINTS:
            # Not enough valid returns in the cone to say anything useful
            # (e.g. everything out of range, or a scan glitch).
            percept.slope_deg = 0.0
            percept.breakpoint_distance_m = NO_BREAKPOINT
            percept.confidence = 0.0
            self._percept_pub.publish(percept)
            self.get_logger().debug(f'too few points in cone: {len(points)}')
            return

        points.sort(key=lambda p: p[0])  # by forward distance x - required for segmentation

        single_slope, _, single_residual = _ols_fit(points)
        segments = self._best_piecewise_fit(points)

        if segments is not None and len(segments) >= 2:
            split_idx, near_slope = segments[0][1], segments[0][2]
            split_x = points[split_idx - 1][0]  # last point of segment 0 = the break point's forward distance
            # Onset slope comes from a small, FIXED-size local window of
            # points immediately after the split, not from segment[1] as
            # the DP/BIC happened to draw it. Why: the true first fillet
            # panel (e.g. 5.33 deg) changes height by only ~3 cm over its
            # 0.3 m run - comparable to or smaller than real scan noise
            # (~1-2 cm RMS, see RESIDUAL_CONFIDENCE_SCALE's tuning note),
            # so BIC (correctly, by design) often won't spend 2 more free
            # parameters isolating it as its own segment - confirmed live,
            # it kept merging those points into segment[1] together with
            # the next, steeper panel, converging back toward ~15-18 deg
            # instead of the true ~5 deg onset even after the DP fix
            # above. A local window fit answers a different, better-posed
            # question for this specific number - not "how many distinct
            # segments does BIC think are here" but "what's the trend
            # right at the point where terrain starts changing" - and
            # isn't at the mercy of a segment-count decision that's
            # legitimately hard to make near the noise floor.
            onset_end = min(split_idx + ONSET_WINDOW_POINTS, len(points))
            window_slope, _, window_rms = _ols_fit(points[split_idx:onset_end])
            onset_n = onset_end - split_idx
            # Plausibility guard, found live: the small local window above
            # fixed the curved-world onset-angle bug, but on the SHARP
            # world it regressed the already-accurate stationary-Zone-A
            # baseline (slope_deg swung to -78 deg) - a small, close-in
            # window is exactly what phase 4's own README already
            # documents as vulnerable to a real artifact: "a downward ray
            # grazing [a sharp, non-filleted] seam at a shallow angle can
            # catch a sliver of the ramp box's end face, which is a
            # near-vertical surface." No real surface in either world
            # exceeds ~16 deg, so any reading steeper than
            # PLAUSIBLE_SLOPE_DEG_MAX is far more likely to be this
            # artifact than real terrain.
            # A single fallback to segment[1] isn't enough - confirmed
            # live the DP can isolate the seam artifact as ITS OWN
            # segment[1] too (a clean, tight 2-3 point fit at ~77 deg
            # spanning under 2 cm of x - a real, distinct feature from the
            # DP's point of view, just not the one we want), with the true
            # ramp surface only showing up correctly in segment[2] or
            # segment[3].
            # Two-tier, not "first plausible in sequence": trust the local
            # window FIRST if it's plausible (that's the whole fix for the
            # curved world - a small window is deliberately preferred over
            # any larger, BIC-merged segment there, since larger segments
            # are exactly what converges back to the wrong, blended peak
            # angle). Only once the window itself is implausible (the
            # sharp-world artifact case) fall back to the DP segments - and
            # there, pick whichever PLAUSIBLE segment has the MOST points,
            # not just the first one in sequence: confirmed live that
            # "first plausible" can land on another small (2-3 point),
            # only-plausible-by-luck transitional segment (e.g. a genuine
            # 8 deg reading from 3 points) instead of the true, much
            # better-supported surface (17.9 deg from 10 points) sitting
            # right after it. A small segment being plausible is as often
            # coincidence as it is signal; a larger one agreeing with the
            # plausibility bound is real corroborating evidence - the same
            # reasoning this file already applies via MIN_SEGMENT_POINTS.
            if abs(math.degrees(math.atan(window_slope))) <= PLAUSIBLE_SLOPE_DEG_MAX:
                onset_slope, onset_rms = window_slope, window_rms
            else:
                fallback_candidates = [
                    (sl, math.sqrt(rss / (e - s)), e - s) for (s, e, sl, ic, rss) in segments[1:]
                ]
                plausible = [c for c in fallback_candidates if abs(math.degrees(math.atan(c[0]))) <= PLAUSIBLE_SLOPE_DEG_MAX]
                if plausible:
                    onset_slope, onset_rms, _ = max(plausible, key=lambda c: c[2])
                else:
                    onset_slope, onset_rms = single_slope, single_residual  # every candidate implausible
            onset_slope_deg = math.degrees(math.atan(onset_slope))
            slope_diff_deg = abs(onset_slope_deg - math.degrees(math.atan(near_slope)))
            accepted = onset_n >= MIN_SEGMENT_POINTS and slope_diff_deg >= MIN_SLOPE_DIFF_DEG
            seg_str = ' '.join(
                f'[{s}:{e} n={e-s} slope={math.degrees(math.atan(sl)):.1f} x=({points[s][0]:.2f}-{points[e-1][0]:.2f})]'
                for (s, e, sl, ic, rss) in segments
            )
            self.get_logger().debug(
                f'{len(segments)}-seg n={len(points)} split_x={split_x:.3f} onset_window=[{split_idx}:{onset_end}] '
                f'onset_slope={onset_slope_deg:.1f} diff={slope_diff_deg:.1f} '
                f'accepted={accepted} segments={seg_str}'
            )
            if accepted:
                percept.breakpoint_distance_m = float(split_x)
                percept.slope_deg = float(onset_slope_deg)
                percept.confidence = _confidence_from_residual(onset_rms)
                self._percept_pub.publish(percept)
                self.get_logger().debug(
                    f'break point at {split_x:.2f} m, near={math.degrees(math.atan(near_slope)):.1f} deg, '
                    f'onset={percept.slope_deg:.1f} deg, confidence={percept.confidence:.2f}'
                )
                return
        else:
            self.get_logger().debug(f'no candidate multi-segment fit beat a single line: n={len(points)}')

        # No convincing break point in view - report the overall slope of
        # whatever uniform terrain is ahead instead.
        percept.breakpoint_distance_m = NO_BREAKPOINT
        percept.slope_deg = float(math.degrees(math.atan(single_slope)))
        percept.confidence = _confidence_from_residual(single_residual)
        self._percept_pub.publish(percept)

    def _extract_cone_points(self, msg: LaserScan):
        angle_min_rad = math.radians(FORWARD_CONE_MIN_DEG)
        angle_max_rad = math.radians(FORWARD_CONE_MAX_DEG)
        points = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < angle_min_rad or angle > angle_max_rad:
                continue
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r >= msg.range_max - 1e-3:
                continue  # below min range, or "no return" (reported at/beyond max range)
            points.append((r * math.cos(angle), r * math.sin(angle)))
        return points

    def _best_piecewise_fit(self, points):
        """Optimal piecewise-OLS partition of `points` (sorted by x) into
        2..MAX_SEGMENTS contiguous segments, found by dynamic programming
        (see _prefix_sums/_segment_fit for the O(1)-per-segment cost this
        relies on) - generalizes the original two-segment split search
        below to more than one change point.

        Returns a list of segments [(start_idx, end_idx, slope,
        intercept, rss), ...] for however many segments the data
        justified (see the k-selection loop below), or None if not even
        2 segments meaningfully beat a single line.

        ===== Why more than 2 segments: curved-world onset-angle bug =====
        ramp_world_curved.sdf's break points are each built from 3
        discrete flat fillet panels stepping through intermediate angles
        (e.g. 0 -> 5.33 -> 10.67 -> 16 deg), not one sharp corner. A
        2-segment fit can only ever report ONE "far" slope for
        everything past its single split, so approaching that filleted
        transition it was reporting `slope_deg` around 15-16 deg - the
        ramp's ultimate PEAK angle - even while the robot was still on
        flat ground, nowhere near the fillet yet (confirmed live: logged
        readings of 14.5-16.6 deg for the entire flat approach, when the
        true immediate onset the robot was about to encounter was only
        5.33 deg). The 2-segment "far" fit was silently blending all 3
        fillet panels (and beyond) into one slope, dominated by whichever
        panels contribute the most y-variation to that blend - not
        necessarily the correct nearest one.
        Fitting up to MAX_SEGMENTS segments (a target informed directly by
        the curved world's own 3-panel fillet construction, not picked
        arbitrarily) is necessary for a real fix but turned out NOT
        sufficient on its own: the true first fillet panel's height
        change (~3 cm over 0.3 m) is comparable to real scan noise, so
        BIC often can't justify isolating it as its own segment[1]
        either, still blending it with steeper panels beyond. `_on_scan`
        reports the onset slope from a small local window right after the
        split instead (decoupled from however many segments BIC settled
        on), with a plausibility fallback into these DP segments for the
        sharp world's own close-range artifact - see that function's own
        long comment for the full story and why neither piece works
        alone. `breakpoint_distance_m` still comes from the first split
        (unchanged semantics: "where does the current uniform terrain
        end").

        ===== Root cause of the ASCENT-stuck bug, found live =====
        The originally-reported symptom (breakpoint_distance_m freezing
        instead of counting down, stranding ASCENT -> TRANSITION) was
        root-caused by instrumenting this exact search tick-by-tick
        (point count, x-range, chosen split, accepted/rejected) during a
        real climb, rather than guessing from the existing logs. Ruled
        out first, by inspection: a stale point buffer (there isn't
        one - `points` comes fresh from the current scan every call,
        nothing is cached across ticks) and a search stuck in a local
        minimum (this loop is exhaustive over every candidate split, not
        iterative/gradient-based, so there's no minimum to get stuck in).

        What the instrumented data actually showed, approaching a sharp
        break point: `breakpoint_distance_m` counted down smoothly and
        accurately (confirmed against ground truth) until the robot got
        within roughly half a metre of the real corner. At that point
        the raw (x, y) point dump showed the true structure: a small
        (1-3 point) cluster right at the corner, then a genuine GAP in
        returns (a real feature of the scan - the corner occludes
        whatever is just beyond it from this close, a normal outcome for
        a single-plane rangefinder near a convex crest, not sensor
        noise), then a long, uniform run of points still on the current
        surface. With MIN_SEGMENT_POINTS=5, that near-corner cluster was
        never large enough to be tried as a candidate segment on its
        own, so every candidate split fell entirely inside the long
        uniform run - correctly finding no slope difference there
        (there IS no break point structure inside a single uniform
        surface) and reporting NO_BREAKPOINT well before the robot
        actually got close enough to lose the corner cluster for real.
        Lowering MIN_SEGMENT_POINTS to 3 lets the search use that
        genuine small cluster once it exists, confirmed live to extend
        valid detections much closer to the true corner. Some final,
        genuinely unresolvable blind radius remains right at the corner
        itself (once the near-corner cluster disappears entirely, there
        is no data left to reconstruct it from, for any threshold) -
        that residual gap is exactly why state_machine_node.py's IMU
        pitch-rate fallback and ASCENT-completeness edge still exist.
        """
        n = len(points)
        prefix = _prefix_sums(points)
        max_k = min(MAX_SEGMENTS, n // MIN_SEGMENT_POINTS)
        if max_k < 2:
            return None  # not even enough points for a 2-segment split

        INF = float('inf')
        # dp[k][m]: min total RSS partitioning points[0:m] into k segments.
        # back[k][m]: start index of that partition's k-th (last) segment,
        # for reconstructing the actual split points afterward.
        dp = [[INF] * (n + 1) for _ in range(max_k + 1)]
        back = [[None] * (n + 1) for _ in range(max_k + 1)]
        dp[0][0] = 0.0
        for k in range(1, max_k + 1):
            for m in range(k * MIN_SEGMENT_POINTS, n + 1):
                best_cost, best_s = INF, None
                for s in range((k - 1) * MIN_SEGMENT_POINTS, m - MIN_SEGMENT_POINTS + 1):
                    if dp[k - 1][s] == INF:
                        continue
                    _, _, seg_rss = _segment_fit(prefix, s, m)
                    total = dp[k - 1][s] + seg_rss
                    if total < best_cost:
                        best_cost, best_s = total, s
                dp[k][m] = best_cost
                back[k][m] = best_s

        def reconstruct(k):
            if dp[k][n] == INF:
                return None
            bounds = [n]
            cur = n
            for kk in range(k, 0, -1):
                cur = back[kk][cur]
                bounds.append(cur)
            bounds.reverse()  # [0, s1, s2, ..., n]
            return [
                (bounds[i], bounds[i + 1]) + _segment_fit(prefix, bounds[i], bounds[i + 1])
                for i in range(k)
            ]

        # Choose k by BIC, not a fixed RSS-improvement ratio - see
        # BIC_PENALTY's own comment for why a ratio isn't strict enough
        # (it can't distinguish real structure from a good noise fit).
        # BIC(k) = n*ln(RSS(k)/n) + BIC_PENALTY*2*k*ln(n) - each extra
        # segment's 2 extra free parameters (slope, intercept) has to
        # earn back its fixed penalty in reduced RSS or it isn't worth
        # it. floor_rss guards ln(0) for a near-perfect fit.
        floor_rss = 1e-9
        log_n = math.log(n)
        best_bic = n * math.log(max(_segment_fit(prefix, 0, n)[2], floor_rss) / n)
        chosen_k = 1
        for k in range(2, max_k + 1):
            if dp[k][n] == INF:
                continue
            bic = n * math.log(max(dp[k][n], floor_rss) / n) + BIC_PENALTY * 2 * k * log_n
            if bic < best_bic:
                best_bic = bic
                chosen_k = k

        if chosen_k < 2:
            return None
        return reconstruct(chosen_k)


def main(args=None):
    rclpy.init(args=args)
    node = RampClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
