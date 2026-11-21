# Perception re-checks after the ramp-geometry fix (2026-09-20) - break-point accuracy and obstacle-LiDAR

Scope: re-run the two checks that were skipped after the 0.05 m ramp-slab raise (`../crest_freeze/README.md`), same method as their original validation, guarded clean host (RTF 0.998-0.999),
plus an A/B on the ORIGINAL (unfixed) geometry on today's stack (control worlds `ramp_world_sharp_origgeom_TESTONLY`, `ramp_world_sharp_obstacles_origgeom_TESTONLY`, `ramp_world_curved_mu045_origgeom_TESTONLY`) so that any difference can be attributed to the geometry rather than to stack drift since the original numbers were recorded.
Scripts: `breakpoint_accuracy.py`, `obstacle_lidar_check.py`, `run_perception_checks.sh`, `run_perception_ab.sh`, `run_costmap_diff.sh`. Data: `*.json`, `ab_*.json`, `cm_*`.

## 1. Break-point detection accuracy - PASS at the operative standard (two changes noted)
Ground truth = marker x (4.0 bottom / 7.5 top) - (Gazebo chassis x + 0.35 m LiDAR offset), robot stopped at each position (original used /odom).
| Stop (sharp world) | original doc (error) | corrected geometry | original geometry, today (A/B) |
|---|---|---|---|
| Zone A, spawn (true 2.65 m) | -3 cm | -9.1 cm (2.559 m), slope 17.0 deg, conf 0.64 | -9.2 cm, 17.8 deg, 0.68 |
| Zone A, close (true ~0.30 m) | +4.3 cm | +3.3 cm, slope 15.4 deg, conf 0.94 | +4.9 cm, 14.9 deg |
| On ramp, top ahead, far (~2.1 m) | +9 cm | **-37.3 cm, only 5/30 samples valid** | -18.3 cm (28/30 valid) |
| On ramp, top ahead, closer (~1.1 m) | +9 cm | -4.0 cm | -5.9 cm |
| On ramp, top ahead, imminent (~0.6 m) | ~+4 cm | -4.6 cm, conf 0.87 | -4.3 cm |
| Curved: spawn (true 2.65) / close (true 1.40) | +44 cm / -5 cm | +10.6 cm / +9.4 cm | (control not run) |
* Distance errors at every stop within ~10 cm except the far top approach; bottom-approach slope within 2 deg of the true 16 deg (17.0 / 15.4). The spawn value (2.56 m) equals what the original geometry gives today (2.56 m) and the obstacle-test's own earlier note (2.55 m): stack drift, not geometry.
* **State-machine use verified:** `ASCENT -> TRANSITION` fired from a genuine `breakpoint_imminent` at 0.524 m, confidence 0.98 (original geometry today: 0.489 m, 0.96) - inside the 0.75 m trigger, the only range the FSM consumes.
* **Change 1 (improvement):** the top-approach slope artifact is gone. The +0.05 m step presented a near-vertical end face (original doc 70-75 deg; 59.7 deg on the original geometry today at the imminent stop); on the corrected geometry it reads -19.1 deg, the physically sensible value for a flat platform ahead of a robot pitched -16 deg.
* **Change 2 (degradation, far range only):** at ~2.1 m before the top break point the detector reports intermittently (5/30 valid, -37 cm when valid) on the corrected geometry, vs 28/30 valid and -18 cm on the original geometry today. The +0.05 m step was a stronger change-point than the now-continuous crest; this is the already-documented far-range limitation (weak change-point signal / too few far-side points), outside the FSM's 0.75 m use. The original +9 cm figure is not reproducible on today's stack even on the original geometry (-18 cm).

## 2. Obstacle-LiDAR (`world:=sharp_obstacles_TESTONLY`) - PASS (one measured change)
| Check | original standard | corrected geometry | original geometry, today |
|---|---|---|---|
| behind obstacle (true 0.65 m) | bearing ~180 deg, 0.64 m | 0.641 m at 181 deg (-179) | 0.635 m at 174 deg |
| side obstacle (true 0.85 m) | bearing ~-90 deg, 0.85 m, span ~16 deg | 0.838 m, span 18.1 deg | 0.834 m, span 18.1 deg |
| `/ramp_perception` at rest | 17.68 deg / 2.55 m / 0.97 (2.61-2.63 m in the accuracy table) | 17.02 deg / 2.555 m / 0.64 | 17.73 deg / 2.565 m / 0.54 |
| local costmap cells >= 90 | 631 of 14400 | **854 and 737 (two runs)** | 631 and 638 (two runs) |
* Raw returns, bearings, ranges and span match the true geometry to ~1 cm on both geometries; `/ramp_perception` is unaffected by the second sensor; the costmap consumes `/scan_360` and marks the obstacles (the original test's actual pass condition).
* **Change (geometry-attributable, no functional impact):** the occupied-cell count is 100-220 higher on the corrected geometry (the original geometry reproduces the documented 631 exactly). The difference is a coherent blob of ~107 cells at world x ~3.6-4.0, y ~1.2-2.0, i.e. beside the ramp foot on the left, where the raised ramp rail's low end sits 0.05 m higher (likely now within the level ring's scan height; the ring height was not separately measured - hypothesis, not a finding). It is real geometry the robot can now see, inside the window's edge; nothing consumed it differently in any run (60+ reverification runs and both round trips).

## Verdict
Break-point accuracy: **PASS** (operative range; far-range top-approach reading degraded, top-approach slope artifact removed). Obstacle-LiDAR: **PASS** (returns and costmap behaviour unchanged; occupied-cell count differs on the geometry, explained above).
