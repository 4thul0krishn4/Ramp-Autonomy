# Phase 4 perception output - see ramp_classifier_node.py for how these
# are computed from /scan. All fields are in the LiDAR's own local frame
# (header.frame_id), not a global/world frame.

std_msgs/Header header

# Slope, in degrees, of the terrain segment ahead of the detected break
# point (or of the whole forward cone, if no break point was found - see
# breakpoint_distance_m). This is RELATIVE TO THE ROBOT'S CURRENT PITCH,
# not an absolute world-frame angle: 0 means "continues at the same pitch
# the robot is on right now", not "flat ground".
float32 slope_deg

# Estimated forward distance (metres, from the LiDAR) to the nearest
# detected slope break point. -1.0 means no clear break point was found
# within the sensor's forward cone (e.g. still far from the ramp, or
# already on a long uniform segment with nothing changing ahead).
float32 breakpoint_distance_m

# 0.0-1.0 confidence, derived from how well the fitted line(s) match the
# actual scan points (low residual = high confidence). Low confidence
# means "don't trust slope_deg/breakpoint_distance_m this cycle" - e.g.
# too few valid returns, or a scattered/noisy fit.
float32 confidence
