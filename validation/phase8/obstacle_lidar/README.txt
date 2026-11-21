Live verification of the second (360 deg obstacle-ring) LiDAR added to
robot.urdf.xacro (obstacle_lidar_link, topic /scan_360).

Test world: src/ramp_autonomy_gazebo/worlds/ramp_world_sharp_obstacles_TESTONLY.sdf
  - test_obstacle_side: 0.3x0.3x1.2 m box at (1.0, -1.0, 0.6) - directly
    beside the robot's spawn pose, outside the ramp sensor's forward-only cone.
  - test_obstacle_behind: same size at (0.2, 0, 0.6) - directly behind.
  - Both boxes are 1.2 m tall specifically to cross obstacle_lidar_link's
    own ~0.85 m mount height (a shorter box sits below its scan plane and
    is correctly invisible, same as a real 2D LiDAR missing a short
    obstacle below its mount height - this was a real bug in the first
    version of this test world, height 0.6 m, fixed to 1.2 m).

Results (single live run, world:=sharp_obstacles_TESTONLY):
  /scan_360 behind-obstacle return:  bearing ~180 deg, range 0.64 m
    (true distance: robot at x~1.0, obstacle at x=0.2, minus half the
    obstacle's 0.3 m width = 0.65 m - matches)
  /scan_360 side-obstacle return:    bearing ~-90 deg, range 0.85 m
    (true distance: robot at y=0, obstacle at y=-1.0, minus half-width
    = 0.85 m - matches exactly), spanning ~16 deg of bearing (matches
    the obstacle's expected angular width at that range)
  /local_costmap/costmap: 631 of 14400 cells occupied (>=90) - confirms
    Nav2's ObstacleLayer is genuinely consuming /scan_360 and marking
    real occupancy, not just that the raw scan looks right in isolation.
  /ramp_perception (the OTHER sensor, /scan, unaffected check): slope_deg
    17.68, breakpoint_distance_m 2.55, confidence 0.97 - matches this
    project's long-documented sharp-world baseline (~17.8 deg / ~2.6 m),
    confirming the ramp-detection sensor and ramp_classifier_node.py are
    completely unaffected by the new sensor's addition.
