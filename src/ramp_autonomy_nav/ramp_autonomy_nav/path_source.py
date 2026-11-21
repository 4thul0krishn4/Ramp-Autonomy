"""
path_source.py

Path construction for FollowPath goals (Nav2 MPPI). Two things live here:

  * make_straight_path(): the straight centerline used for the curved-world
    ramp DESCENT, with an explicit lateral constraint - every waypoint is
    pinned to `y` (the panel centerline in the odom frame) - and an explicit
    waypoint spacing.
  * densify_path(): insert interpolated waypoints into any existing Path so
    that no two consecutive waypoints are further apart than `max_spacing`,
    keeping every ORIGINAL waypoint exactly (so the polyline is unchanged;
    only the sampling of it is finer).

Why waypoint density matters to this controller (read from nav2_mppi_controller's
own source, headers verified identical to the installed Jazzy build):
  * PathHandler prunes the path handed to the critics to `prune_distance` (1.7 m)
    ahead of the robot, so at 0.4 m spacing the critics see ~4-5 points.
  * PathAlignCritic (a) returns zero cost unless the trajectories' furthest
    reached path index >= offset_from_furthest, and (b) scores each trajectory
    point against the path point at the SAME integrated arc length, taken from
    the discrete waypoints - so its lateral signal is quantized/noisy at
    waypoint spacing.
  * MPPI's horizon (56 steps x 0.05 s at <= 0.35 m/s on the ramp) only reaches
    ~1 m ahead, i.e. ~2 waypoints at 0.4 m spacing.
Denser waypoints make every point-index-denominated critic parameter
(offset_from_furthest, prune window, integrated-distance matching) act on a finer
lattice. It changes what those point counts MEAN (offset 3 is 1.2 m at 0.4 m
spacing but 0.15 m at 0.05 m spacing), which is a property of the change, not a
side effect to be ignored - see validation/phase8/path_structural/README.md.

No ROS node is needed here: these helpers only build nav_msgs/Path messages.
"""
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

DEFAULT_SPACING_M = 0.4     # the spacing every earlier validation run used


def _yaw_of(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _shortest_delta(a, b):
    d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return d


def make_straight_path(x_start, x_end, yaw, stamp, spacing=DEFAULT_SPACING_M, y=0.0, frame='odom'):
    """Straight waypoint line from x_start to x_end at fixed lateral position `y` and heading `yaw`.

    Identical waypoint generation to the original validation make_path() (x advanced by
    `spacing`, inclusive of the end while it is not overshot) so spacing == DEFAULT_SPACING_M
    reproduces the historical path exactly.
    """
    path = Path()
    path.header.frame_id = frame
    path.header.stamp = stamp
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    direction = 1.0 if x_end > x_start else -1.0
    n = 0
    x = x_start
    while (direction > 0 and x <= x_end) or (direction < 0 and x >= x_end):
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path.poses.append(ps)
        n += 1
        x = x_start + n * spacing * direction     # multiply, don't accumulate: no float drift at fine spacing
    return path


def densify_path(path, max_spacing):
    """Return a new Path with consecutive waypoints <= max_spacing apart; original waypoints kept exactly."""
    out = Path()
    out.header = path.header
    poses = path.poses
    if not poses:
        return out
    out.poses.append(poses[0])
    for a, b in zip(poses[:-1], poses[1:]):
        ax, ay = a.pose.position.x, a.pose.position.y
        bx, by = b.pose.position.x, b.pose.position.y
        d = math.hypot(bx - ax, by - ay)
        k = max(1, int(math.ceil(d / max_spacing - 1e-9)))
        ya, yb = _yaw_of(a.pose), _yaw_of(b.pose)
        dyaw = _shortest_delta(ya, yb)
        for i in range(1, k):
            f = i / k
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = ax + f * (bx - ax)
            ps.pose.position.y = ay + f * (by - ay)
            yaw = ya + f * dyaw
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            out.poses.append(ps)
        out.poses.append(b)
    return out


def path_stats(path):
    """(n_points, length_m, max_spacing_m, mean_spacing_m)"""
    p = path.poses
    if len(p) < 2:
        return len(p), 0.0, 0.0, 0.0
    d = [math.hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y)
         for a, b in zip(p[:-1], p[1:])]
    return len(p), sum(d), max(d), sum(d) / len(d)
