#!/usr/bin/env python3
"""One fresh-sim session of the construction-site maneuver with full raw-signal recording, either
  mode=normal : the unmodified maneuver (legs 1-3) - source of false-positive data, or
  mode=freeze : the same maneuver with the POSITIVE CONTROL - a low cage is spawned around the
                chassis at up to three points while it is being commanded to move:
                  straight_raw  leg 2, raw straight drive, 3.5 m onto the platform
                  arc           leg 3, once the turn has reached >= 30 deg (mid-turn)
                  taper         leg 3, once the turn is >= 170 deg (near-straight travel)
                The cage stays OBS_S seconds, then is removed and the maneuver continues.
Ground truth (Gazebo pose) and every candidate detector input are recorded to one .npz.

Usage: freeze_session.py <normal|freeze> <unloaded|loaded> <session_id> <out_prefix>
                         [clearance_m=front,back,side (default 0.06,0.08,0.04)] [lead_s=0.30] [obs_s=20] [events=straight_raw,arc,taper]
Refuses to run (exit 2) unless harness_guard.verify() passes.
"""
import sys, os, time, math, json
from pathlib import Path
HERE = Path(__file__).resolve().parent
PH8 = HERE.parent
sys.path.insert(0, str(PH8 / 'harness'))
sys.path.insert(0, str(PH8 / 'construction_site_descent' / 'crosstrack_diagnosis_flatmode' / 'drift_characterization'))
sys.path.insert(0, str(HERE))
MODE, PAYLOAD, SID, OUT = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
_c = [float(v) for v in sys.argv[5].split(',')] if len(sys.argv) > 5 else [0.06, 0.08, 0.04]
CLEARANCE = tuple(_c) if len(_c) == 3 else _c[0]      # metres: front,back,side  (or one value for all)
LEAD_S = float(sys.argv[6]) if len(sys.argv) > 6 else 0.30
OBS_S = float(sys.argv[7]) if len(sys.argv) > 7 else 20.0
EVENTS = (sys.argv[8] if len(sys.argv) > 8 else 'straight_raw,arc,taper').split(',')

import harness_guard as hg
import stall_diagnosis_trials as sd      # reuses Test, path builder, constants (reads argv[1..3] harmlessly)
import stall_control_lib as lib
import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath
from action_msgs.msg import GoalStatus

STRAIGHT_RAW_ODOM_X = 43.5        # odom frame; world x = odom x + ~1.03, i.e. ~3.5 m past the ramp top


class Sess(sd.Test, lib.Recorder):
    def __init__(self, truth):
        sd.Test.__init__(self)
        self.setup_recorder()
        self.truth = truth
        self.events, self.fired = [], set()

    # ---- positive control -------------------------------------------------------------------
    def fire(self, kind, raw_cmd=None):
        t, x, y, z, yaw = self.truth.latest()
        speed, yawrate = self.truth.velocity(0.3)
        px, py, pyaw = x + speed * math.cos(yaw) * LEAD_S, y + speed * math.sin(yaw) * LEAD_S, yaw + yawrate * LEAD_S
        t_call = time.time()
        ok, rtt, out = lib.spawn_cage('freeze_cage', px, py, z - lib.FLOOR_BELOW_MODEL_Z, pyaw, CLEARANCE)
        ev = dict(kind=kind, t_call=t_call, t_done=time.time(), rtt=rtt, ok=ok, out=out[:160],
                  pose_pred=[px, py, pyaw], pose_gt_at_call=[x, y, yaw], speed=speed, yawrate=yawrate,
                  clearance=CLEARANCE, lead=LEAD_S, obs_s=OBS_S, state_at_call=self.state)
        self.events.append(ev)
        self.get_logger().warn(f'POSITIVE CONTROL {kind}: cage ok={ok} rtt={rtt:.2f}s speed={speed:.2f} yawrate={yawrate:.2f}')
        if not ok:
            return
        end = time.time() + OBS_S
        while time.time() < end:
            if raw_cmd is not None:
                self.cmd_pub.publish(raw_cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        ev['t_obs_end'] = time.time()
        ev['removed'] = lib.remove_cage('freeze_cage')[0]
        self.get_logger().warn(f'POSITIVE CONTROL {kind}: cage removed={ev["removed"]}')

    def moving(self, min_speed=0.15):
        return self.truth.velocity(0.5)[0] >= min_speed and self.state != 'HALT'

    # ---- leg 2 with the straight-travel event ---------------------------------------------------
    def drive_to_ev(self, target_x, vx, timeout=400.0):
        cmd = Twist(); cmd.linear.x = vx
        deadline = time.time() + timeout
        try:
            while time.time() < deadline:
                self.cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
                if (MODE == 'freeze' and 'straight_raw' in EVENTS and 'straight_raw' not in self.fired
                        and self.odom_x >= STRAIGHT_RAW_ODOM_X and self.moving(0.2)):
                    self.fired.add('straight_raw')
                    self.fire('straight_raw', raw_cmd=cmd)
                if self.odom_x >= target_x:
                    break
        finally:
            self.hard_stop()

    # ---- leg 3 with the mid-turn / taper events ---------------------------------------------------
    def _send(self, path):
        goal = FollowPath.Goal(); goal.path = path; goal.controller_id = 'FollowPath'
        fut = self.follow_client.send_goal_async(goal)
        end = time.time() + 10
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        return (gh, gh.get_result_async()) if gh is not None and gh.accepted else (None, None)

    def follow_arc_ev(self, timeout=480.0):
        self.follow_client.wait_for_server(timeout_sec=10.0)
        path = sd.build_turn_and_ramp_path(self.odom_x, self.odom_y, sd.ARC_RADIUS, sd.RAMP_APPROACH_X,
                                           sd.RAMP_EXIT_X, self.get_clock().now().to_msg())
        plen = sum(math.hypot(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
                              path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y)
                   for i in range(1, len(path.poses)))
        gh, rfut = self._send(path)
        if gh is None:
            return False, {'error': 'goal rejected'}
        v0 = self.verified_progress_m
        yaw0 = self.truth.latest()[4]
        deadline = time.time() + timeout
        resends, succeeded, aborted = 0, False, False
        try:
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.margin < sd.ABORT_MARGIN or abs(self.pitch) > sd.ABORT_PITCH_DEG:
                    aborted = True
                    break
                dyaw = abs(lib.wrap(self.truth.latest()[4] - yaw0))
                if MODE == 'freeze':
                    if 'arc' in EVENTS and 'arc' not in self.fired and dyaw >= 0.52 and self.moving():
                        self.fired.add('arc'); self.fire('arc')
                    elif 'taper' in EVENTS and 'taper' not in self.fired and dyaw >= 2.97 and self.moving():
                        self.fired.add('taper'); self.fire('taper')
                if rfut is not None and rfut.done():
                    st = rfut.result().status
                    if st == GoalStatus.STATUS_SUCCEEDED:
                        succeeded = True
                        break
                    # Nav2's progress checker aborts a goal on a robot that is (deliberately) frozen: re-send
                    # the remaining path from the nearest pose so later events can still run.
                    if resends < 6 and MODE == 'freeze':
                        resends += 1
                        best = min(range(len(path.poses)), key=lambda i: math.hypot(
                            path.poses[i].pose.position.x - self.odom_x, path.poses[i].pose.position.y - self.odom_y))
                        rest = NavPath(); rest.header = path.header; rest.poses = path.poses[best:]
                        gh, rfut = self._send(rest)
                        if gh is None:
                            break
                    else:
                        break
        finally:
            self.hard_stop()
        frac = (self.verified_progress_m - v0) / plen if plen > 0 else 0.0
        return succeeded, dict(aborted=aborted, resends=resends, verified_fraction=round(frac, 3))


def main():
    guard = hg.verify()
    monitor = hg.HealthMonitor(period=20.0).start()
    truth = lib.TruthStream().start()
    rclpy.init()
    node = Sess(truth)
    result = {}
    try:
        node.wait_for_odom()
        node.get_logger().info(f'SESSION {MODE}/{PAYLOAD}/{SID} start events={EVENTS if MODE == "freeze" else "-"}')
        t_start = time.time()
        node.drive_to(sd.LAYDOWN_END_X, sd.DRIVE_VX)
        t1 = time.time()
        node.drive_to_ev(sd.ELEV_MID_X, sd.DRIVE_VX)
        t2 = time.time()
        ok, info = node.follow_arc_ev()
        t3 = time.time()
        result = dict(leg3_ok=ok, **info, t_start=t_start, t_leg1_end=t1, t_leg2_end=t2, t_leg3_end=t3)
        node.get_logger().info(f'SESSION done: {result}')
    finally:
        try:
            lib.remove_cage('freeze_cage')
        except Exception:
            pass
        node.hard_stop()
        health = monitor.stop()
        node.save(OUT + '.npz', truth, node.events,
                  extra=dict(mode=MODE, payload=PAYLOAD, session=SID, guard=guard, health=health, result=result,
                             clearance=CLEARANCE, lead=LEAD_S, obs_s=OBS_S, events_cfg=EVENTS))
        truth.stop()
        print(f'saved {OUT}.npz  events={len(node.events)}  health={health}')
        if health['starved']:
            hg.banner(f'session {MODE}/{SID} ran while the host was STARVED (min RTF {health["min_rtf"]}, '
                      f'max load1 {health["max_load1"]}); results INVALID.')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
