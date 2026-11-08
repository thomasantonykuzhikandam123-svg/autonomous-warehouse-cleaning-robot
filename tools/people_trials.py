#!/usr/bin/env python3
"""people_trials.py (v4) - automated worker trials, one command per experiment.

Drives spawn <-> aisle 1 and back, timing each send to a fixed point in the
worker's 6 s cycle so every configuration meets her at the same moments:
  passes 1-2 (up, down) at phase 0 s, passes 3-4 at 1 s, ... passes 11-12 at 5 s.
Her cycle, sim time mod 6: 0 s at the rack_1 end walking across, 2.2-3.0 s
turning at the rack_2 end, 5.2-6.0 s turning at the rack_1 end.

Goals (map frame): up = aisle 1, west half, y 8 (the northbound lane);
down = spawn. Two ways to get there:
  route=direct   one NavigateToPose goal, Nav2 picks the path (default)
  route=lanes    keep-left lanes from lanes.py, sent as NavigateThroughPoses:
                 line up before the aisle, up the west half, back down the east half

Before the first pass it reads the Nav2 settings that matter from the running
nodes (not the file) and saves them in every row. Add key=value checks and it
refuses to start unless Nav2 is really running with them:

    python3 ~/ros2_ws/src/tools/people_trials.py home      # drive to spawn, then exit
    python3 ~/ros2_ws/src/tools/people_trials.py <label> [passes] [route=...] [key=value ...]
    e.g.  people_trials.py lanes_wait 12 route=lanes tolerance=5.0

It must be the only thing sending goals. A goal from T8 or RViz cancels the
runner's (it shows up as ABORTED). If that happens the runner stops and flags
the pass.

Run worker_clearance.py (v2, it sees both goal types) with the same label
alongside: it measures, this only drives. Rows go to ~/runs/trials/<label>.csv;
join with people_trials.csv on 'goal'. Ctrl+C stops cleanly.

v3: route=lanes; the up goal moved to the northbound lane (-1.25, 8) for both
    routes so they can be compared; rows record route, goal and via-points,
    and which behaviour trees Nav2 is running.
v4: rows also record whether people_guard and the Collision Monitor were
    running (columns guard, cm); check them like the rest: guard=True cm=True.
"""
import csv
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lanes  # noqa: E402  (same folder)

PERIOD = 6.0                       # worker_test loop, sim seconds
PHASES = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
UP = (-1.25, 8.0, math.pi / 2)     # map x, y, yaw: aisle 1 northbound lane
DOWN = (0.0, 0.0, -math.pi / 2)    # spawn, facing the front wall
ROUTES = ('direct', 'lanes')
MIN_GAP = 1.0                      # sim s between arriving and the next send
PASS_TIMEOUT = 240.0               # sim s; a pass taking longer is cancelled
OUT_DIR = os.path.expanduser('~/runs/trials')
RESULT = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
          GoalStatus.STATUS_CANCELING)
STATUS_TOPICS = ('/navigate_to_pose/_action/status',
                 '/navigate_through_poses/_action/status')

# (node, parameter, column) read from the live nodes and saved with every row
STAMP = [
    ('/global_costmap/global_costmap', 'plugins', 'g_plugins'),
    ('/global_costmap/global_costmap', 'obstacle_layer.scan.inf_is_valid', 'g_inf'),
    ('/local_costmap/local_costmap', 'plugins', 'l_plugins'),
    ('/local_costmap/local_costmap', 'obstacle_layer.scan.inf_is_valid', 'l_inf'),
    ('/controller_server', 'failure_tolerance', 'tolerance'),
    ('/controller_server', 'FollowPath.use_rotate_to_heading', 'rotate_to_heading'),
    ('/bt_navigator', 'default_nav_to_pose_bt_xml', 'bt_to_pose'),
    ('/bt_navigator', 'default_nav_through_poses_bt_xml', 'bt_through'),
]
RUNNING = [('people_guard', 'guard'), ('collision_monitor', 'cm')]   # node, column
FIELDS = (['run', 'pass', 'goal', 'direction', 'route', 'goal_x', 'goal_y', 'via',
           'phase_target', 'phase_sent', 'start_x', 'start_y', 'result', 'duration_s',
           'foreign_goals'] + [c for _, _, c in STAMP] + [c for _, c in RUNNING])

STOP = {'requested': False}


class Stop(Exception):
    """Ctrl+C, raised between spins so it never lands inside rclpy's C code."""


def _request_stop(signum, frame):
    STOP['requested'] = True


def schedule(n):
    """[(direction, phase)] for n passes from spawn: up, down, up, down, ..."""
    return [('up' if i % 2 == 0 else 'down', PHASES[(i // 2) % len(PHASES)])
            for i in range(n)]


def next_send_time(now, phase, min_gap=MIN_GAP):
    """First sim time >= now + min_gap at which (t mod PERIOD) == phase."""
    earliest = now + min_gap
    t = math.floor(earliest / PERIOD) * PERIOD + phase
    while t < earliest - 1e-9:
        t += PERIOD
    return t


def param_value(v):
    """rcl_interfaces/ParameterValue -> python value; None when not set."""
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value,
            4: v.string_value, 9: '+'.join(v.string_array_value)}.get(v.type)


def parse_args(argv):
    """label, passes, {column: expected string}, route."""
    label = argv[1] if len(argv) > 1 else 'unlabelled'
    passes, expect, route = 12, {}, 'direct'
    for a in argv[2:]:
        if '=' in a:
            k, v = a.split('=', 1)
            if k == 'route':
                route = v
            else:
                expect[k] = v
        else:
            passes = int(a)
    return label, passes, expect, route


def mismatches(stamp, expect):
    return {k: (v, stamp.get(k)) for k, v in expect.items() if str(stamp.get(k)) != v}


def gid_of(uuid):
    return bytes(uuid).hex()[:8]


def pose_stamped(msg, x, y, yaw):
    """Fill a geometry_msgs/PoseStamped in the map frame."""
    msg.header.frame_id = 'map'
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class Trials(Node):
    def __init__(self):
        super().__init__('people_trials', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.active = None           # goal handle in progress
        self.own = set()             # goal ids this runner sent
        self.active_seen = set()     # every goal id seen accepted/executing
        for topic in STATUS_TOPICS:
            self.create_subscription(GoalStatusArray, topic, self.on_status, 10)

    def on_status(self, msg):
        for s in msg.status_list:
            if s.status in ACTIVE:
                self.active_seen.add(gid_of(s.goal_info.goal_id.uuid))

    def spin(self, timeout=0.05):
        rclpy.spin_once(self, timeout_sec=timeout)
        if STOP['requested']:
            raise Stop()

    def wait_future(self, fut, wall_timeout=None):
        start = time.monotonic()
        while not fut.done():
            self.spin()
            if wall_timeout is not None and time.monotonic() - start > wall_timeout:
                return False
        return True

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def spin_until(self, cond, wall_timeout=None):
        start = time.monotonic()
        while not cond():
            self.spin()
            if wall_timeout is not None and time.monotonic() - start > wall_timeout:
                return False
        return True

    def robot_xy(self):
        try:
            tf = self.buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def read_stamp(self):
        stamp, by_node = {}, {}
        for node, name, col in STAMP:
            by_node.setdefault(node, []).append((name, col))
        for node, items in by_node.items():
            cli = self.create_client(GetParameters, f'{node}/get_parameters')
            values = []
            if cli.wait_for_service(timeout_sec=5.0):
                req = GetParameters.Request()
                req.names = [name for name, _ in items]
                fut = cli.call_async(req)
                if self.wait_future(fut, wall_timeout=5.0) and fut.result() is not None:
                    values = list(fut.result().values)
            for i, (_, col) in enumerate(items):
                stamp[col] = param_value(values[i]) if i < len(values) else 'NO_REPLY'
            self.destroy_client(cli)
        for col in ('bt_to_pose', 'bt_through'):          # file name is enough
            if isinstance(stamp.get(col), str):
                stamp[col] = os.path.basename(stamp[col]) or 'default'
        names = set(self.get_node_names())
        for node_name, col in RUNNING:
            stamp[col] = node_name in names
        return stamp

    def plan(self, direction, route):
        """[(x, y, yaw)] to send: the goal alone, or lane via-points + goal."""
        x, y, yaw = UP if direction == 'up' else DOWN
        if route == 'direct':
            return [(x, y, yaw)]
        start = self.robot_xy()
        if start is None:
            raise ValueError('no robot pose (TF map -> base_link)')
        return lanes.route(start, (x, y), yaw)

    def run_pass(self, points):
        """Send one goal (NavigateToPose for one point, NavigateThroughPoses for
        several) and wait for its result. Returns (goal id, result)."""
        if len(points) == 1:
            goal = NavigateToPose.Goal()
            pose_stamped(goal.pose, *points[0])
            client = self.nav
        else:
            goal = NavigateThroughPoses.Goal()
            goal.poses = [pose_stamped(PoseStamped(), *p) for p in points]
            client = self.nav_through
        sent = client.send_goal_async(goal)
        self.wait_future(sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return '', 'REJECTED'
        gid = gid_of(handle.goal_id.uuid)
        self.own.add(gid)
        self.active = handle
        result = handle.get_result_async()
        t0 = self.sim_now()
        while not result.done():
            self.spin(0.1)
            if self.sim_now() - t0 > PASS_TIMEOUT:
                cancel = handle.cancel_goal_async()
                self.wait_future(cancel, wall_timeout=5.0)
                self.wait_future(result, wall_timeout=10.0)
                break
        self.active = None
        if not result.done() or result.result() is None:
            return gid, 'NO_RESULT'
        status = result.result().status
        return gid, RESULT.get(status, f'STATUS_{status}')

    def cancel_active(self):
        """After a stop: cancel the goal in progress so the robot halts."""
        if self.active is None:
            return
        fut = self.active.cancel_goal_async()
        start = time.monotonic()
        while not fut.done() and time.monotonic() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.05)


def init_ros():
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    signal.signal(signal.SIGINT, _request_stop)


def main():
    label, passes, expect, route = parse_args(sys.argv)
    if route not in ROUTES:
        print(f'ABORT: route={route}? Use route=direct or route=lanes.')
        return
    init_ros()
    node = Trials()
    try:
        if not node.spin_until(lambda: node.sim_now() > 0.0, wall_timeout=10.0):
            print('ABORT: no /clock. Is T1 running?')
            return
        if not node.nav.wait_for_server(timeout_sec=10.0):
            print('ABORT: navigate_to_pose is not up. Is T4 running?')
            return
        if route == 'lanes' and not node.nav_through.wait_for_server(timeout_sec=10.0):
            print('ABORT: navigate_through_poses is not up. Is T4 running?')
            return
        node.spin_until(lambda: node.robot_xy() is not None, wall_timeout=10.0)
        xy = node.robot_xy()

        if label == 'home':
            if xy is not None and math.hypot(*xy) <= 0.5:
                print('Already at spawn.')
                return
            gid, result = node.run_pass([DOWN])
            print(f'home: goal {gid} {result}')
            return

        if xy is None or math.hypot(*xy) > 0.5:
            print(f'ABORT: robot is at {xy}, not at spawn (0, 0). '
                  f'Run "people_trials.py home" first, then start the probe.')
            return
        node.spin_until(lambda: False, wall_timeout=2.0)   # discovery + status settle
        if 'worker_clearance' not in node.get_node_names():
            print('WARNING: the probe (worker_clearance) is not running. '
                  'Start it with the same label or nothing gets measured.')
        known = set(node.active_seen)                      # goals active before we start

        stamp = node.read_stamp()
        print(f'[{label}] {passes} passes, route={route}. Nav2 as it is running now:')
        for col, val in stamp.items():
            print(f'    {col:18s} {val}')
        bad = mismatches(stamp, expect)
        if bad:
            for k, (want, got) in bad.items():
                print(f'ABORT: expected {k}={want}, Nav2 is running with {k}={got}')
            print('Nothing was sent.')
            return
        print('  Hands off T8 and RViz goals until it says Done.')

        os.makedirs(OUT_DIR, exist_ok=True)
        path = os.path.join(OUT_DIR, f'{label}.csv')
        new_file = not os.path.exists(path)
        if not new_file:
            with open(path) as f:
                if f.readline().strip().split(',') != FIELDS:
                    print(f'ABORT: {path} has different columns (older version). '
                          f'Rename it or use a new label. Nothing was sent.')
                    return
        reported = set()
        for i, (direction, phase) in enumerate(schedule(passes), 1):
            send_at = next_send_time(node.sim_now(), phase)
            node.spin_until(lambda: node.sim_now() >= send_at)
            sx, sy = node.robot_xy() or (float('nan'), float('nan'))
            try:
                points = node.plan(direction, route)
            except ValueError as e:
                print(f'STOPPED: pass {i}: {e}. Nothing sent for this pass.')
                return
            via = ';'.join(f'{x:.2f},{y:.2f}' for x, y, _ in points[:-1])
            t_sent = node.sim_now()
            gid, result = node.run_pass(points)
            t_done = node.sim_now()
            node.spin_until(lambda: False, wall_timeout=0.3)   # late status messages
            foreign = node.active_seen - node.own - known - reported
            reported |= foreign
            gx, gy, _ = points[-1]
            row = dict(run=label, **{'pass': i}, goal=gid, direction=direction, route=route,
                       goal_x=gx, goal_y=gy, via=via, phase_target=phase,
                       phase_sent=round(t_sent % PERIOD, 2),
                       start_x=round(sx, 2), start_y=round(sy, 2), result=result,
                       duration_s=round(t_done - t_sent, 1),
                       foreign_goals=len(foreign), **stamp)
            print(f'  pass {i:2d} {direction:4s} phase {phase:.0f} s  goal {gid}  '
                  f'{result}  {row["duration_s"]} s' + (f'  via {via}' if via else ''))
            with open(path, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=FIELDS)
                if new_file:
                    writer.writeheader()
                    new_file = False
                writer.writerow(row)
            if foreign:
                print(f'STOPPED: goal(s) {", ".join(sorted(foreign))} came from outside the '
                      f'runner (T8 or RViz) during pass {i}. That pass is flagged in the CSV. '
                      f'Rerun with nothing else sending goals.')
                return
        print(f'Done. Rows in {path}')
    except Stop:
        print('\nStopping: cancelling the goal in progress.')
        node.cancel_active()
        print('Stopped.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
