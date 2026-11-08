#!/usr/bin/env python3
"""picker_test.py (v8.2) - drive aisle 2 past the scene's standing picker and post, and measure.

    python3 ~/ros2_ws/src/tools/picker_test.py <label> [passes]     (default 8: 4 up, 4 down)

Up: from home (map 0, 0) on the keep-left lanes (lanes.py) into aisle 2's northbound lane, to
(2.75, 8.0): past the post at y 3.0, then the picker at y 6.0, both 0.35 m from the lane.
Down: home on the southbound lane (x 4.25), 1.85 m from both (a control: nothing should change).
Every 0.1 s of sim time it records the robot's pose and speed (/odom), the speed limit the guard
sends Nav2 (/speed_limit), and the robot's distance to each scene object (~/runs/scene.json).
Per pass and object: closest centre-to-centre distance, the robot's speed there, its top speed
within 1.0 m and 1.5 m, and the seconds it spent within 1.5 m.
Rows: ~/runs/picker/<label>.csv (every 0.1 s) and ~/runs/picker_trials.csv (per pass and object).
Ctrl+C stops cleanly and cancels the goal in progress. picker_report.py compares labels.
It must be the only thing sending goals.
"""
import csv
import json
import math
import os
import signal
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

UP = (2.75, 8.0, math.pi / 2)        # aisle 2, northbound lane
HOME = (0.0, 0.0, -math.pi / 2)      # spawn, facing the front wall
PASS_TIMEOUT = 180.0                 # sim s
MIN_GAP = 2.0                        # sim s between arriving and the next goal
TICK = 0.1                           # sim s between samples
SCENE = os.path.expanduser('~/runs/scene.json')
DEFAULT_SCENE = [dict(name='picker', kind='person', x=2.40, y=6.00),
                 dict(name='post', kind='object', x=2.40, y=3.00)]
OUT_DIR = os.path.expanduser('~/runs/picker')
TRIALS = os.path.expanduser('~/runs/picker_trials.csv')
TRIAL_FIELDS = ['label', 'pass', 'dir', 'result', 'duration_s', 'object', 'kind', 'closest_m',
                'v_at_closest', 'vmax_1m', 'vmax_1_5m', 's_within_1_5m', 'limit_min_1_5m']
STOP = {'requested': False}


def load_scene(path=SCENE):
    try:
        objs = json.load(open(path)).get('objects', [])
        return [o for o in objs if {'name', 'kind', 'x', 'y'} <= set(o)] or DEFAULT_SCENE
    except (OSError, ValueError, AttributeError):
        return DEFAULT_SCENE


def summarise(ticks, obj):
    """ticks: [(t, x, y, yaw, v, limit)] of one pass. Stats for one scene object."""
    if not ticks:
        return None
    d = [math.hypot(x - obj['x'], y - obj['y']) for _, x, y, _, _, _ in ticks]
    k = min(range(len(d)), key=lambda i: d[i])
    v1 = [tk[4] for tk, dd in zip(ticks, d) if dd < 1.0]
    v15 = [tk[4] for tk, dd in zip(ticks, d) if dd < 1.5]
    lim15 = [tk[5] for tk, dd in zip(ticks, d) if dd < 1.5 and tk[5] is not None and tk[5] > 0]
    return dict(closest_m=d[k], v_at_closest=ticks[k][4], vmax_1m=max(v1) if v1 else float('nan'),
                vmax_1_5m=max(v15) if v15 else float('nan'), s_within_1_5m=TICK * len(v15),
                limit_min_1_5m=min(lim15) if lim15 else float('nan'))


def fmt(v, nd=2):
    return '' if v is None or v != v else f'{v:.{nd}f}'


def _request_stop(signum, frame):
    STOP['requested'] = True


class Stop(Exception):
    pass


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print('usage: picker_test.py <label> [passes]')
        return 1
    label = argv[0]
    passes = int(argv[1]) if len(argv) > 1 else 8

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import NavigateThroughPoses
    from action_msgs.msg import GoalStatus
    from tf2_ros import Buffer, TransformListener
    try:
        from nav2_msgs.msg import SpeedLimit
    except ImportError:
        SpeedLimit = None
    import lanes

    result_name = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                   GoalStatus.STATUS_CANCELED: 'CANCELED'}

    def pose_stamped(x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return m

    class Runner(Node):
        def __init__(self):
            super().__init__('picker_test', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.nav = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
            self.buf = Buffer()
            self.tfl = TransformListener(self.buf, self)
            self.v, self.limit = 0.0, None
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            if SpeedLimit is not None:
                self.create_subscription(SpeedLimit, '/speed_limit', self.on_limit, 10)
            self.active = None

        def on_odom(self, msg):
            self.v = abs(msg.twist.twist.linear.x)

        def on_limit(self, msg):
            self.limit = None if msg.speed_limit <= 0.0 else float(msg.speed_limit)

        def spin(self, timeout=0.02):
            rclpy.spin_once(self, timeout_sec=timeout)
            if STOP['requested']:
                raise Stop()

        def now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def pose(self):
            try:
                tf = self.buf.lookup_transform('map', 'base_link', Time())
            except Exception:
                return None
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            return tf.transform.translation.x, tf.transform.translation.y, yaw

        def wait(self, fut, wall=None):
            t0 = time.monotonic()
            while not fut.done():
                self.spin()
                if wall is not None and time.monotonic() - t0 > wall:
                    return False
            return True

        def drive(self, target, ticks):
            """One pass along the lanes to target; samples into ticks. Returns the result."""
            start = self.pose()
            pts = lanes.route(start[:2], target[:2], target[2])
            goal = NavigateThroughPoses.Goal()
            goal.poses = [pose_stamped(*p) for p in pts]
            sent = self.nav.send_goal_async(goal)
            self.wait(sent)
            handle = sent.result()
            if handle is None or not handle.accepted:
                return 'REJECTED'
            self.active = handle
            res = handle.get_result_async()
            t0 = next_tick = self.now()
            while not res.done():
                self.spin()
                t = self.now()
                if t >= next_tick:
                    next_tick = t + TICK
                    p = self.pose()
                    if p is not None:
                        ticks.append((t, p[0], p[1], p[2], self.v, self.limit))
                if t - t0 > PASS_TIMEOUT:
                    c = handle.cancel_goal_async()
                    self.wait(c, wall=5.0)
                    self.wait(res, wall=10.0)
                    break
            self.active = None
            if not res.done() or res.result() is None:
                return 'NO_RESULT'
            return result_name.get(res.result().status, f'STATUS_{res.result().status}')

        def cancel(self):
            if self.active is None:
                return
            f = self.active.cancel_goal_async()
            t0 = time.monotonic()
            while not f.done() and time.monotonic() - t0 < 3.0:
                rclpy.spin_once(self, timeout_sec=0.05)

    scene = load_scene()
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    signal.signal(signal.SIGINT, _request_stop)
    node = Runner()
    os.makedirs(OUT_DIR, exist_ok=True)
    tick_path = os.path.join(OUT_DIR, f'{label}.csv')
    rows = []
    try:
        t0 = time.monotonic()
        while node.now() <= 0.0 and time.monotonic() - t0 < 10.0:
            node.spin(0.05)
        if not node.nav.wait_for_server(timeout_sec=10.0):
            print('ABORT: navigate_through_poses is not up. Is Nav2 (T4) running?')
            return 1
        t0 = time.monotonic()
        while node.pose() is None and time.monotonic() - t0 < 10.0:
            node.spin(0.05)
        if node.pose() is None:
            print('ABORT: no robot pose (TF map -> base_link).')
            return 1
        print(f'[{label}] {passes} passes in aisle 2 (up past the post and the picker, down the far lane). '
              f'Scene: ' + ', '.join(f'{o["name"]} ({o["kind"]}) at ({o["x"]:.2f}, {o["y"]:.2f})' for o in scene))
        with open(tick_path, 'w', newline='') as tf:
            tw = csv.writer(tf)
            tw.writerow(['label', 'pass', 'dir', 't', 'x', 'y', 'yaw', 'v', 'limit'] +
                        [f'd_{o["name"]}' for o in scene])
            for i in range(1, passes + 1):
                direction = 'up' if i % 2 == 1 else 'down'
                t_ready = node.now() + MIN_GAP
                while node.now() < t_ready:
                    node.spin(0.05)
                ticks = []
                s0 = node.now()
                result = node.drive(UP if direction == 'up' else HOME, ticks)
                dur = node.now() - s0
                for tk in ticks:
                    tw.writerow([label, i, direction, f'{tk[0]:.2f}', f'{tk[1]:.3f}', f'{tk[2]:.3f}', f'{tk[3]:.3f}',
                                 f'{tk[4]:.3f}', fmt(tk[5], 3)] +
                                [f'{math.hypot(tk[1] - o["x"], tk[2] - o["y"]):.3f}' for o in scene])
                tf.flush()
                parts = []
                for o in scene:
                    s = summarise(ticks, o)
                    if s is None:
                        continue
                    rows.append(dict(label=label, **{'pass': i}, dir=direction, result=result,
                                     duration_s=f'{dur:.1f}', object=o['name'], kind=o['kind'],
                                     **{k: fmt(v) for k, v in s.items()}))
                    parts.append(f'{o["name"]} {s["closest_m"]:.2f} m at {s["v_at_closest"]:.2f} m/s')
                print(f'  pass {i:2d} {direction:4s} {result:9s} {dur:5.1f} s   ' + '   '.join(parts), flush=True)
                if result not in ('SUCCEEDED',):
                    print('  (stopping: the pass did not succeed)')
                    break
    except Stop:
        print('\nStopped: cancelling the goal in progress.')
        node.cancel()
    finally:
        if rows:
            new = not os.path.exists(TRIALS)
            with open(TRIALS, 'a', newline='') as f:
                w = csv.DictWriter(f, fieldnames=TRIAL_FIELDS)
                if new:
                    w.writeheader()
                w.writerows(rows)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    print(f'Done. {len(rows)} rows -> {TRIALS}; every 0.1 s -> {tick_path}')
    print(f'Compare runs: python3 {os.path.join(TOOLS, "picker_report.py")} <label> [<label> ...]')
    return 0


if __name__ == '__main__':
    sys.exit(main())
