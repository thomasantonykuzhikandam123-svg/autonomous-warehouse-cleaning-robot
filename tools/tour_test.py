#!/usr/bin/env python3
"""tour_test.py (v10.3) - laps through both aisles among the traffic, measured.

    python3 ~/ros2_ws/src/tools/tour_test.py <label> [laps] [noestop]      (default 3 laps)
    python3 ~/ros2_ws/src/tools/tour_test.py <label> watch [max_s]          (v10.3: a cleaning shift)

A lap, on the keep-left lanes (lanes.py), as three Nav2 goals:
  1. home (0, 0) -> up aisle 1's northbound lane to (-1.25, 12.5)
  2. -> east along the back corridor -> into aisle 2's southbound lane to (4.25, 12.0)
  3. -> down aisle 2 -> west along the front corridor -> home
Traffic (traffic.py, stack.sh ... traffic): picker_a in aisle 1, picker_b in aisle 2, the stepper
crossing the back corridor, the visitor in the front corridor, the world's worker crossing aisle 1,
the tugger in the back corridor, the forklift at aisle 2's mouth. Every 0.1 s of sim time it
records the robot (TF, /odom), what the guard does (/people_guard/hold, /speed_limit) and
everyone's true position (people: their scripts; vehicles: their odometry), and measures
body-to-body gaps: the robot's 0.6 x 0.4 m body to a person's 0.25 m radius, or to a vehicle's
body and forks. 0 = touching.
Emergency stop: in lap 2, once the robot is driving up aisle 1 at speed, it presses
/emergency_stop (std_msgs/Bool) for 4 s and measures how fast the robot stopped and that it
carried on afterwards (noestop: skip it). v10.3: pressed from y 6.0 (it was 4.5, just past the
worker's walkway at y 4.0, so the robot stood 4 s in her way and she walked into its back:
every worker 'contact' since tour_v9 was this test).
Writes ~/runs/tour/<label>.csv (every tick), ~/runs/tour_trials.csv (per lap and agent),
~/runs/tour/<label>_laps.csv, ~/runs/tour/<label>_estop.json. tour_report.py compares runs.
Ctrl+C cancels the goal in progress and releases the emergency stop. It must be the only thing
sending goals.

v10.3, watch: sends no goals and presses no stop. It measures whatever drives the robot - the
cleaning (coverage_nav2) - exactly as a tour is measured, as one 'lap', from the first move until
coverage_nav2's /mission_state says done (for 5 s), max_s s of sim time (default 1800) or Ctrl+C.
The tick file gets the waypoint ('goal'), the zone and coverage_nav2's coverage figure, so
tour_report.py and tour_why.py read a shift like a tour.
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

VERSION = 'v10.3'
GOALS = [('aisle 1 up', (-1.25, 12.5, math.pi / 2)),
         ('back corridor, aisle 2', (4.25, 12.0, -math.pi / 2)),
         ('aisle 2 down, home', (0.0, 0.0, -math.pi / 2))]
GOAL_TIMEOUT = 240.0          # sim s per goal
GAP = 2.0                     # sim s between goals
TICK = 0.1
NEAR = 1.0                    # m body-to-body: 'close'
ESTOP_LAP, ESTOP_S = 2, 4.0
ESTOP_Y0 = 6.0                # v10.3: pressed only past y 6.0 (2 m beyond the worker's walkway)
OUT_DIR = os.path.expanduser('~/runs/tour')
TRIALS = os.path.expanduser('~/runs/tour_trials.csv')
TRIAL_FIELDS = ['label', 'lap', 'agent', 'kind', 'closest_m', 'v_at_closest', 'where', 'vmax_near',
                's_near', 'contact_moving_s', 'contact_still_s', 'drove_into_s', 'fork_closest_m']
# the world's own worker (warehouse.sdf actor), world frame: (time s, x, y); 6 s loop
WORKER = [(0.0, 2.4, 8.0), (2.2, 4.6, 8.0), (3.0, 4.6, 8.0), (5.2, 2.4, 8.0), (6.0, 2.4, 8.0)]
STOP = {'requested': False}


def worker_xy(t):
    t %= WORKER[-1][0]
    for (t0, x0, y0), (t1, x1, y1) in zip(WORKER, WORKER[1:]):
        if t0 <= t <= t1:
            u = (t - t0) / (t1 - t0)
            return x0 + (x1 - x0) * u - 4.0, y0 + (y1 - y0) * u - 4.0
    return WORKER[0][1] - 4.0, WORKER[0][2] - 4.0


def region(x, y):
    if -2.0 < x < 1.0 and 1.5 < y < 13.5:
        return 'aisle 1'
    if 2.0 < x < 5.0 and 1.5 < y < 13.5:
        return 'aisle 2'
    if 6.0 < x < 9.0 and 1.5 < y < 13.5:
        return 'aisle 3'
    if y >= 13.5:
        return 'back corridor'
    if y <= 1.5:
        return 'front corridor'
    return 'by a rack'


def _request_stop(signum, frame):
    STOP['requested'] = True


class Stop(Exception):
    pass


def save_trials(label, rows):
    """One run per label: replace this label's old rows in TRIALS instead of appending to them (a
    second run under one label used to double its laps in the report). Written to a temporary file
    and then renamed, so a crash can't truncate everyone's rows; if the old file can't be read
    (damaged), this run's rows are appended to it instead."""
    try:
        keep = []
        if os.path.exists(TRIALS):
            with open(TRIALS, newline='', errors='replace') as f:
                keep = [r for r in csv.DictReader(f) if r.get('label') != label]
        tmp = TRIALS + '.tmp'
        with open(tmp, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=TRIAL_FIELDS, extrasaction='ignore')
            w.writeheader()
            w.writerows(keep)
            w.writerows(rows)
        os.replace(tmp, TRIALS)
    except Exception as e:
        print(f'  note: could not rewrite {TRIALS} ({e}); added this run\'s rows to its end instead',
              flush=True)
        new = not os.path.exists(TRIALS)
        with open(TRIALS, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=TRIAL_FIELDS, extrasaction='ignore')
            if new:
                w.writeheader()
            w.writerows(rows)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__)
        return 1
    label = argv[0]
    watch = len(argv) > 1 and argv[1] == 'watch'
    watch_max = float(argv[2]) if watch and len(argv) > 2 else 1800.0
    laps = 1 if watch else (int(argv[1]) if len(argv) > 1 and argv[1].isdigit() else 3)
    do_estop = 'noestop' not in argv and not watch

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
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener
    try:
        from nav2_msgs.msg import SpeedLimit
    except ImportError:
        SpeedLimit = None
    import lanes
    import traffic as T

    result_name = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                   GoalStatus.STATUS_CANCELED: 'CANCELED'}
    scripts = {n: T.person_waypoints(s) for n, s in T.PEOPLE.items()}
    try:
        spawned = set(json.load(open(os.path.join(T.OUT_DIR, 'spawned.json'))).get('spawned', []))
    except (OSError, ValueError):
        spawned = set(T.PEOPLE) | set(T.VEHICLES)
    people = ['worker'] + [n for n in T.PEOPLE if n in spawned]
    vehicles = [n for n in T.VEHICLES if n in spawned]

    def pose_stamped(x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        return m

    class Runner(Node):
        def __init__(self):
            super().__init__('tour_test', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.nav = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
            self.buf = Buffer()
            self.tfl = TransformListener(self.buf, self)
            self.v, self.limit, self.hold, self.hold_t = 0.0, None, False, None
            self.veh = {}
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, '/people_guard/hold', self.on_hold, 10)
            if SpeedLimit is not None:
                self.create_subscription(SpeedLimit, '/speed_limit', self.on_limit, 10)
            for n in vehicles:
                self.create_subscription(Odometry, f'/model/{n}/odometry', lambda m, n=n: self.on_veh(n, m), 10)
            self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
            self.active = None
            self.estop_on = False
            self.mission, self.mission_t = {}, None
            self.create_subscription(String, '/mission_state', self.on_mission, 10)

        def on_mission(self, msg):
            try:
                m = json.loads(msg.data)
            except ValueError:
                return
            if isinstance(m, dict):
                self.mission, self.mission_t = m, self.now()

        def on_odom(self, msg):
            self.v = abs(msg.twist.twist.linear.x)

        def on_hold(self, msg):
            self.hold = msg.width * msg.height > 0
            self.hold_t = self.now()

        def on_limit(self, msg):
            self.limit = None if msg.speed_limit <= 0.0 else float(msg.speed_limit)

        def on_veh(self, n, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            p = msg.pose.pose.position
            self.veh[n] = (p.x - 4.0, p.y - 4.0, yaw, msg.twist.twist.linear.x)

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

        def estop(self, on):
            self.estop_on = on
            self.estop_pub.publish(Bool(data=on))

        def cancel(self):
            if self.active is not None:
                f = self.active.cancel_goal_async()
                t0 = time.monotonic()
                while not f.done() and time.monotonic() - t0 < 3.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
            for _ in range(3):
                self.estop(False)
                rclpy.spin_once(self, timeout_sec=0.05)

    def gaps_at(t, rx, ry, ryaw, vehs):
        """{agent: (gap, kind, fork_gap, ax, ay)} body to body."""
        rob = T.corners(rx, ry, ryaw, T.ROBOT_L, T.ROBOT_W)
        out = {}
        for n in people:
            if n == 'worker':
                px, py = worker_xy(t)
            else:
                px, py, _, _ = T.person_at(scripts[n], t)
            out[n] = (T.circle_gap(rob, px, py, T.PERSON_R), 'person', None, px, py)
        for n in vehicles:
            if n not in vehs:
                continue
            x, y, a, _ = vehs[n]
            parts = T.outline(n, x, y, a)
            g = [T.poly_gap(rob, p) for p in parts]
            out[n] = (min(g), 'vehicle', g[1] if len(g) > 1 else None, x, y)
        return out

    def new_stats():
        return {n: dict(closest=9.0, v_at=0.0, where='', vmax_near=0.0, s_near=0.0, c_move=0.0,
                        c_still=0.0, into=0.0, fork=9.0) for n in names_all}

    def measure(stats, t, p, v, gaps):
        reg = region(p[0], p[1])
        for n, (g, kind, fg, ax, ay) in gaps.items():
            s = stats[n]
            if g < s['closest']:
                s.update(closest=g, v_at=v, where=reg)
            if g < NEAR:
                s['vmax_near'] = max(s['vmax_near'], v)
                s['s_near'] += TICK
            if g <= 0.0:
                if v > 0.05:
                    s['c_move'] += TICK
                    if math.cos(p[2]) * (ax - p[0]) + math.sin(p[2]) * (ay - p[1]) > 0:
                        s['into'] += TICK
                else:
                    s['c_still'] += TICK
            if fg is not None:
                s['fork'] = min(s['fork'], fg)
        return reg

    def run_watch(node, label, names, tick_path, max_s):
        """v10.3: measure a cleaning shift (one 'lap'). -> (trial rows, lap row) or None"""
        stats = new_stats()
        holds, hold_s, prev_hold = 0, 0.0, False
        t_start, done_t, last_note = None, None, None
        with open(tick_path, 'w', newline='') as tf:
            tw = csv.writer(tf)
            tw.writerow(['label', 'lap', 'goal', 't', 'x', 'y', 'yaw', 'v', 'limit', 'hold', 'estop', 'region'] +
                        [f'gap_{n}' for n in names] + ['zone', 'coverage_pct', 'mission_done'])
            next_tick = node.now()
            try:
                while True:
                    node.spin()
                    t = node.now()
                    if t < next_tick:
                        continue
                    next_tick = t + TICK
                    p = node.pose()
                    if p is None:
                        continue
                    m = node.mission
                    if t_start is None:
                        if node.v < 0.05:
                            continue                     # the shift starts when the robot first moves
                        t_start = t
                        print(f'  [{t:7.1f}] the robot moved: measuring', flush=True)
                    gaps = gaps_at(t, p[0], p[1], p[2], node.veh)
                    hold = node.hold and node.hold_t is not None and t - node.hold_t < 0.5
                    if hold and not prev_hold:
                        holds += 1
                    hold_s += TICK if hold else 0.0
                    prev_hold = hold
                    reg = measure(stats, t, p, node.v, gaps)
                    tw.writerow([label, 1, m.get('goal', ''), f'{t:.2f}', f'{p[0]:.3f}', f'{p[1]:.3f}',
                                 f'{p[2]:.3f}', f'{node.v:.3f}', '' if node.limit is None else f'{node.limit:.2f}',
                                 int(hold), 0, reg] + [f'{gaps[n][0]:.3f}' if n in gaps else '' for n in names]
                                + [m.get('zone', ''), m.get('coverage_pct', ''), int(bool(m.get('done')))])
                    if last_note is None or t - last_note >= 60.0:
                        last_note = t
                        print(f'  [{t:7.1f}] waypoint {m.get("goal", "?")}/{m.get("total", "?")} {m.get("zone", "")}, '
                              f'coverage {m.get("coverage_pct", "?")} %, guard waited {holds}x ({hold_s:.0f} s)',
                              flush=True)
                        tf.flush()
                    if m.get('done'):
                        done_t = t if done_t is None else done_t
                        if t - done_t >= 5.0:
                            print(f'  [{t:7.1f}] coverage_nav2 says done', flush=True)
                            break
                    else:
                        done_t = None
                    if t - t_start >= max_s:
                        print(f'  [{t:7.1f}] {max_s:.0f} s of sim time measured', flush=True)
                        break
            except Stop:
                print('\n  Ctrl+C: saving what was measured', flush=True)
            if t_start is None:
                print('  the robot never moved: nothing measured', flush=True)
                return None
            dur = node.now() - t_start
            rows = [dict(label=label, lap=1, agent=n, kind='vehicle' if n in vehicles else 'person',
                         closest_m=f'{s["closest"]:.3f}', v_at_closest=f'{s["v_at"]:.2f}', where=s['where'],
                         vmax_near=f'{s["vmax_near"]:.2f}', s_near=f'{s["s_near"]:.1f}',
                         contact_moving_s=f'{s["c_move"]:.1f}', contact_still_s=f'{s["c_still"]:.1f}',
                         drove_into_s=f'{s["into"]:.1f}',
                         fork_closest_m='' if s['fork'] >= 9.0 else f'{s["fork"]:.3f}')
                    for n, s in stats.items()]
            m = node.mission
            lap = dict(label=label, lap=1, duration_s=f'{dur:.1f}',
                       results=f'SHIFT {m.get("coverage_pct", "?")} % ' + ('done' if m.get('done') else 'not done'),
                       guard_waits=holds, guard_wait_s=f'{hold_s:.1f}')
            contacts = [n for n in names if stats[n]['c_move'] + stats[n]['c_still'] > 0]
            print(f'  shift: {dur:5.1f} s, coverage {m.get("coverage_pct", "?")} %, guard waited {holds}x '
                  f'({hold_s:.0f} s), contacts: {", ".join(contacts) if contacts else "none"}', flush=True)
            print('     closest: ' + ', '.join(
                f'{n} {stats[n]["closest"]:.2f} m' + (f' (forks {stats[n]["fork"]:.2f})' if stats[n]['fork'] < 9.0 else '')
                for n in names if stats[n]['closest'] < 9.0), flush=True)
            return rows, lap

    names_all = people + vehicles
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    signal.signal(signal.SIGINT, _request_stop)
    node = Runner()
    os.makedirs(OUT_DIR, exist_ok=True)
    tick_path = os.path.join(OUT_DIR, f'{label}.csv')
    laps_path = os.path.join(OUT_DIR, f'{label}_laps.csv')
    estop_path = os.path.join(OUT_DIR, f'{label}_estop.json')
    rows, lap_rows = [], []
    estop = dict(done=False)
    try:
        t0 = time.monotonic()
        while node.now() <= 0.0 and time.monotonic() - t0 < 10.0:
            node.spin(0.05)
        if not watch and not node.nav.wait_for_server(timeout_sec=10.0):
            print('ABORT: navigate_through_poses is not up. Is Nav2 (T4) running?')
            return 1
        t0 = time.monotonic()
        while (node.pose() is None or len(node.veh) < len(vehicles)) and time.monotonic() - t0 < 10.0:
            node.spin(0.05)
        if node.pose() is None:
            print('ABORT: no robot pose (TF map -> base_link).')
            return 1
        missing = [n for n in vehicles if n not in node.veh]
        names = people + vehicles
        if watch:
            print(f'[{label}] watching the cleaning (sending no goals) until coverage_nav2 says done, '
                  f'{watch_max:.0f} s of sim time or Ctrl+C. People: {", ".join(people)}. Vehicles: '
                  f'{", ".join(vehicles) or "none"}'
                  + (f' (no odometry from {", ".join(missing)} - is the traffic tab running?)' if missing else ''),
                  flush=True)
            shift = run_watch(node, label, names, tick_path, watch_max)
            if shift is not None:
                rows.extend(shift[0])
                lap_rows.append(shift[1])
            raise Stop()
        print(f'[{label}] {laps} laps: up aisle 1, along the back corridor, down aisle 2, home. People: '
              f'{", ".join(people)}. Vehicles: {", ".join(vehicles) or "none"}'
              + (f' (no odometry from {", ".join(missing)} - is the traffic tab running?)' if missing else '')
              + ('' if do_estop else '. No emergency-stop test.'), flush=True)
        with open(tick_path, 'w', newline='') as tf:
            tw = csv.writer(tf)
            tw.writerow(['label', 'lap', 'goal', 't', 'x', 'y', 'yaw', 'v', 'limit', 'hold', 'estop', 'region'] +
                        [f'gap_{n}' for n in names])
            for lap in range(1, laps + 1):
                lap_t0 = node.now()
                stats = {n: dict(closest=9.0, v_at=0.0, where='', vmax_near=0.0, s_near=0.0, c_move=0.0,
                                 c_still=0.0, into=0.0, fork=9.0) for n in names}
                results, holds, hold_s, prev_hold = [], 0, 0.0, False
                for gi, (gname, target) in enumerate(GOALS, 1):
                    t_ready = node.now() + GAP
                    while node.now() < t_ready:
                        node.spin(0.05)
                    start = node.pose()
                    pts = lanes.route(start[:2], target[:2], target[2])
                    goal = NavigateThroughPoses.Goal()
                    goal.poses = [pose_stamped(*p) for p in pts]
                    sent = node.nav.send_goal_async(goal)
                    node.wait(sent)
                    handle = sent.result()
                    if handle is None or not handle.accepted:
                        results.append('REJECTED')
                        break
                    node.active = handle
                    res = handle.get_result_async()
                    g0 = next_tick = node.now()
                    while not res.done():
                        node.spin()
                        t = node.now()
                        if t < next_tick:
                            continue
                        next_tick = t + TICK
                        p = node.pose()
                        if p is None:
                            continue
                        # the emergency stop test
                        if do_estop and lap == ESTOP_LAP and not estop['done']:
                            if 'press_t' not in estop:
                                if gi == 1 and ((ESTOP_Y0 < p[1] < 10.5 and node.v > 0.45) or
                                                (p[1] >= 10.5 and node.v > 0.25)):
                                    node.estop(True)
                                    estop.update(press_t=t, v0=node.v, x0=p[0], y0=p[1])
                                    print(f'  [{t:7.1f}] EMERGENCY STOP pressed at ({p[0]:.2f}, {p[1]:.2f}), '
                                          f'robot at {node.v:.2f} m/s', flush=True)
                            elif node.estop_on:
                                node.estop(True)                    # keep it pressed (10 Hz)
                                if 'stop_t' not in estop and node.v < 0.02:
                                    estop.update(stop_t=t, stop_d=math.hypot(p[0] - estop['x0'], p[1] - estop['y0']))
                                if t - estop['press_t'] >= ESTOP_S:
                                    node.estop(False)
                                    estop.update(release_t=t, moved_while_pressed=math.hypot(
                                        p[0] - estop['x0'], p[1] - estop['y0']))
                                    print(f'  [{t:7.1f}] emergency stop released', flush=True)
                            elif node.v > 0.15:
                                estop.update(resume_t=t, done=True)
                        gaps = gaps_at(t, p[0], p[1], p[2], node.veh)
                        hold = node.hold and node.hold_t is not None and t - node.hold_t < 0.5
                        if hold and not prev_hold:
                            holds += 1
                        hold_s += TICK if hold else 0.0
                        prev_hold = hold
                        reg = region(p[0], p[1])
                        tw.writerow([label, lap, gi, f'{t:.2f}', f'{p[0]:.3f}', f'{p[1]:.3f}', f'{p[2]:.3f}',
                                     f'{node.v:.3f}', '' if node.limit is None else f'{node.limit:.2f}',
                                     int(hold), int(node.estop_on), reg] +
                                    [f'{gaps[n][0]:.3f}' if n in gaps else '' for n in names])
                        for n, (g, kind, fg, ax, ay) in gaps.items():
                            s = stats[n]
                            if g < s['closest']:
                                s.update(closest=g, v_at=node.v, where=reg)
                            if g < NEAR:
                                s['vmax_near'] = max(s['vmax_near'], node.v)
                                s['s_near'] += TICK
                            if g <= 0.0:
                                if node.v > 0.05:
                                    s['c_move'] += TICK
                                    if math.cos(p[2]) * (ax - p[0]) + math.sin(p[2]) * (ay - p[1]) > 0:
                                        s['into'] += TICK
                                else:
                                    s['c_still'] += TICK
                            if fg is not None:
                                s['fork'] = min(s['fork'], fg)
                        if t - g0 > GOAL_TIMEOUT:
                            c = handle.cancel_goal_async()
                            node.wait(c, wall=5.0)
                            node.wait(res, wall=10.0)
                            break
                    node.active = None
                    r = 'NO_RESULT' if not res.done() or res.result() is None else \
                        result_name.get(res.result().status, f'STATUS_{res.result().status}')
                    results.append(r)
                    tf.flush()
                    if r != 'SUCCEEDED':
                        break
                dur = node.now() - lap_t0
                for n in names:
                    s = stats[n]
                    kind = 'vehicle' if n in vehicles else 'person'
                    rows.append(dict(label=label, lap=lap, agent=n, kind=kind, closest_m=f'{s["closest"]:.3f}',
                                     v_at_closest=f'{s["v_at"]:.2f}', where=s['where'],
                                     vmax_near=f'{s["vmax_near"]:.2f}', s_near=f'{s["s_near"]:.1f}',
                                     contact_moving_s=f'{s["c_move"]:.1f}', contact_still_s=f'{s["c_still"]:.1f}',
                                     drove_into_s=f'{s["into"]:.1f}',
                                     fork_closest_m='' if s['fork'] >= 9.0 else f'{s["fork"]:.3f}'))
                lap_rows.append(dict(label=label, lap=lap, duration_s=f'{dur:.1f}', results='/'.join(results),
                                     guard_waits=holds, guard_wait_s=f'{hold_s:.1f}'))
                contacts = [n for n in names if stats[n]['c_move'] + stats[n]['c_still'] > 0]
                print(f'  lap {lap}: {dur:5.1f} s  goals {"/".join(results)}  guard waited {holds}x ({hold_s:.0f} s)  '
                      f'contacts: {", ".join(contacts) if contacts else "none"}', flush=True)
                print('     closest: ' + ', '.join(
                    f'{n} {stats[n]["closest"]:.2f} m' + (f' (forks {stats[n]["fork"]:.2f})' if stats[n]['fork'] < 9.0 else '')
                    for n in names if stats[n]['closest'] < 9.0), flush=True)
                if any(r != 'SUCCEEDED' for r in results):
                    print('  (stopping: a goal did not succeed)')
                    break
    except Stop:
        if not watch:
            print('\nStopped: cancelling the goal in progress.')
            node.cancel()
    finally:
        try:
            if not watch:
                node.estop(False)
        except Exception:
            pass
        # the laps and the emergency stop first: they never depend on the old trials file
        if lap_rows:
            with open(laps_path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(lap_rows[0]))
                w.writeheader()
                w.writerows(lap_rows)
        if 'press_t' in estop:
            json.dump(estop, open(estop_path, 'w'), indent=1)
            if 'stop_t' in estop:
                print(f'  emergency stop: from {estop["v0"]:.2f} m/s to standstill in '
                      f'{estop["stop_t"] - estop["press_t"]:.2f} s and {estop["stop_d"]:.2f} m; '
                      + (f'moving again {estop["resume_t"] - estop["release_t"]:.1f} s after release'
                         if 'resume_t' in estop else 'did not move again before the lap ended'), flush=True)
        if rows:
            save_trials(label, rows)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    print(f'Done. Per lap and agent -> {TRIALS}; every 0.1 s -> {tick_path}')
    print(f'Report: python3 {os.path.join(TOOLS, "tour_report.py")} <label> [<label> ...]')
    return 0


if __name__ == '__main__':
    sys.exit(main())
