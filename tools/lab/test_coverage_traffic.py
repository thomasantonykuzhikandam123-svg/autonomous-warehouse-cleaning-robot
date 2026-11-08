"""Fake-ROS test of coverage_nav2 v10.5 (traffic-aware cleaning, lanes driven along). No ROS needed.

    python3 test_coverage_traffic.py <path to coverage_nav2.py>

A fake clock (sim time) steps 0.1 s; the node's timers are called on schedule. A fake Nav2
drives the robot straight at the goal at 0.5 m/s unless the test holds it, succeeds within
0.25 m, cancels on request, aborts when told. A fake guard publishes /people_guard/state.
Scenarios: a long guard hold is not a stall; with no guard the old stall still fires; a
waypoint blocked past TRAFFIC_PATIENCE is put off and revisited before home (only its dirty
part) and the zone still counts as cleaned; Nav2 giving up in traffic is retried, not a
failure; no reversing with someone close; a revisit that fails again is skipped for good;
the hard deadline; /mission_state carries the traffic fields.

v10.5: a fake controller (FollowPath) follows the lane polyline and will not drive into a
person standing in it (aborts after failure_tolerance, like RPP's collision check); the fake
Nav2 plans round them (via a point beside them, or W.via_far: through "the next aisle") and
publishes /plan. Scenarios 10-17: lanes driven with FollowPath; standing in the lane -> wait,
go round, carry on along the lane, the patch pass sweeps what was gone round; a short stop is
waited out; a detour out of the aisle is refused (put off, revisited); cleaned = painted lanes;
the patch pass sweeps a lane never driven; stale results ignored; never round a vehicle;
build_mission tags every lane.
"""
import importlib.util
import json
import math
import os
import sys
import tempfile
import types

FAILS = []


def check(name, ok, detail=''):
    print(f'{"ok  " if ok else "FAIL"} {name}{"  (" + str(detail) + ")" if detail != "" else ""}')
    if not ok:
        FAILS.append(name)


P = types.SimpleNamespace


class World:
    def __init__(self):
        self.t = 100.0
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.hold = False            # the test holds the robot (traffic)
        self.speed = 0.5
        self.goal = None             # active fake goal handle
        self.pending = []            # (time, callback) to run
        self.subs = {}
        self.pubs = {}
        self.timers = []
        self.cmd_vel = []
        self.cleared = 0
        self.abort_next = 0          # abort this many goals (status 6) ...
        self.abort_after = 0.5       # ... this long after they are sent
        self.osc = False             # dither back and forth (RPP round someone)
        self.log = []
        self.sent = []
        self.kinds = []              # v10.5: ('nav'|'follow', t, first point, last point)
        self.blocker = None          # v10.5: (x, y) someone standing (personal space)
        self.block_r = 1.6           # the controller stops this close (0.9 + 0.3 + lookahead)
        self.pass_r = 1.3            # Nav2's way round passes this far from them
        self.fail_tol = 5.0          # FollowPath blocked this long -> ABORTED (failure_tolerance)
        self.blocked_for = 0.0
        self.via_far = None          # Nav2's only way round goes through here (the next aisle)
        self.plan_n = 0
        self.trail = []              # (t, x, y)
        self.trail_kind = []         # v10.6: 'follow' | 'nav' | '' at each trail point
        self.blocker_r = 0.9         # v10.6: what /scan_nav shows of her: 0.9 personal space, 0.12 legs
        self.scan_on = True          # v10.6: the guard publishes /scan_nav
        self.scan_next = 0.0
        self.nav_state = 'up'        # v10.6: 'inactive' (rejects goals) | 'gone' (no server, no answers)
        self.walls = []              # v10.6: ((x1, y1), (x2, y2)) segments in /scan_nav (rack faces)
        self.rejected = 0


W = World()


class Clock:
    def now(self):
        t = W.t
        return P(nanoseconds=int(round(t * 1e9)),
                 to_msg=lambda: P(sec=int(t), nanosec=int(round((t - int(t)) * 1e9))))


class Logger:
    def _p(self, lvl, msg):
        W.log.append((round(W.t, 1), lvl, msg))
        if os.environ.get('VERBOSE'):
            print(f'   [{W.t:7.1f}] {lvl} {msg}')

    def info(self, m): self._p('INFO', m)
    def warn(self, m): self._p('WARN', m)
    def error(self, m): self._p('ERROR', m)


class Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []
        W.pubs[topic] = self

    def publish(self, m):
        self.msgs.append(m)
        if self.topic == '/cmd_vel':
            W.cmd_vel.append((W.t, m.linear.x))
        self.msgs = self.msgs[-50:]
        if self.topic == '/plan' and '/plan' in W.subs:
            W.subs['/plan'](m)                 # DDS hands a node its own messages too


class Client:
    def service_is_ready(self):
        return True

    def call_async(self, req):
        W.cleared += 1
        return P()


class Node:
    def __init__(self, name):
        self._clock = Clock()

    def get_clock(self):
        return self._clock

    def get_logger(self):
        return Logger()

    def create_publisher(self, typ, topic, qos):
        return Pub(topic)

    def create_subscription(self, typ, topic, cb, qos):
        W.subs[topic] = cb

    def create_timer(self, period, cb):
        W.timers.append([period, W.t + period, cb])

    def create_client(self, typ, name):
        return Client()

    def declare_parameter(self, name, value):
        pass

    def get_parameter(self, name):
        return P(value=False)

    def add_on_set_parameters_callback(self, cb):
        W.param_cb = cb


class Fut:
    def __init__(self, val):
        self.val, self.cbs = val, []

    def result(self):
        return self.val

    def add_done_callback(self, cb):
        W.pending.append((W.t, lambda: cb(self)))


class GoalHandle:
    def __init__(self, x, y, path=None):
        self.x, self.y = x, y
        self.path = path             # v10.5: FollowPath: the polyline to follow
        self.route = None            # v10.5: NavigateToPose: [via..., goal]
        self.accepted = True
        self.res = None
        self.done = False
        self.cancels = 0

    def get_result_async(self):
        self.res = Fut(None)
        return self.res

    def finish(self, status):
        if self.done:
            return
        self.done = True
        self.res.val = P(status=status)
        for t, cb in list(getattr(self.res, '_cbs', [])):
            pass

    def cancel_goal_async(self):
        self.cancels += 1
        if W.nav_state != 'gone':
            W.pending.append((W.t + 0.2, lambda: end_goal(self, 5)))
        return Fut(None)


def end_goal(gh, status):
    if gh.done or W.goal is not gh:
        return
    gh.done = True
    W.goal = None
    if gh.res is None:                 # v10.7: cancelled before anyone asked for its result (a stale
        return                         # goal the node cancels on arrival): nothing to deliver
    gh.res.val = P(status=status)
    # deliver the result callbacks registered on the result future
    for cb in gh.res.cbs:
        cb(gh.res)


class ResFut(Fut):
    def add_done_callback(self, cb):
        self.cbs.append(cb)


class NeverFut:
    """v10.6: a request to a server that is gone: never answered."""
    def add_done_callback(self, cb):
        pass


class ActionClient:
    def __init__(self, node, typ, name):
        self.name = name

    def wait_for_server(self, timeout_sec=None):
        return True

    def server_is_ready(self):
        return W.nav_state != 'gone'

    def send_goal_async(self, g):
        if W.nav_state == 'gone':                          # v10.6
            W.sent_dead = getattr(W, 'sent_dead', 0) + 1
            return NeverFut()
        if W.nav_state == 'inactive':
            gh = GoalHandle(0.0, 0.0)
            gh.accepted = False
            W.rejected += 1
            return Fut(gh)
        if getattr(g, 'path', None) is not None:           # v10.5: FollowPath
            pts = [(p.pose.position.x, p.pose.position.y) for p in g.path.poses]
            gh = GoalHandle(pts[-1][0], pts[-1][1], path=pts)
            W.kinds.append(('follow', round(W.t, 1), pts[0], pts[-1]))
        else:
            gh = GoalHandle(g.pose.pose.position.x, g.pose.pose.position.y)
            W.kinds.append(('nav', round(W.t, 1), (W.x, W.y), (gh.x, gh.y)))
            gh.route = nav_route((W.x, W.y), (gh.x, gh.y))
            W.pending.append((W.t + 0.1, lambda gh=gh: publish_plan(gh)))
        gh.get_result_async = lambda gh=gh: setattr(gh, 'res', ResFut(None)) or gh.res
        W.sent.append((round(W.t, 1), gh.x, gh.y))
        if W.goal is not None and not W.goal.done:
            end_goal(W.goal, 5)
        W.goal = gh
        if W.abort_next > 0:
            W.abort_next -= 1
            W.pending.append((W.t + W.abort_after, lambda: end_goal(gh, 6)))
        return Fut(gh)


def nav_route(p, goal):
    """The fake planner: straight to the goal, or round whoever stands in the way."""
    if W.blocker is None:
        return [goal]
    bx, by = W.blocker
    dx, dy = goal[0] - p[0], goal[1] - p[1]
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return [goal]
    s = max(0.0, min(L, ((bx - p[0]) * dx + (by - p[1]) * dy) / L))
    cx, cy = p[0] + dx / L * s, p[1] + dy / L * s
    if math.hypot(bx - cx, by - cy) >= W.pass_r:
        return [goal]
    if W.via_far is not None:
        return [W.via_far, goal]
    nx, ny = -dy / L, dx / L                              # pass on the left
    return [(bx + nx * (W.pass_r + 0.1), by + ny * (W.pass_r + 0.1)), goal]


def publish_plan(gh):
    """Nav2's /plan for a NavigateToPose goal (and again every second while it runs)."""
    if gh.done or W.goal is not gh or '/plan' not in W.subs:
        return
    pts, here = [], (W.x, W.y)
    for q in gh.route:
        L = math.hypot(q[0] - here[0], q[1] - here[1])
        n = max(1, int(L / 0.1))
        pts += [(here[0] + (q[0] - here[0]) * k / n, here[1] + (q[1] - here[1]) * k / n) for k in range(n)]
        here = q
    pts.append(here)
    W.plan_n += 1
    msg = P(header=P(frame_id='map', stamp=P(sec=int(W.t), nanosec=W.plan_n)),
            poses=[P(pose=P(position=P(x=x, y=y, z=0.0))) for x, y in pts])
    W.subs['/plan'](msg)
    W.pending.append((W.t + 1.0, lambda: publish_plan(gh)))


def scan_msg():
    """v10.6: /scan_nav from the robot's centre: the blocker as a circle of W.blocker_r."""
    n, a0 = 360, -math.pi
    inc = 2 * math.pi / n
    rng = [float('inf')] * n
    if W.blocker is not None:
        bx, by = W.blocker[0] - W.x, W.blocker[1] - W.y
        R = W.blocker_r
        for k in range(n):
            a = W.yaw + a0 + k * inc                  # the scan is in the laser's frame
            c, s = math.cos(a), math.sin(a)
            t = bx * c + by * s                       # along the ray to the closest approach
            d2 = bx * bx + by * by - t * t
            if t > 0 and d2 <= R * R:
                rng[k] = t - math.sqrt(R * R - d2)
    for (x1, y1), (x2, y2) in W.walls:
        ex, ey = x2 - x1, y2 - y1
        for k in range(n):
            a = W.yaw + a0 + k * inc
            c, s = math.cos(a), math.sin(a)
            den = c * ey - s * ex
            if abs(den) < 1e-9:
                continue
            qx, qy = x1 - W.x, y1 - W.y
            t = (qx * ey - qy * ex) / den                 # along the ray
            u = (qx * s - qy * c) / den                   # along the wall, 0..1
            if t > 0 and 0.0 <= u <= 1.0 and t < rng[k]:
                rng[k] = t
    return P(header=P(frame_id='laser', stamp=None), angle_min=a0, angle_increment=inc,
             range_min=0.3, range_max=8.0, ranges=rng)


def carrot(path, p, look=0.4):
    i0 = min(range(len(path)), key=lambda i: (path[i][0] - p[0]) ** 2 + (path[i][1] - p[1]) ** 2)
    for i in range(i0, len(path)):
        if math.hypot(path[i][0] - p[0], path[i][1] - p[1]) >= look:
            return path[i]
    return path[-1]


class Buffer:
    def lookup_transform(self, a, b, t):
        return P(transform=P(translation=P(x=W.x, y=W.y, z=0.0),
                             rotation=P(x=0.0, y=0.0, z=math.sin(W.yaw / 2), w=math.cos(W.yaw / 2))))


def install_fakes():
    def mod(name, **kw):
        m = types.ModuleType(name)
        m.__dict__.update(kw)
        sys.modules[name] = m
        return m

    class Enum:
        KEEP_LAST = RELIABLE = TRANSIENT_LOCAL = BEST_EFFORT = 1

    class Msg:
        def __init__(self, **kw):
            self.__dict__.update(kw)

    def pose():
        return P(header=P(frame_id='', stamp=None),
                 pose=P(position=P(x=0.0, y=0.0, z=0.0), orientation=P(x=0.0, y=0.0, z=0.0, w=1.0)))

    class Goal:
        def __init__(self):
            self.pose = pose()

    class FPGoal:                                          # nav2_msgs/action/FollowPath
        def __init__(self):
            self.path = None
            self.controller_id = ''
            self.goal_checker_id = ''

    class Path:
        def __init__(self):
            self.header = P(frame_id='', stamp=None)
            self.poses = []

    class PoseStamped:
        def __init__(self):
            self.header = P(frame_id='', stamp=None)
            self.pose = P(position=P(x=0.0, y=0.0, z=0.0), orientation=P(x=0.0, y=0.0, z=0.0, w=1.0))

    class String:
        def __init__(self, data=''):
            self.data = data

    class Twist:
        def __init__(self):
            self.linear = P(x=0.0, y=0.0, z=0.0)
            self.angular = P(x=0.0, y=0.0, z=0.0)

    class Marker:
        CUBE_LIST = 6
        ADD = 0

        def __init__(self):
            self.header = P(frame_id='', stamp=None)
            self.scale = P(x=0, y=0, z=0)
            self.color = P(r=0, g=0, b=0, a=0)
            self.pose = P(orientation=P(w=1.0))
            self.points = []

    class MarkerArray:
        def __init__(self):
            self.markers = []

    class Point:
        def __init__(self):
            self.x = self.y = self.z = 0.0

    rclpy = mod('rclpy', init=lambda **k: None, spin=lambda n: None, shutdown=lambda: None)
    rclpy.time = mod('rclpy.time', Time=lambda *a, **k: None)
    mod('rclpy.node', Node=Node)
    mod('rclpy.action', ActionClient=ActionClient)
    mod('rclpy.qos', QoSProfile=lambda **k: None, QoSDurabilityPolicy=Enum,
        QoSReliabilityPolicy=Enum, QoSHistoryPolicy=Enum)
    mod('nav2_msgs')
    mod('nav2_msgs.action', NavigateToPose=P(Goal=Goal), FollowPath=P(Goal=FPGoal))
    mod('nav2_msgs.srv', ClearEntireCostmap=P(Request=lambda: None))
    mod('rcl_interfaces')
    mod('rcl_interfaces.msg', SetParametersResult=lambda **k: P(**k))
    mod('nav_msgs')
    mod('nav_msgs.msg', OccupancyGrid=object, Path=Path)
    mod('std_msgs')
    mod('std_msgs.msg', String=String)
    mod('sensor_msgs')
    mod('sensor_msgs.msg', LaserScan=object)
    mod('visualization_msgs')
    mod('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
    mod('geometry_msgs')
    mod('geometry_msgs.msg', Point=Point, Twist=Twist, PoseStamped=PoseStamped)
    mod('tf2_ros', Buffer=Buffer, TransformListener=lambda buf, node: None)


GUARD = {'on': False, 'state': 'go', 'person_m': None, 'vehicle_m': None, 'say': 'clear'}


def guard_msg():
    pm = GUARD['person_m']
    if pm is None and W.blocker is not None:                # v10.5: the one standing in the lane
        pm = round(math.hypot(W.blocker[0] - W.x, W.blocker[1] - W.y), 2)
    s = dict(t=W.t, state=GUARD['state'], why='vehicle' if GUARD['state'] == 'wait' else 'clear',
             say=GUARD['say'], v_limit=GUARD.get('v_limit'), limit=GUARD.get('limit'), person_m=pm,
             vehicle_m=GUARD['vehicle_m'], people=0, vehicles=0, standing=0, version='v10.4')
    return types.SimpleNamespace(data=json.dumps(s))


def step(dt=0.1):
    W.t = round(W.t + dt, 6)
    # robot motion (fake Nav2): straight to the goal unless held
    g = W.goal
    if g is not None and not g.done and not W.hold and W.nav_state == 'up':
        d = math.hypot(g.x - W.x, g.y - W.y)
        if d <= 0.25:
            gg = g
            W.pending.append((W.t, lambda: end_goal(gg, 4)))
        else:
            if g.path is not None:                       # FollowPath: along the lane
                tx, ty = carrot(g.path, (W.x, W.y))
            else:                                        # NavigateToPose: round people
                while len(g.route) > 1 and math.hypot(g.route[0][0] - W.x, g.route[0][1] - W.y) < 0.15:
                    g.route.pop(0)
                tx, ty = g.route[0]
            dx, dy = tx - W.x, ty - W.y
            dd = max(math.hypot(dx, dy), 1e-9)
            mv = min(W.speed * dt, dd)
            if W.osc and int(W.t) % 2:
                mv = -mv                      # one second back, one forward
            nx, ny = W.x + dx / dd * mv, W.y + dy / dd * mv
            if g.path is not None and W.blocker is not None:
                bx, by = W.blocker
                d0, d1 = math.hypot(bx - W.x, by - W.y), math.hypot(bx - nx, by - ny)
                if d1 < W.block_r and d1 < d0:           # RPP: collision ahead, stands
                    nx, ny = W.x, W.y
                    W.blocked_for += dt
                    if W.blocked_for >= W.fail_tol - 1e-9:
                        W.blocked_for = 0.0
                        gg = g
                        W.pending.append((W.t, lambda: end_goal(gg, 6)))
                else:
                    W.blocked_for = 0.0
            if (nx, ny) != (W.x, W.y):
                W.yaw = math.atan2(ny - W.y, nx - W.x)
            W.x, W.y = nx, ny
    W.trail.append((W.t, W.x, W.y))
    W.trail_kind.append('follow' if (g is not None and not g.done and g.path is not None) else
                        'nav' if (g is not None and not g.done) else '')
    if GUARD['on'] and '/people_guard/state' in W.subs:
        W.subs['/people_guard/state'](guard_msg())
    if W.scan_on and GUARD['on'] and '/scan_nav' in W.subs and W.t >= W.scan_next:
        W.scan_next = W.t + 0.1                      # 10 Hz, like the lidar
        W.subs['/scan_nav'](scan_msg())
    due = [p for p in W.pending if p[0] <= W.t]
    W.pending = [p for p in W.pending if p[0] > W.t]
    for _, cb in due:
        cb()
    for tm in W.timers:
        if W.t >= tm[1] - 1e-9:
            tm[1] += tm[0]
            tm[2]()


def run_until(cond, limit):
    t_end = W.t + limit
    while W.t < t_end:
        step()
        if cond():
            return True
    return False


def load(path):
    spec = importlib.util.spec_from_file_location('coverage_nav2_t', path)
    m = importlib.util.module_from_spec(spec)
    sys.path.insert(0, os.path.dirname(os.path.abspath(path)))
    spec.loader.exec_module(m)
    return m


def fake_geom(goals, zones):
    """v10.5: the aisles' lanes as this test's mission drives them (x of each goal, y span)."""
    geo = {}
    for (x, y, _), z in zip(goals, zones):
        if str(z).startswith('aisle_'):
            g = geo.setdefault(z, [set(), set()])
            g[0].add(round(x, 3))
            g[1].add(y)
    return {z: (sorted(xs), min(ys), max(ys), min(xs) - 0.3, max(xs) + 0.3) for z, (xs, ys) in geo.items()}


def fresh(m, goals, zones, lanes=False, sweeps=None, geom=None, patch=True):
    """A node with its own mission, as if 'clean' had been sent (zone totals set).
    v10.5: lanes=True drives sweeps ({end goal: start}) with FollowPath; the aisles'
    geometry (for the painted % and the patch pass) is this mission's lanes."""
    W.__init__()
    GUARD.update(on=False, state='go', person_m=None, vehicle_m=None, say='clear', v_limit=None, limit=None)
    n = m.CoverageNav2()
    geo = fake_geom(goals, zones) if geom is None else geom
    n._aisle_geom = lambda z, geo=geo: geo.get(z)
    n.LANES_FOLLOW = lanes
    n.PATCH = patch
    n.cleaned, n.zone_pct = [], {}      # every scenario starts from a dirty floor (the file persists)
    n.sweeps = dict(sweeps or {})
    n.goals = [g for g in goals]
    n.goal_zones = list(zones)
    n.gi = 0
    n._done = False
    n._zone_total = {}
    for z in zones:
        if z not in ('start', 'home'):
            n._zone_total[z] = n._zone_total.get(z, 0) + 1
    n._epoch += 1
    n._awaiting = False                 # v10.7: as if 'clean' had been sent (auto_start is false here)
    n.send_next()
    return n


def logs(kind):
    return [m for _, l, m in W.log if kind in m]


def main(path):
    install_fakes()
    os.environ['HOME'] = tempfile.mkdtemp()
    m = load(path)
    lane = [(0.0, 0.0, 'wp1'), (0.0, 6.0, 'wp2'), (0.5, 6.0, 'wp3'), (0.5, 0.0, 'wp4'), (0.0, 0.0, 'HOME')]
    zones = ['aisle_1', 'aisle_1', 'aisle_1', 'aisle_1', 'home']

    # 0. no guard, no traffic: the mission runs as it always did
    n = fresh(m, lane, zones)
    done = run_until(lambda: n._done, 200)
    check('0. plain run: completes, aisle_1 cleaned, no traffic log', done and n.cleaned == ['aisle_1']
          and not logs('TRAFFIC') and not logs('DEFER') and n.holds == 0, (n.cleaned, n.holds))
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('0. mission_state without a guard: traffic None, guard False', ms.get('traffic') is None
          and ms.get('guard') is False and ms.get('deferred') == [], {k: ms.get(k) for k in ('traffic', 'guard')})

    # 1. a long guard hold mid-lane is not a stall
    n = fresh(m, lane, zones)
    GUARD['on'] = True
    run_until(lambda: W.y > 3.0, 60)
    GUARD.update(state='wait', vehicle_m=1.5, say='giving way to a vehicle')
    W.hold = True
    n_before = len(W.sent)
    run_until(lambda: False, 45)
    check('1. 45 s guard hold: no STALL', not logs('STALL'), logs('STALL')[:1])
    check('1. no goal cancelled or resent during the hold', len(W.sent) == n_before, W.sent)
    GUARD.update(state='go', vehicle_m=None, say='clear')
    W.hold = False
    done = run_until(lambda: n._done, 200)
    check('1. mission completes after the hold', done and n.cleaned == ['aisle_1'], n.cleaned)
    check('1. one hold, about 45 s, counted', n.holds == 1 and 43 <= n.held_s <= 47, (n.holds, round(n.held_s, 1)))
    check('1. nothing put off or skipped', not n.deferred and not n.skipped and n.revisited == 0)
    check('1. logged once while waiting', len(logs('TRAFFIC: wp')) >= 2, logs('TRAFFIC')[:2])

    # 2. no guard running: the old stall detector still fires after 15 s
    n = fresh(m, lane, zones)
    run_until(lambda: W.y > 3.0, 60)
    W.hold = True
    t_hold = W.t
    run_until(lambda: logs('STALL'), 30)
    st = [t for t, l, msg in W.log if 'STALL' in msg]
    check('2. no guard: STALL 15-16 s after it stopped', bool(st) and 15.0 <= st[0] - t_hold <= 16.5,
          [round(x - t_hold, 1) for x in st[:1]])
    W.hold = False
    run_until(lambda: n._done, 300)
    check('2. no guard: retry ladder as before ("retry 1")', bool(logs('retry 1')))

    # 3. blocked past TRAFFIC_PATIENCE: put off, revisited before home, only the dirty part
    lane3 = [(0.0, 0.0, 'wp1'), (0.0, 10.0, 'wp2'), (3.0, 10.0, 'wp3'), (3.0, 0.0, 'wp4'), (0.0, 0.0, 'HOME')]
    n = fresh(m, lane3, zones)
    GUARD['on'] = True
    run_until(lambda: W.y > 3.0, 60)
    GUARD.update(state='go', person_m=1.0, say='clear')        # someone standing in the lane
    W.hold = True                                               # Nav2 will not pass her
    run_until(lambda: n.deferred, 90)
    check('3. put off after TRAFFIC_PATIENCE (60 s) standing', len(n.deferred) == 1 and bool(logs('DEFER')),
          [d['label'] for d in n.deferred])
    y_def = W.y
    GUARD.update(person_m=None)
    W.hold = False
    run_until(lambda: n._done, 400)
    rv = [g for g in W.sent if g[1] == 0.0 and g[2] > 2.9]
    check('3. revisit: goes back for the dirty stretch of lane 1 before home', bool(logs('REVISIT')), logs('REVISIT')[:1])
    revis = [s for s in W.sent if abs(s[1]) < 1e-6 and s[2] >= y_def - 0.3]
    rv = [s for s in W.sent if abs(s[1]) < 1e-6 and 0.5 < s[2] < 9.5]
    check('3. revisit starts near where it was blocked (y %.1f), not at the lane start' % y_def,
          bool(rv) and abs(rv[0][2] - y_def) < 1.5, W.sent)
    check('3. ... and drives the rest of the lane (to y 10)', any(abs(s[1]) < 1e-6 and s[2] > 9.9 for s in W.sent[2:]), W.sent)
    check('3. zone still counts as cleaned (revisit credited)', n.cleaned == ['aisle_1'], (n.cleaned, n._zone_done))
    check('3. nothing skipped', not n.skipped, n.skipped)
    check('3. home reached last', abs(W.x) < 0.3 and abs(W.y) < 0.3, (round(W.x, 2), round(W.y, 2)))

    # 4. Nav2 gives up in traffic: same goal again, not a failure
    n = fresh(m, lane, zones)
    GUARD['on'] = True
    GUARD.update(state='wait', vehicle_m=2.0, say='giving way to a vehicle')
    W.abort_next = 3
    W.hold = True
    run_until(lambda: len(W.sent) >= 4, 30)
    check('4. three aborts in traffic -> three resends of the same goal', len(W.sent) >= 4 and
          len({(s[1], s[2]) for s in W.sent}) == 1, W.sent[:5])
    check('4. no rescue, no "retry 1" ladder', not logs('RESCUE') and not logs('retry 1'))
    GUARD.update(state='go', vehicle_m=None)
    W.hold = False
    run_until(lambda: n._done, 300)
    check('4. completes', n._done and n.cleaned == ['aisle_1'])

    # 5. failures outside traffic, but someone within 1.5 m by the time of the rescue: no reversing
    n = fresh(m, lane, zones)
    GUARD['on'] = True
    run_until(lambda: W.y > 2.0, 60)
    W.hold = True
    n.TRAFFIC_MAX_RETRIES = 0                 # straight to the ladder
    GUARD.update(state='go', person_m=1.2)    # close, so no reversing
    W.abort_next, W.abort_after = 2, 4.0
    n._goal_handle.cancel_goal_async()
    run_until(lambda: logs('RESCUE'), 30)
    check('5. rescue without reversing when someone is within 1.5 m', bool(logs('RESCUE without reversing')),
          logs('RESCUE')[:1])
    check('5. no reverse command sent', not any(v < 0 for _, v in W.cmd_vel), W.cmd_vel[:3])
    GUARD.update(person_m=None)
    W.hold = False
    run_until(lambda: n._done, 300)

    # 6. a revisit that fails again is skipped for good (no endless loop)
    n = fresh(m, lane, zones)
    GUARD['on'] = True
    run_until(lambda: W.y > 3.0, 60)
    GUARD.update(person_m=1.0)
    W.hold = True
    run_until(lambda: n.deferred, 90)
    run_until(lambda: n._is_revisit(), 300)
    run_until(lambda: n._done or n.skipped, 200)
    check('6. revisit blocked again: skipped for good, mission ends', bool(n.skipped) and bool(logs('SKIP')),
          (n.skipped, n._done))
    W.hold = False
    GUARD.update(person_m=None)
    run_until(lambda: n._done, 300)
    check('6. home after the skip', n._done)

    # 7. hard deadline: crawling in traffic forever is put off too
    n = fresh(m, lane, zones)
    GUARD['on'] = True
    GUARD.update(state='go', person_m=1.5)
    W.osc = True                                     # dithering round her: moving, never "standing"
    run_until(lambda: n.deferred or logs('DEFER'), 600)
    check('7. hard deadline (3x the normal time) puts the waypoint off', bool(logs('normal time')),
          logs('normal time')[:1])
    W.osc = False
    GUARD.update(person_m=None)
    run_until(lambda: n._done, 400)

    # 9. a busy stretch: many holds, progress in between, 90 s standing in all: NOT put off
    n = fresh(m, [(0.0, 0.0, 'wp1'), (0.0, 12.0, 'wp2'), (0.0, 0.0, 'HOME')], ['aisle_1', 'aisle_1', 'home'])
    GUARD['on'] = True
    run_until(lambda: W.y > 1.0, 30)
    for k in range(6):                                    # 6 holds of 15 s, 1 m of progress between
        GUARD.update(state='wait', vehicle_m=1.5, say='giving way to a vehicle')
        W.hold = True
        run_until(lambda: False, 15)
        GUARD.update(state='go', vehicle_m=None, say='clear')
        W.hold = False
        y0 = W.y
        run_until(lambda: W.y > y0 + 1.0, 10)
    run_until(lambda: n._done, 200)
    check('9. 6 holds of 15 s with progress between: nothing put off', not logs('DEFER') and n._done
          and n.cleaned == ['aisle_1'] and n.holds == 6, (n.holds, logs('DEFER')[:1]))

    # 8. /mission_state carries the traffic fields
    n = fresh(m, lane, zones)
    GUARD['on'] = True
    GUARD.update(state='wait', vehicle_m=1.2, say='giving way to a vehicle: going on would pass 0.4 m from it')
    W.hold = True
    run_until(lambda: False, 6)
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('8. mission_state: traffic say, for_s, holds', ms.get('traffic', {}).get('say', '').startswith('giving way')
          and ms['traffic']['for_s'] >= 5 and ms['holds'] == 1 and ms['guard'] is True,
          {k: ms.get(k) for k in ('traffic', 'holds', 'held_s', 'waiting_s')})
    GUARD.update(state='go', vehicle_m=None, say='clear')
    W.hold = False
    run_until(lambda: False, 3)
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('8. last_hold kept after it ended', (ms.get('last_hold') or {}).get('say', '').startswith('giving way')
          and ms['last_hold']['for_s'] >= 5, ms.get('last_hold'))
    lanes_v105(m)
    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


def follows():
    return [k for k in W.kinds if k[0] == 'follow']


def navs():
    return [k for k in W.kinds if k[0] == 'nav']


def v106(m, L2, Z2, SW2):
    """v10.6: clearance in the lane, vehicles at lane ends and starts, the guard falling silent,
    a frozen simulation."""
    # 20. someone standing in the lane whom nothing recognises (her legs only, as for the stepper
    # at rack 2's end in shift_v105a): the controller alone stops only at her legs (0.55 m from
    # her centre); the robot must stop short (LANE_STOP 0.35 m from her legs)
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.blocker, W.blocker_r, W.block_r = (0.0, 4.0), 0.12, 0.55
    run_until(lambda: logs('CLEARANCE'), 40)
    run_until(lambda: False, 6)
    near = min(math.hypot(x, y - 4.0) for t, x, y in W.trail)
    check('20. stops short of a standing person nothing recognises (>= 0.70 m from her centre; '
          'the controller alone: 0.55)', bool(logs('CLEARANCE')) and near >= 0.70, round(near, 2))
    run_until(lambda: logs('BYPASS') or n._done, 40)
    check('20. ... then goes round her after waiting', bool(logs('BYPASS')), logs('BYPASS')[:1])
    run_until(lambda: W.goal is not None and W.goal.path is not None and abs(W.goal.x - 0.5) < 1e-6, 120)
    t_gone = W.t
    W.blocker, W.blocker_r, W.block_r = None, 0.9, 1.6
    run_until(lambda: n._done, 300)
    check('20. ... and the job completes, aisle cleaned', n._done and n.cleaned == ['aisle_1'],
          (n._done, n.cleaned, n.swept_pct))
    near2 = min(math.hypot(x, y - 4.0) for (t, x, y), k in zip(W.trail, W.trail_kind)
                if t < t_gone and k != 'nav')
    check('20. driving the lane, never closer than 0.70 m to her centre while she stood there '
          '(going round is Nav2 planning round her)', near2 >= 0.70, round(near2, 2))

    # 20b. a rack face beside an edge lane (0.42 m from its line) and the robot set off 0.12 m off
    # the lane towards it (just onto the lane): no stop for the rack (shift_v105a, rack 2's corner)
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.walls = [((-0.42, -0.5), (-0.42, 9.0))]
    W.x = -0.12
    run_until(lambda: n._done, 300)
    check('20b. a rack face 0.42 m beside the lane, the robot 0.12 m off the lane towards it: no stop, '
          'the job completes', n._done and not logs('CLEARANCE') and n.cleaned == ['aisle_1'],
          (logs('CLEARANCE')[:1], n.cleaned))
    W.walls = []

    # 21. a vehicle holds the robot near the end of its lane: after 10 s the lane is done there
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: W.y > 4.5, 60)
    GUARD.update(state='wait', vehicle_m=None, say='waiting: a vehicle is standing in my way and its forks may be out')
    W.hold = True
    t0 = W.t
    run_until(lambda: logs('VEHICLE'), 30)
    check('21. held 10 s for a vehicle 3.5 m from the lane end: lane done there (not 40 s)',
          bool(logs('VEHICLE')) and 'end of lane' in logs('VEHICLE')[0] and 9.5 <= W.t - t0 <= 12.0,
          (logs('VEHICLE')[:1], round(W.t - t0, 1)))
    GUARD.update(state='go', say='clear')
    W.hold = False
    run_until(lambda: n._done, 300)
    check('21. ... the job completes with the lane end swept (by the way to the next lane or the patch '
          'pass), aisle cleaned', n._done and n.cleaned == ['aisle_1'] and n.vehicle_skips == 1
          and n.swept_pct['aisle_1'] >= 90, (n.cleaned, n.vehicle_skips, n.swept_pct))
    f2 = [k for k in follows() if abs(k[2][0] - 0.5) < 1e-6]
    to_wp3 = [s for s in W.sent if abs(s[1] - 0.5) < 1e-6 and abs(s[2] - 8.0) < 1e-6
              and f2 and s[0] < f2[0][1]]
    check('21. ... the next lane starts beside that end (same vehicle): begun 4 m in at once, its start '
          'not driven to', bool(f2) and abs(f2[0][2][1] - 4.0) < 0.3 and not to_wp3
          and 'starts beside it' in logs('VEHICLE')[0], (f2[:1], to_wp3[:2]))

    # 22. a vehicle holds the robot on the way to the next lane's start: that lane starts 4 m in
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: n.gi == 2 and W.goal is not None, 80)            # on the way to wp3 (0.5, 8)
    GUARD.update(state='wait', say='giving way to a vehicle')
    W.hold = True
    run_until(lambda: logs('VEHICLE'), 30)
    GUARD.update(state='go', say='clear')
    W.hold = False
    run_until(lambda: any(k[0] == 'follow' and abs(k[2][0] - 0.5) < 1e-6 for k in W.kinds), 60)
    f1 = [k for k in follows() if abs(k[2][0] - 0.5) < 1e-6]
    check('22. lane start held by a vehicle: lane 2 begun 4 m in (y 4.0, not 8.0)',
          bool(logs('VEHICLE')) and 'start of the next lane' in logs('VEHICLE')[0] and bool(f1)
          and abs(f1[0][2][1] - 4.0) < 0.3, (logs('VEHICLE')[:1], f1[:1]))
    run_until(lambda: n._done, 300)
    check('22. ... its first 4 m swept by the patch pass, aisle cleaned', n._done and n.cleaned == ['aisle_1']
          and n.patches >= 1, (n.cleaned, n.patches, n.swept_pct))

    # 23. the guard falls silent mid-lane (guard or lidar down): hold still until it is back
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: W.y > 3.0, 60)
    GUARD['on'] = False
    y_off = W.y
    run_until(lambda: logs('SAFETY: no word'), 5)
    sent0 = len(W.sent)
    run_until(lambda: False, 20)
    moved = max(y for t, x, y in W.trail[-200:]) - y_off
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('23. guard silent: holds still within ~2.5 s, no goals sent while silent',
          bool(logs('SAFETY: no word')) and moved < 1.5 and len(W.sent) == sent0 and W.goal is None
          and ms.get('safety_hold') is True, (logs('SAFETY')[:1], round(moved, 2), len(W.sent) - sent0))
    GUARD['on'] = True
    run_until(lambda: n._done, 300)
    check('23. ... guard back: carries on and completes', bool(logs('the people guard is back')) and n._done
          and n.cleaned == ['aisle_1'], (logs('SAFETY')[-1:], n.cleaned))
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)       # no guard at all: no hold (as before)
    run_until(lambda: n._done, 200)
    check('23. no guard from the start: no safety hold, completes as before', n._done and not logs('SAFETY'))

    # 24. the simulation freezes (real time passes, sim time does not): logged, nothing else
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: W.y > 2.0, 60)
    real = m.time.monotonic
    m.time.monotonic = lambda: real() + 1345.0
    run_until(lambda: logs('SIM PAUSED'), 2)
    m.time.monotonic = real
    run_until(lambda: n._done, 300)
    check('24. a frozen simulation is logged (SIM PAUSED), the job goes on', bool(logs('SIM PAUSED'))
          and n._done and n.sim_pauses >= 1, logs('SIM PAUSED')[:1])

    # 26. Nav2 stops taking goals mid-lane (deactivated by its lifecycle manager: goals rejected),
    # then comes back: the robot holds at the waypoint, skips nothing, carries on
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: W.y > 3.0, 60)
    W.nav_state = 'inactive'
    gi0 = n.gi
    end_goal(W.goal, 6)                                  # its running goal is aborted
    run_until(lambda: False, 60)
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('26. Nav2 rejecting goals: holds at the waypoint (NAV2: logged once, nothing skipped, '
          'a try every 5 s)', bool(logs('NAV2: 3 goals rejected')) and n.gi == gi0 and not n.skipped
          and not n.deferred and not n._done and ms.get('nav2_down') is True and 10 <= W.rejected <= 16,
          (logs('NAV2')[:1], n.gi, gi0, W.rejected))
    W.nav_state = 'up'
    run_until(lambda: n._done, 300)
    check('26. ... Nav2 back: carries on and completes, aisle cleaned', bool(logs('NAV2: taking goals again'))
          and n._done and n.cleaned == ['aisle_1'] and not n.skipped, (logs('NAV2')[-1:], n.cleaned, n.skipped))
    # ... and Nav2 gone altogether (crashed): no answers, no server
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: W.y > 3.0, 60)
    W.nav_state = 'gone'
    gi0 = n.gi
    run_until(lambda: False, 120)
    dead = getattr(W, 'sent_dead', 0)
    check('26. Nav2 gone mid-lane: 3 s later it holds (NAV2: logged), skips nothing, sends nothing '
          'while there is no server', bool(logs('NAV2: the Nav2 server went away')) and n.gi == gi0 and not n.skipped
          and not n.deferred and dead == 0, (logs('NAV2')[:1], n.gi, gi0, dead))
    W.nav_state = 'up'
    run_until(lambda: n._done, 300)
    check('26. ... Nav2 back: carries on and completes', n._done and n.cleaned == ['aisle_1'] and not n.skipped,
          (n.cleaned, n.skipped, logs('NAV2')[-1:]))
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.nav_state = 'inactive'
    W.pending.append((W.t + 1.5, lambda: setattr(W, 'nav_state', 'up')))
    run_until(lambda: n._done, 300)
    check('26. a goal rejected now and then: tried again, nothing skipped (v10.5 skipped it)',
          n._done and W.rejected >= 1 and not logs('NAV2:') and not n.skipped and n.cleaned == ['aisle_1'],
          (W.rejected, logs('REJECTED')[:1]))

    # ... Nav2 gone between goals, back before the deadline: the waypoint is not skipped
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: W.y > 3.0, 60)
    W.nav_state = 'gone'
    run_until(lambda: False, 3.5)                       # dropped (3 s), then retries find no server
    W.nav_state = 'up'
    gi0 = n.gi
    run_until(lambda: n._done, 300)
    check('26. Nav2 gone for 3.5 s: nothing skipped, the job completes', n._done and not n.skipped
          and not logs('TIMEOUT (no active goal)') and n.cleaned == ['aisle_1'], (logs('NAV2')[:2], n.skipped))
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: n.gi == 2 and W.goal is None, 80)  # between goals
    W.nav_state = 'gone'
    run_until(lambda: False, 2.5)
    W.nav_state = 'up'
    run_until(lambda: n._done, 300)
    check('26. Nav2 gone for 2.5 s between goals: nothing sent into the void, nothing skipped',
          n._done and not n.skipped and not logs('TIMEOUT (no active goal)') and n.cleaned == ['aisle_1']
          and getattr(W, 'sent_dead', 0) == 0, (getattr(W, 'sent_dead', 0), logs('TIMEOUT')[:2]))

    # 27. a vehicle holds the robot in the patch pass: that stretch after the others
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.blocker, W.blocker_r, W.block_r = (0.0, 4.0), 0.9, 1.6
    run_until(lambda: logs('BYPASS'), 120)
    W.blocker = None
    run_until(lambda: logs('PATCH:'), 200)
    run_until(lambda: n.gi < len(n.goals) and str(n.goals[n.gi][2]).startswith('pt'), 5)
    GUARD.update(state='wait', say='giving way to a vehicle')
    W.hold = True
    t0 = W.t
    run_until(lambda: logs('VEHICLE: a vehicle held it'), 40)
    held = W.t - t0
    GUARD.update(state='go', say='clear')
    W.hold = False
    run_until(lambda: n._done, 300)
    check('27. patch pass held 10 s by a vehicle: that stretch put after the others (not up to 60 s)',
          bool(logs('VEHICLE: a vehicle held it')) and 9.5 <= held <= 12.5, (logs('VEHICLE')[:1], round(held, 1)))
    check('27. ... and swept on the way back: the job completes, aisle cleaned', n._done
          and n.cleaned == ['aisle_1'] and not n.skipped and bool(logs('REVISIT')), (n.cleaned, n.swept_pct))

    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.blocker, W.blocker_r, W.block_r = (0.0, 4.0), 0.9, 1.6
    run_until(lambda: logs('BYPASS'), 120)
    W.blocker = None
    run_until(lambda: logs('PATCH:'), 200)
    run_until(lambda: n._cur_sweep() is not None and n._cur_sweep()['phase'] == 'follow'
              and str(n.goals[n.gi][2]).startswith('pt') and n._goal_handle is not None, 60)
    run_until(lambda: False, 1.0)
    GUARD.update(state='wait', say='giving way to a vehicle')
    W.hold = True
    run_until(lambda: logs('VEHICLE: a vehicle held it'), 40)
    GUARD.update(state='go', say='clear')
    W.hold = False
    run_until(lambda: n._done, 300)
    check('27. ... held on the stretch itself: the rest of it put off (not skipped for good), swept after',
          bool(logs('on patch stretch')) and not n.skipped and n._done and n.cleaned == ['aisle_1']
          and bool(logs('REVISIT')), (logs('VEHICLE')[:1], n.skipped, n.swept_pct))

    # 25. someone stands in the lane through the first patch round (a picker at a rack face):
    # a second round for the aisle still under 90 %; never a third
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.blocker = (0.0, 4.0)
    run_until(lambda: logs('PATCH round 2') or n._done, 500)
    r2 = logs('PATCH round 2')
    W.blocker = None
    run_until(lambda: n._done, 300)
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('25. still blocked in the patch pass: a second round, and the aisle gets cleaned',
          bool(r2) and n._done and n.cleaned == ['aisle_1'] and n.swept_pct['aisle_1'] >= 90
          and ms.get('patch_round') == 2, (r2[:1], n.cleaned, n.swept_pct))
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.blocker = (0.0, 4.0)
    run_until(lambda: n._done, 900)
    check('25. ... blocked for good: two rounds only, then home, the aisle honestly not cleaned',
          n._done and len(logs('PATCH round 2')) == 1 and len(logs('PATCH:')) == 1 and n.cleaned == []
          and n.swept_pct['aisle_1'] < 90, (len(logs('PATCH round 2')), n.cleaned, n.swept_pct))
    W.blocker = None
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: n._done, 300)
    check('25. ... nothing in the way: no second round', n._done and not logs('PATCH round 2')
          and n.cleaned == ['aisle_1'])


def lanes_v105(m):
    """v10.5 scenarios: lanes driven along with FollowPath, going round, the patch pass."""
    L2 = [(0.0, 0.0, 'wp1'), (0.0, 8.0, 'wp2'), (0.5, 8.0, 'wp3'), (0.5, 0.0, 'wp4'), (0.0, 0.0, 'HOME')]
    Z2 = ['aisle_1', 'aisle_1', 'aisle_1', 'aisle_1', 'home']
    SW2 = {L2[1]: (0.0, 0.0), L2[3]: (0.5, 8.0)}

    # 10. lanes are driven with FollowPath along the lane; the lane goes on /plan
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    plans = []
    run_until(lambda: len(follows()) >= 1 and W.y > 3.0, 60)
    plans = [msg for msg in W.pubs['/plan'].msgs] if '/plan' in W.pubs else []
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    done = run_until(lambda: n._done, 200)
    f = follows()
    check('10. both lanes sent as FollowPath goals along x=0 and x=0.5',
          len(f) == 2 and abs(f[0][2][0]) < 1e-6 and abs(f[0][3][1] - 8.0) < 1e-6
          and abs(f[1][2][0] - 0.5) < 1e-6 and abs(f[1][3][1]) < 1e-6, f)
    check('10. the lane is on /plan while it is driven (all of it on x=0)',
          bool(plans) and all(abs(p.pose.position.x) < 1e-6 for p in plans[-1].poses), len(plans))
    check('10. our own /plan never read as a detour', not logs('DETOUR'), logs('DETOUR')[:1])
    check('10. /mission_state: lane phase follow, swept %', (ms.get('lane') or {}).get('phase') == 'follow'
          and 'aisle_1' in (ms.get('swept') or {}), {k: ms.get(k) for k in ('lane', 'swept')})
    check('10. completes; aisle_1 cleaned by its painted lanes', done and n.cleaned == ['aisle_1']
          and n.swept_pct.get('aisle_1', 0) >= 90, (n.cleaned, n.swept_pct))
    check('10. nothing to patch', bool(logs('nothing to patch')) and n.patches == 0, logs('PATCH'))

    # 11. someone stands in the lane: wait, go round, on along the lane; patch what was gone round
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.blocker = (0.0, 4.0)
    run_until(lambda: logs('BYPASS'), 60)
    t_by = W.t
    stop_y = max(y for t, x, y in W.trail if abs(x) < 0.05 and t < t_by)
    stood = [t for t, x, y in W.trail if abs(y - stop_y) < 0.02 and abs(x) < 0.02]
    check('11. stopped 1.5-2.0 m short of her (0.9 m personal space + 0.35 m + braking)',
          1.5 <= 4.0 - stop_y <= 2.0, round(stop_y, 2))
    check('11. waited about 15 s before going round (not straight round her)',
          bool(stood) and 14.0 <= t_by - stood[0] <= 17.5, round(t_by - stood[0], 1) if stood else None)
    check('11. it stopped short of her on its own (CLEARANCE) or Nav2 gave up in traffic: never '
          'the retry/rescue ladder', (bool(logs('CLEARANCE')) or bool(logs('in traffic')))
          and not logs('retry 1') and not logs('RESCUE'), (logs('CLEARANCE')[:1], logs('in traffic')[:1]))
    run_until(lambda: any(k[0] == 'follow' and k[1] > t_by for k in W.kinds) or n._done, 60)
    nb = navs()[-1]
    f = follows()
    check('11. went round to a point on the lane past her (y > 5.4)',
          abs(nb[3][0]) < 1e-6 and nb[3][1] > 5.4, nb)
    side = max(abs(x) for t, x, y in W.trail if t >= t_by and y < 6.0)
    check('11. ... passing beside her, not through the next aisle (max 1.6 m off the lane)', side <= 1.6, round(side, 2))
    check('11. then on along the lane from there with FollowPath', len(f) >= 2 and f[-1][2][1] > 5.0
          and abs(f[-1][2][0]) < 1e-6 and abs(f[-1][3][1] - 8.0) < 1e-6, f[-2:])
    run_until(lambda: W.goal is not None and W.goal.path is not None and abs(W.goal.x - 0.5) < 1e-6, 120)
    W.blocker = None                                            # she has gone
    run_until(lambda: n._done, 300)
    pt = [k for k in follows() if k[1] > t_by + 20 and abs(k[2][0]) < 1e-6]
    check('11. patch pass before home sweeps the stretch gone round (x=0, y ~2.4..5.6)',
          n.patches >= 1 and bool(pt) and pt[0][2][1] < stop_y + 0.4 and pt[0][3][1] > 5.0, (n.patches, pt))
    check('11. aisle_1 cleaned (lanes painted >= 90 %), 1 bypass counted',
          n.cleaned == ['aisle_1'] and n.swept_pct['aisle_1'] >= 90 and n.bypasses == 1,
          (n.cleaned, n.swept_pct, n.bypasses))
    check('11. home reached last', n._done and abs(W.x) < 0.3 and abs(W.y) < 0.3, (round(W.x, 2), round(W.y, 2)))

    # 12. a picker stands 8 s: waited out, no going round, nothing to patch
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.blocker = (0.0, 4.0)
    run_until(lambda: W.blocked_for > 0.5 or n._ahead_hold, 60)
    run_until(lambda: False, 8)
    W.blocker = None
    run_until(lambda: n._done, 200)
    check('12. 8 s stop: no BYPASS, lane finished along x=0', not logs('BYPASS') and n._done
          and n.cleaned == ['aisle_1'], (logs('BYPASS')[:1], n.cleaned))
    check('12. ... and nothing left to patch', n.patches == 0, logs('PATCH'))

    # 13. the only way round is out of the aisle: refused, put off, revisited (she has gone)
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    W.blocker = (0.0, 4.0)
    W.via_far = (4.0, 3.0)
    run_until(lambda: logs('DETOUR'), 80)
    x_far = max(abs(x) for t, x, y in W.trail)
    check('13. detour through the next aisle refused (DETOUR), robot did not go there',
          bool(logs('DETOUR')) and x_far < 1.0, (logs('DETOUR')[:1], round(x_far, 2)))
    run_until(lambda: n.deferred or n.gi >= 2, 20)
    t_def = W.t
    check('13. the rest of the lane put off', bool(logs('DEFER')) and 'out of the aisle' in ' '.join(logs('DEFER')),
          logs('DEFER')[:1])
    W.blocker, W.via_far = None, None
    run_until(lambda: n._done, 300)
    rv = [k for k in follows() if abs(k[2][0]) < 1e-6 and k[1] > t_def + 5]
    check('13. revisited before home, driven along the lane (FollowPath on x=0)',
          bool(logs('REVISIT')) and bool(rv), (logs('REVISIT')[:1], rv[:2]))
    check('13. aisle_1 cleaned in the end', n.cleaned == ['aisle_1'], (n.cleaned, n.swept_pct))

    # 14. cleaned = painted lanes: the aisle has 3 lanes, the job drove 2 (as Nav2 "reaching"
    # lane ends did in shift_v104a): not cleaned without the patch pass; with it, the third
    # lane is swept before home and it is
    geo3 = {'aisle_1': ([0.0, 0.5, 1.0], 0.0, 8.0, -0.3, 1.3)}
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2, geom=geo3, patch=False)
    run_until(lambda: n._done, 200)
    check('14. 2 of 3 lanes swept, no patch pass: NOT cleaned (was "cleaned" in v10.4)',
          n._done and n.cleaned == [] and 60 <= n.swept_pct['aisle_1'] < 90, (n.cleaned, n.swept_pct))
    check('14. ... and the summary says how much was swept', any('lanes swept' in x for x in logs('lanes swept')),
          logs('lanes swept')[:1])
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2, geom=geo3, patch=True)
    run_until(lambda: n._done, 300)
    p3 = [k for k in follows() if abs(k[2][0] - 1.0) < 1e-6]
    check('14. with the patch pass: lane x=1.0 swept before home, aisle cleaned',
          bool(p3) and n.cleaned == ['aisle_1'] and n.swept_pct['aisle_1'] >= 90, (p3[:1], n.cleaned, n.swept_pct))

    # 15. stale results are ignored; a stale accepted goal is cancelled
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    run_until(lambda: W.y > 2.0, 60)
    gi0, sid0 = n.gi, n._send_id
    n.goal_done(Fut(P(status=6)), n._epoch, sid0 - 1)
    check('15. an old result (stale send id) changes nothing', n.gi == gi0 and not logs('retry 1')
          and not logs('in traffic'), (n.gi, gi0))
    old = GoalHandle(9.0, 9.0)
    n.goal_response(Fut(old), n._epoch, sid0 - 1)
    run_until(lambda: False, 0.5)
    check('15. a stale goal accepted late is cancelled', old.cancels == 1, old.cancels)
    run_until(lambda: n._done, 200)

    # 16. a vehicle standing in the lane is never gone round: the patience rules put it off
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    GUARD['on'] = True
    run_until(lambda: W.y > 2.0, 60)
    GUARD.update(state='go', vehicle_m=2.0, say='clear')
    W.hold = True
    run_until(lambda: n.deferred, 120)
    check('16. vehicle in the lane: no BYPASS, put off after the patience', not logs('BYPASS')
          and bool(n.deferred), (logs('BYPASS')[:1], [d['label'] for d in n.deferred]))
    GUARD.update(vehicle_m=None)
    W.hold = False
    run_until(lambda: n._done, 300)
    check('16. ... and swept on the revisit', n.cleaned == ['aisle_1'], (n.cleaned, n.swept_pct))

    # 19. follow_lanes switched off at run time (ros2 param set): the next lane goes to Nav2
    n = fresh(m, L2, Z2, lanes=True, sweeps=SW2)
    run_until(lambda: len(follows()) >= 1, 30)
    r = W.param_cb([P(name='follow_lanes', value=False)])
    run_until(lambda: n._done, 200)
    check('19. follow_lanes false at run time: accepted, the next lane is a NavigateToPose goal',
          r.successful and not n.LANES_FOLLOW and len(follows()) == 1
          and any(k[0] == 'nav' and abs(k[3][0] - 0.5) < 1e-6 and abs(k[3][1]) < 1e-6 for k in W.kinds)
          and n._done, (r, W.kinds[-3:]))

    v106(m, L2, Z2, SW2)

    # 17. build_mission tags every lane (real geometry): 5 lanes x 2 aisles, end -> start
    n = fresh(m, L2, Z2)
    goals, zl = n.build_mission(['aisle_1', 'aisle_2'])
    sw = n._last_sweeps
    pairs_ok = all(abs(a[0] - b[0]) < 1e-6 and abs(abs(a[1] - b[1]) - 12.8) < 0.01 for (b, a) in
                   ((k[:2], v) for k, v in sw.items()))
    idx = {g: i for i, g in enumerate(goals)}
    before = all(i > 0 and math.hypot(goals[i - 1][0] - v[0], goals[i - 1][1] - v[1]) < 0.4
                 for k, v in sw.items() for i in [idx[k]])
    check('17. 10 lanes tagged, each end -> its start (same x, 12.8 m)', len(sw) == 10 and pairs_ok,
          (len(sw), list(sw.items())[:2]))
    check('17. ... each lane start is the goal before its end', before)
    gf, zf = n.build_mission(['front'])
    check('17. front rows tagged too', len(n._last_sweeps) >= 5, len(n._last_sweeps))

    # 18. the detour test is exact in the real aisles (map: aisle 2 is x 2.06..4.96)
    def plan_through(pts):
        out = []
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            k = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
            out += [(x0 + (x1 - x0) * i / k, y0 + (y1 - y0) * i / k) for i in range(k)]
        W.plan_n += 1
        return P(header=P(frame_id='map', stamp=P(sec=1, nanosec=W.plan_n)),
                 poses=[P(pose=P(position=P(x=x, y=y, z=0.0))) for x, y in out + [pts[-1]]])

    def refused(robot, pts):
        n = fresh(m, [(4.54, 13.9, 'wp1'), (4.54, 1.1, 'wp2'), (0.0, 0.0, 'HOME')], ['aisle_2', 'aisle_2', 'home'])
        n.goals, n.gi = [(4.54, 13.9, 'wp1'), (4.54, 1.1, 'wp2'), (0.0, 0.0, 'HOME')], 1
        W.x, W.y = robot
        n._sweep = dict(key=(4.54, 1.1, 'wp2'), gi=1, a=(4.54, 13.9), b=(4.54, 1.1), L=12.8, ux=0.0, uy=-1.0,
                        target=8.0, bypasses=1, phase='enter', entered=0, stand=0.0, stand_go=0.0,
                        ref_s=None, s0=0.0, box=n._aisle_box(4.54))
        n._goal_handle = GoalHandle(4.54, 6.0)
        n.on_plan(plan_through(pts))
        return n._detour_pending
    check('18. round a person by the far rack face of aisle 2 (1.9 m off lane 4.54): allowed',
          not refused((4.54, 9.5), [(4.54, 9.5), (2.6, 8.0), (4.54, 6.0)]))
    check('18. through aisle 3 (x 6.4, only 1.86 m off lane 4.54): refused',
          refused((4.54, 9.5), [(4.54, 9.5), (4.54, 13.9), (6.4, 13.9), (6.4, 6.0), (6.4, 1.1), (4.54, 1.1), (4.54, 6.0)]))
    check('18. out round the rack end and back in: allowed',
          not refused((4.54, 2.5), [(4.54, 2.5), (5.5, 0.9), (4.54, 0.9), (4.54, 1.9)]))
    check('18. from aisle 1 (the robot is there) down, across and up aisle 2: allowed',
          not refused((0.0, 6.0), [(0.0, 6.0), (0.0, 0.9), (4.54, 0.9), (4.54, 6.0)]))
    check('18. from aisle 1 via aisle 3: refused',
          refused((0.0, 6.0), [(0.0, 6.0), (0.0, 0.9), (6.4, 0.9), (6.4, 14.0), (4.54, 14.0), (4.54, 6.0)]))


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
