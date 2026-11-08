"""coverage_nav2 v10.7 (12 Oct, after shift_v106a): fake-ROS scenarios for what went wrong there.

    python3 test_v107.py <path to coverage_nav2.py>

Uses test_coverage_traffic.py's fake ROS, with two additions:
  - an RPP-like controller: it turns on the spot only beyond 1.80 rad (rotate_to_heading_min_angle)
    and otherwise drives forward on an arc to a carrot 0.4 m along the path; before each step it
    projects the arc 1 s ahead (up to the carrot) and stands still if the 0.30 m circle would touch
    a rack ("collision ahead"); a FollowPath it cannot follow is aborted after failure_tolerance.
  - the robot obeys the node's own /cmd_vel (stack.sh sends it to /cmd_vel_raw: guard_gate, then the
    Collision Monitor) while no goal runs: nothing moves while the guard holds; backing into a rack
    is refused (the Monitor's Brake zone).
A rack (x 1..2, y 1.5..13.5, map frame: rack 2) is in the static map and in /scan_nav.

Scenarios: A start-up (no command: nothing moves, whatever the guard does), B the zone order
(shift_v106a's command-time pose), C the wedge at lane 2.48's start (shift_v106a 490-770 s): got
out of in seconds, D a lane start facing away next to the rack: turned first, never touches it,
E what does not unstick (a person close, open floor, no guard), F a hold or a 'stop' during a
manoeuvre, G nothing behind -> no backing off, H at most UNSTICK_MAX, I /mission_state, the agent
and the narrator.
"""
import importlib.util
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_coverage_traffic as T  # noqa: E402

W, GUARD, P = T.W, T.GUARD, T.P
FAILS = []


def check(name, ok, detail=''):
    print(f'{"ok  " if ok else "FAIL"} {name}{"  (" + str(detail) + ")" if detail != "" else ""}')
    if not ok:
        FAILS.append(name)


RACK = (1.0, 2.0, 1.5, 13.5)                 # rack 2, map frame: x0, x1, y0, y1
R_ROBOT = 0.30
TW = {'t': -1e9, 'vx': 0.0, 'wz': 0.0}       # the last /cmd_vel the node sent


def rack_walls():
    x0, x1, y0, y1 = RACK
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def dist_rack(x, y):
    x0, x1, y0, y1 = RACK
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


def fake_map():
    """OccupancyGrid-like: x -2..7, y -1..15 at 0.05 m; rack 2 occupied (100), the rest free (0)."""
    res, ox, oy = 0.05, -2.0, -1.0
    w, h = int(9.0 / res), int(16.0 / res)
    data = [0] * (w * h)
    x0, x1, y0, y1 = RACK
    for gy in range(h):
        cy = oy + (gy + 0.5) * res
        if not (y0 <= cy <= y1):
            continue
        for gx in range(w):
            cx = ox + (gx + 0.5) * res
            if x0 <= cx <= x1:
                data[gy * w + gx] = 100
    return P(info=P(resolution=res, width=w, height=h, origin=P(position=P(x=ox, y=oy))), data=data)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rpp_motion(g, dt):
    """One controller step for goal g (FollowPath path or NavigateToPose route). True if it moved."""
    if g.path is not None:
        tx, ty = T.carrot(g.path, (W.x, W.y), look=0.4)
    else:
        while len(g.route) > 1 and math.hypot(g.route[0][0] - W.x, g.route[0][1] - W.y) < 0.15:
            g.route.pop(0)
        tx, ty = g.route[0]
    L = max(math.hypot(tx - W.x, ty - W.y), 1e-6)
    e = wrap(math.atan2(ty - W.y, tx - W.x) - W.yaw)
    if abs(e) > 1.80:                                   # rotate_to_heading_min_angle
        W.yaw = wrap(W.yaw + math.copysign(1.0 * dt, e))
        W.rpp_rot = getattr(W, 'rpp_rot', 0) + 1
        return True
    v = W.speed
    kappa = 2.0 * math.sin(e) / L
    # collision check: the arc projected up to 1 s or the carrot, like RPP's isCollisionImminent
    px, py, pth = W.x, W.y, W.yaw
    if dist_rack(px, py) < R_ROBOT:
        return False
    for _ in range(20):
        px += 0.05 * v * math.cos(pth)
        py += 0.05 * v * math.sin(pth)
        pth += 0.05 * v * kappa
        if math.hypot(px - W.x, py - W.y) > L:
            break
        if dist_rack(px, py) < R_ROBOT:
            return False                                # collision ahead: RPP stands
    nx, ny = W.x + v * dt * math.cos(W.yaw), W.y + v * dt * math.sin(W.yaw)
    if g.path is not None and W.blocker is not None:     # RPP will not drive into someone standing
        bx, by = W.blocker
        d0, d1 = math.hypot(bx - W.x, by - W.y), math.hypot(bx - nx, by - ny)
        if d1 < W.block_r and d1 < d0:
            return False
    W.x, W.y = nx, ny
    W.yaw = wrap(W.yaw + v * kappa * dt)
    return True


def step(dt=0.1):
    """test_coverage_traffic.step with the RPP-like controller and /cmd_vel obeyed."""
    W.t = round(W.t + dt, 6)
    g = W.goal
    held = GUARD['on'] and GUARD['state'] == 'wait'      # the Collision Monitor's PeopleHold
    if g is not None and not g.done and not W.hold and W.nav_state == 'up':
        d = math.hypot(g.x - W.x, g.y - W.y)
        if d <= 0.25:
            gg = g
            W.pending.append((W.t, lambda: T.end_goal(gg, 4)))
        elif not held:
            moved = rpp_motion(g, dt)
            if not moved and g.path is not None:
                W.blocked_for += dt
                if W.blocked_for >= W.fail_tol - 1e-9:     # failure_tolerance: FollowPath aborted
                    W.blocked_for = 0.0
                    gg = g
                    W.pending.append((W.t, lambda: T.end_goal(gg, 6)))
            elif moved:
                W.blocked_for = 0.0
    elif (g is None or g.done) and W.t - TW['t'] <= 0.25 and not held:
        nx = W.x + TW['vx'] * dt * math.cos(W.yaw)
        ny = W.y + TW['vx'] * dt * math.sin(W.yaw)
        if TW['vx'] == 0.0 or dist_rack(nx, ny) >= 0.32 or dist_rack(nx, ny) > dist_rack(W.x, W.y):
            W.x, W.y = nx, ny                           # the Brake zone: never closer than 0.32
        W.yaw = wrap(W.yaw + TW['wz'] * dt)
    W.min_gap = min(getattr(W, 'min_gap', 9.0), dist_rack(W.x, W.y) - R_ROBOT)
    W.trail.append((W.t, W.x, W.y))
    W.trail_kind.append('')
    if GUARD['on'] and '/people_guard/state' in W.subs:
        W.subs['/people_guard/state'](T.guard_msg())
    if W.scan_on and GUARD['on'] and '/scan_nav' in W.subs and W.t >= W.scan_next:
        W.scan_next = W.t + 0.1
        W.subs['/scan_nav'](T.scan_msg())
    due = [p for p in W.pending if p[0] <= W.t]
    W.pending = [p for p in W.pending if p[0] > W.t]
    for _, cb in due:
        cb()
    for tm in W.timers:
        if W.t >= tm[1] - 1e-9:
            tm[1] += tm[0]
            tm[2]()


T.step = step                                           # run_until uses it
_pub = T.Pub.publish


def pub(self, m):
    _pub(self, m)
    if self.topic == '/cmd_vel':
        TW.update(t=W.t, vx=m.linear.x, wz=m.angular.z)
        W.twists = getattr(W, 'twists', []) + [(round(W.t, 1), m.linear.x, m.angular.z,
                                                W.goal is not None and not W.goal.done)]


T.Pub.publish = pub
logs = T.logs


def cmd(n, **c):
    n.on_command(P(data=json.dumps(c)))


def lane_case(m, pose, guard=True):
    """lane 2.48 from its bottom end beside rack 2's corner (shift_v106a, aisle 2's first lane)."""
    goals = [(2.48, 1.12, 'wp1'), (2.48, 13.9, 'wp2'), (0.0, 0.0, 'HOME')]
    zones = ['aisle_2', 'aisle_2', 'home']
    sweeps = {(2.48, 13.9, 'wp2'): (2.48, 1.12)}
    W.__init__()
    TW.update(t=-1e9, vx=0.0, wz=0.0)
    W.x, W.y, W.yaw = pose
    W.walls = rack_walls()
    W.twists = []
    W.min_gap = 9.0
    n = T.fresh(m, goals, zones, lanes=True, sweeps=sweeps,
                geom={'aisle_2': ([2.48], 1.12, 13.9, 2.06, 4.96)}, patch=False)
    W.x, W.y, W.yaw = pose                              # fresh() reset the world
    W.walls = rack_walls()
    W.min_gap = 9.0
    n._map = fake_map()
    GUARD['on'] = guard
    return n


def main(path):
    T.install_fakes()
    os.environ['HOME'] = tempfile.mkdtemp()
    m = T.load(path)

    # ---------------------------------------------------------------- A. start-up
    W.__init__()
    GUARD.update(on=True, state='go', person_m=None, vehicle_m=None, say='clear')
    n = m.CoverageNav2()                                # auto_start false (stack.sh)
    T.run_until(lambda: False, 5)
    GUARD['on'] = False                                 # the guard goes quiet for 3 s ...
    T.run_until(lambda: False, 3)
    GUARD['on'] = True                                  # ... and is back (shift_v106a at start-up)
    T.run_until(lambda: False, 10)
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('A. no command yet: the guard going quiet and coming back sends no goal '
          '(v10.6 drove the built-in 65-waypoint mission)', not W.sent and not logs('SAFETY: the people guard is back'),
          (W.sent[:2], logs('SAFETY')[:2]))
    check('A. ... /mission_state: waiting for a job, nothing queued, zone start',
          ms.get('awaiting_command') is True and ms.get('queue') == [] and ms.get('zone') == 'start'
          and ms.get('goal') == 0, {k: ms.get(k) for k in ('awaiting_command', 'queue', 'zone', 'goal')})
    cmd(n, action='resume')
    cmd(n, action='stop')
    cmd(n, action='resume')
    T.run_until(lambda: False, 5)
    check('A. resume / stop / resume before any job: still nothing sent', not W.sent, W.sent[:2])
    cmd(n, action='clean', zones=['aisle_1'])
    T.run_until(lambda: len(W.sent) > 0, 5)
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('A. the first job: goals go out at once', len(W.sent) > 0 and ms.get('awaiting_command') is False
          and 'aisle_1' in (ms.get('queue') or []), (W.sent[:1], ms.get('queue')))
    W.__init__()
    n = m.CoverageNav2()
    cmd(n, action='home')
    T.run_until(lambda: len(W.sent) > 0 or logs('already there'), 5)
    check('A. "go home" before any job: allowed (it is a command)', bool(W.sent) or bool(logs('HOME')),
          W.sent[:1])

    # ---------------------------------------------------------------- B. zone order
    W.__init__()
    n = m.CoverageNav2()
    W.x, W.y = 4.28, -3.20                              # shift_v106a: below aisle 2 when the command came
    g, z = n.build_mission(['aisle_1', 'aisle_2'])
    a2 = [(x, y) for (x, y, _), zz in zip(g, z) if zz == 'aisle_2']
    first_lane = next(((x, y) for (x, y) in a2 if abs(x - 2.48) < 0.01 and abs(y - 13.9) < 0.05), None)
    check('B. command from below aisle 2: aisle 1 from the bottom, then aisle 2 from the TOP '
          '(v10.6: from the bottom, 138 s down aisle 2 first)',
          bool(a2) and a2[0][1] > 13.0 and first_lane is not None and a2.index(first_lane) <= 2,
          a2[:4])
    check('B. ... via the back strip (y 14.8), not through aisle 2',
          any(abs(y - 14.8) < 0.01 for x, y in a2[:2]), a2[:3])
    a1 = [(x, y) for (x, y, _), zz in zip(g, z) if zz == 'aisle_1']
    check('B. ... aisle 1 entered from the front corridor (y 0), bottom first',
          bool(a1) and abs(a1[0][1]) < 0.01 and abs(a1[2][1] - 1.1) < 0.01, a1[:3])
    W.x, W.y = 0.0, 0.0                                 # at the dock
    g, z = n.build_mission(['aisle_1', 'aisle_2'])
    a1 = [(x, y) for (x, y, _), zz in zip(g, z) if zz == 'aisle_1']
    a2 = [(x, y) for (x, y, _), zz in zip(g, z) if zz == 'aisle_2']
    check('B. from the dock: aisle 1 bottom first (straight in), aisle 2 from the top',
          abs(a1[0][1] - 1.1) < 0.01 and a2[0][1] > 13.0, (a1[:2], a2[:2]))
    W.x, W.y = 3.0, 8.0                                 # standing inside aisle 2, half way up
    g, z = n.build_mission(['aisle_2', 'aisle_1'])
    a2 = [(x, y) for (x, y, _), zz in zip(g, z) if zz == 'aisle_2']
    a1 = [(x, y) for (x, y, _), zz in zip(g, z) if zz == 'aisle_1']
    check('B. inside aisle 2 (first zone): straight to its nearer end, no transit; aisle 1 after it '
          'by its end',
          not any(abs(y - 14.8) < 0.01 or abs(y) < 0.01 for x, y in a2[:1]) and bool(a1)
          and (abs(a1[0][1] - 14.8) < 0.01 or abs(a1[0][1]) < 0.01), (a2[:2], a1[:3]))

    # ---------------------------------------------------------------- C. the wedge (shift_v106a)
    wedged = (2.25, 1.20, math.radians(117))
    GUARD.update(on=True, state='go', person_m=1.8, vehicle_m=2.5, say='clear')   # visitor, forklift
    n = lane_case(m, wedged)
    GUARD.update(person_m=1.8, vehicle_m=2.5)
    T.run_until(lambda: W.y > 3.5 or n.deferred, 120)
    t_out = W.t - 100.0
    check('C. wedged at rack 2\'s corner (2.25, 1.20, 117 deg), people 1.8 m and a vehicle 2.5 m off '
          '(traffic in v10.6: 4 x 60 s): unstuck and along the lane within 25 s',
          W.y > 3.5 and t_out < 25.0 and getattr(n, 'unsticks', 0) >= 1 and not n.deferred,
          (round(t_out, 1), getattr(n, 'unsticks', 0), [d['label'] for d in n.deferred], logs('UNSTICK')[:1]))
    tw = W.twists
    check('C. ... it backed off (negative speed) and turned on the spot (angular only)',
          any(v < 0 and w == 0 for _, v, w, _g in tw) and any(v == 0 and w != 0 for _, v, w, _g in tw), tw[:3])
    check('C. ... never closer than 0.03 m to the rack (circle of 0.30 m)', W.min_gap >= 0.03,
          round(W.min_gap, 3))
    check('C. ... no STALL, no RESCUE, no DEFER', not logs('STALL') and not logs('RESCUE') and not logs('DEFER'))
    # the same with v10.6's rules (no unstick): the model reproduces the wedge
    n = lane_case(m, wedged)
    GUARD.update(person_m=1.8, vehicle_m=2.5)
    n.UNSTICK_MAX = 0                                   # v10.6's rules
    n.ALIGN_MAX = 9.0
    T.run_until(lambda: W.y > 3.5 or logs('DEFER wp2'), 130)
    check('C. control: without UNSTICK the model stays wedged there and puts the lane off (as shift_v106a)',
          W.y < 1.6 and bool(logs('DEFER wp2')), (round(W.x, 2), round(W.y, 2), logs('DEFER')[:1]))

    # ---------------------------------------------------------------- D. the lane start, facing away
    start = (2.57, 1.12, math.radians(-112))             # shift_v106a t 475: reached lane 2.48's start
    GUARD.update(on=True, state='go', person_m=None, vehicle_m=None, say='clear')
    n = lane_case(m, start)
    T.run_until(lambda: W.y > 5.0, 90)
    check('D. lane start facing 158 deg off it, 0.69 m from rack 2\'s corner: ALIGN first, then the lane',
          W.y > 5.0 and getattr(n, 'aligns', 0) == 1 and bool(logs('ALIGN: wp2')) and not logs('UNSTICK'),
          (round(W.y, 2), getattr(n, 'aligns', 0), logs('ALIGN')[:2]))
    check('D. ... never closer than 0.10 m to the rack', W.min_gap >= 0.10, round(W.min_gap, 3))
    k = [i for i, kd in enumerate(W.kinds) if kd[0] == 'follow']
    check('D. ... the FollowPath went out after the turn (robot lined up within 0.12 rad)',
          bool(k) and bool(logs('ALIGN: lined up')), logs('ALIGN:')[-1:])
    n = lane_case(m, (4.0, 1.12, math.radians(-112)))   # the same, but in the open (lane 4.0)
    n.goals = [(4.0, 1.12, 'wp1'), (4.0, 13.9, 'wp2'), (0.0, 0.0, 'HOME')]
    n.sweeps = {(4.0, 13.9, 'wp2'): (4.0, 1.12)}
    T.run_until(lambda: W.y > 5.0, 90)
    check('D. facing away from a lane in the open (2 m from any rack): no ALIGN, RPP turns as before',
          W.y > 5.0 and getattr(n, 'aligns', 0) == 0, (round(W.y, 2), getattr(n, 'aligns', 0)))

    # ---------------------------------------------------------------- E. what is not a wedge
    n = lane_case(m, wedged)
    GUARD.update(on=True, state='go', person_m=0.8, vehicle_m=None)   # someone right beside it
    T.run_until(lambda: False, 30)
    check('E. a person 0.8 m away (< 1.0): no UNSTICK (the traffic rules wait)', getattr(n, 'unsticks', 0) == 0
          and not logs('UNSTICK'), logs('UNSTICK')[:1])
    n = lane_case(m, wedged)
    GUARD.update(on=True, state='wait', person_m=1.5, vehicle_m=None, say='waiting for a person to pass')
    T.run_until(lambda: False, 30)
    check('E. the guard holding (WAIT): no UNSTICK', getattr(n, 'unsticks', 0) == 0, logs('UNSTICK')[:1])
    n = lane_case(m, wedged, guard=False)
    GUARD.update(on=False, state='go', person_m=None, vehicle_m=None)
    T.run_until(lambda: False, 40)
    check('E. no guard running: no UNSTICK, no ALIGN, no turning (the old STALL / ladder)',
          getattr(n, 'unsticks', 0) == 0 and getattr(n, 'aligns', 0) == 0 and all(w == 0 for _, v, w, _g in W.twists),
          (getattr(n, 'unsticks', 0), getattr(n, 'aligns', 0), W.twists[:2]))
    # standing in the open with the guard saying GO and nobody near: not a rack: no UNSTICK
    goals = [(4.0, 3.0, 'wp1'), (4.0, 9.0, 'wp2'), (0.0, 0.0, 'HOME')]
    W.__init__()
    n = T.fresh(m, goals, ['aisle_2', 'aisle_2', 'home'], patch=False)
    n._map = fake_map()
    W.walls = rack_walls()
    GUARD.update(on=True, state='go', person_m=None, vehicle_m=None, say='clear')
    W.x, W.y, W.yaw = 4.0, 1.0, math.pi / 2             # in aisle 2's middle, 2 m from rack 2
    T.run_until(lambda: W.y > 2.0, 30)
    W.hold = True                                       # Nav2 stands for some other reason
    T.run_until(lambda: False, 25)
    check('E. standing in the open (no rack within 0.55 m ahead): no UNSTICK; the old STALL fires',
          getattr(n, 'unsticks', 0) == 0 and bool(logs('STALL')), (getattr(n, 'unsticks', 0), logs('STALL')[:1]))
    W.hold = False

    # ---------------------------------------------------------------- F. a hold or a stop mid-manoeuvre
    n = lane_case(m, wedged)
    GUARD.update(on=True, state='go', person_m=1.8, vehicle_m=None, say='clear')
    T.run_until(lambda: getattr(n, '_man', None) is not None and n._man['phase'] in ('back', 'turn'), 30)
    p0 = (W.x, W.y, W.yaw)
    GUARD.update(state='wait', person_m=1.2, say='waiting for a person to pass')
    T.run_until(lambda: False, 8)
    p1 = (W.x, W.y, W.yaw)
    check('F. the guard holds mid-manoeuvre: nothing moves meanwhile, the manoeuvre is not given up',
          math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 0.02 and abs(wrap(p1[2] - p0[2])) < 0.05
          and getattr(n, '_man', None) is not None, (p0, p1, getattr(n, '_man', None) and n._man['phase']))
    GUARD.update(state='go', person_m=1.8, say='clear')
    T.run_until(lambda: W.y > 3.5, 60)
    check('F. ... the hold over: it finishes and drives the lane', W.y > 3.5, round(W.y, 2))
    n = lane_case(m, wedged)
    GUARD.update(on=True, state='go', person_m=1.8, vehicle_m=None, say='clear')
    T.run_until(lambda: getattr(n, '_man', None) is not None and n._man['phase'] in ('back', 'turn'), 30)
    cmd(n, action='stop')
    k0 = len(W.twists)
    T.run_until(lambda: False, 5)
    later = W.twists[k0:]
    check('F. "stop" mid-manoeuvre: zero at once, nothing after it',
          getattr(n, '_man', None) is None and (not later or all(v == 0 and w == 0 for _, v, w, _g in later))
          and k0 > 0 and W.twists[k0 - 1][1:3] == (0.0, 0.0), (later[:3], W.twists[k0 - 1:k0]))
    cmd(n, action='resume')
    T.run_until(lambda: W.y > 3.5, 90)
    check('F. ... "resume": gets unstuck and drives the lane', W.y > 3.5, (round(W.y, 2), logs('UNSTICK')[-1:]))

    # ---------------------------------------------------------------- G. something behind
    n = lane_case(m, wedged)
    W.walls = rack_walls() + [((2.05, 0.85), (2.75, 0.85))]   # a pallet 0.4 m behind the robot
    GUARD.update(on=True, state='go', person_m=1.8, vehicle_m=None, say='clear')
    T.run_until(lambda: getattr(n, 'unsticks', 0) >= 1 and getattr(n, '_man', None) is None, 40)
    check('G. something 0.4 m behind: no backing off ("not backing off: something behind")',
          bool(logs('not backing off: something behind')) and not any(v < 0 for _, v, w, _g in W.twists),
          (logs('UNSTICK:')[:2], [t for t in W.twists if t[1] < 0][:2]))

    # ---------------------------------------------------------------- H. at most UNSTICK_MAX
    n = lane_case(m, wedged)
    W.walls = rack_walls() + [((2.05, 0.85), (2.75, 0.85))]
    n.TURN_CLEAR = 9.0                                  # cannot turn either: stays wedged
    GUARD.update(on=True, state='go', person_m=1.8, vehicle_m=None, say='clear')
    T.run_until(lambda: bool(logs('DEFER wp2')), 150)
    u2 = [x for x in logs('UNSTICK wp2')]
    check('H. still wedged after UNSTICK_MAX (2) on a waypoint: no third there; the usual rules take '
          'over (traffic retries, bypass, put off)',
          len(u2) == 2 and (n.bypasses >= 1 or bool(logs('DEFER wp2'))),
          (len(u2), n.bypasses, logs('DEFER')[:1], logs('BYPASS')[:1]))

    # ---------------------------------------------------------------- I. state, agent, narrator
    n = lane_case(m, wedged)
    GUARD.update(on=True, state='go', person_m=1.8, vehicle_m=None, say='clear')
    T.run_until(lambda: getattr(n, '_man', None) is not None and n._man['kind'] == 'unstick', 30)
    T.run_until(lambda: False, 0.5)
    ms = json.loads(W.pubs['/mission_state'].msgs[-1].data)
    check('I. /mission_state during an unstick: manoeuvre unstick, rescuing, unsticks 1',
          ms.get('manoeuvre') == 'unstick' and ms.get('rescuing') is True and ms.get('unsticks') == 1,
          {k: ms.get(k) for k in ('manoeuvre', 'rescuing', 'unsticks', 'aligns')})
    narr = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '..', 'narrator.py')
    if os.path.exists(narr):
        spec = importlib.util.spec_from_file_location('narrator_v107', narr)
        N = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(N)
        nar = N.Narrator()
        a = nar.feed(dict(ms, unsticks=0), 1.0)
        b = nar.feed(ms, 2.0)
        check('I. narrator: one line when it gets itself unstuck', any('stuck' in x for x in b)
              and not any('stuck' in x for x in a), b)
        nar = N.Narrator()
        idle = {'goal': 0, 'total': 0, 'zone': 'start', 'queue': [], 'paused': False, 'done': False,
                'awaiting_command': True, 'coverage_pct': 0.0}
        check('I. narrator: nothing about a job before the first command', not nar.feed(idle, 1.0),
              nar.feed(idle, 2.0))
    try:                                                # the package's robot_agent (as installed)
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(path))))
        A = importlib.import_module('warehouse_planner.robot_agent')
        check('I. agent "why did you stop" during an unstick: says it is freeing itself from a rack',
              'rack' in (A.explain_stop(ms) or ''), A.explain_stop(ms))
        check('I. agent before any job: waiting for one', 'job' in (A.explain_stop(
            {'awaiting_command': True, 'guard': True}) or ''))
    except ImportError as e:
        print(f'skip the agent checks (no {e.name} here)')

    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
