#!/usr/bin/env python3
"""tour_why.py (v10.1) - why the robot came close to someone during a tour run.

    python3 ~/ros2_ws/src/tools/tour_why.py <label> [gap_m]     (default 0.15 m)
    python3 ~/ros2_ws/src/tools/tour_why.py live [seconds]      (with the stack running)

Joins the three logs of a tour run on simulation time:
  ~/runs/tour/<label>.csv      the robot every 0.1 s, with its body-to-body gap to everyone
  ~/runs/guard/<label>.csv     every decision the guard made, including its vehicle columns
  ~/runs/traffic/<label>.csv   what the traffic was doing (people's scripts, vehicles' odometry)

Finds every moment the robot's body came within gap_m of someone's body (0 = touching),
groups them into events, and prints the seconds around each: where the robot was and how fast,
who it was close to and what they were doing, what the guard decided and why, and - for a
vehicle - whether the guard was still following it as a vehicle at that moment. The last
column is the point of the whole file: a vehicle the guard is not following has no forks.

"people" and "hid" say whether the guard saw anyone there at all and how many rays it took out
of the scan Nav2 reads. A vehicle with people 0 and hid 0 was invisible to the whole stack; one
with hid > 0 was seen, called a crossing person, and hidden from Nav2 on purpose.

The end (v10.1, printed even when nobody came close): for each vehicle, how long it was in sight
within 5 m of the robot, and for how much of that time the guard was following a vehicle. That
says whether the guard keeps losing vehicles it could see (the case for track persistence).

live mode needs the stack running (ros2 + traffic). It compares the lidar against where the
vehicles really are (their odometry) and says whether the lidar sees them at all.
"""
import csv
import math
import os
import sys
import time

RUNS = os.path.expanduser('~/runs')
WINDOW = 3.0            # s shown either side of an event
STEP = 0.4              # s between printed rows
JOIN = 0.2              # s: how far a guard / traffic row may be from the tick it describes


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='', errors='replace') as f:
        return list(csv.DictReader(f))


def fnum(v, default=float('nan')):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


class ByTime:
    """Rows of one log, looked up by the nearest simulation time."""

    def __init__(self, rows, key=lambda r: True):
        self.rows = sorted((fnum(r.get('t')), r) for r in rows if key(r) and fnum(r.get('t')) == fnum(r.get('t')))
        self.times = [t for t, _ in self.rows]

    def at(self, t):
        if not self.rows:
            return None
        import bisect
        i = bisect.bisect_left(self.times, t)
        best = None
        for j in (i - 1, i, i + 1):
            if 0 <= j < len(self.rows):
                if best is None or abs(self.times[j] - t) < abs(self.times[best] - t):
                    best = j
        return self.rows[best][1] if best is not None and abs(self.times[best] - t) <= JOIN else None


def live(seconds=10.0):
    """Does the lidar see the traffic\'s vehicles at all? Compares /scan against where each
    vehicle really is (/model/<name>/odometry). Needs the stack running."""
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import traffic as TR

    TOL = 0.35          # m: a return this close to where the body should be counts as seeing it

    class Look(Node):
        def __init__(self):
            super().__init__('tour_why_live', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.buf = Buffer()
            self.tfl = TransformListener(self.buf, self)
            self.veh, self.scans = {}, 0
            self.tally = {n: dict(seen=0, blocked=0, missed=0, scans=0, hit_scans=0, nearest=9.0,
                                  near_d=9e9, far_d=0.0, z=None)
                          for n in TR.VEHICLES}
            for n in TR.VEHICLES:
                self.create_subscription(Odometry, f'/model/{n}/odometry',
                                         lambda m, n=n: self.on_odom(n, m), 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        def on_odom(self, n, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            p = msg.pose.pose.position
            self.veh[n] = (p.x - 4.0, p.y - 4.0, yaw)
            self.tally[n]['z'] = p.z

        def laser(self):
            for frame in ('lidar_link', 'base_link'):
                try:
                    tf = self.buf.lookup_transform('map', frame, Time())
                except Exception:
                    continue
                q = tf.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
                return tf.transform.translation.x, tf.transform.translation.y, yaw
            return None

        def on_scan(self, scan):
            lp = self.laser()
            if lp is None or not self.veh:
                return
            self.scans += 1
            lx, ly, lyaw = lp
            n_beams = len(scan.ranges)
            for n, (vx, vy, vyaw) in self.veh.items():
                sp = TR.VEHICLES[n]
                c = TR.corners(vx, vy, vyaw, sp['L'], sp['W'])
                pts = []
                for i in range(4):                       # sample the body outline
                    a, b = c[i], c[(i + 1) % 4]
                    for k in range(5):
                        u = k / 5.0
                        pts.append((a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u))
                t = self.tally[n]
                reach = min(scan.range_max, 12.0) - 0.5
                d_body = math.hypot(vx - lx, vy - ly)
                t['near_d'] = min(t['near_d'], d_body)
                t['far_d'] = max(t['far_d'], d_body)
                hits, in_range = 0, 0
                for px, py in pts:
                    d = math.hypot(px - lx, py - ly)
                    if d > reach:
                        continue                        # out of the lidar's reach, not its fault
                    in_range += 1
                    bear = math.atan2(py - ly, px - lx) - lyaw
                    i = int(round((math.atan2(math.sin(bear), math.cos(bear)) - scan.angle_min)
                                  / scan.angle_increment))
                    if not 0 <= i < n_beams:
                        continue
                    r = scan.ranges[i]
                    if r != r or math.isinf(r):
                        t['missed'] += 1                # the beam went off to infinity
                        continue
                    t['nearest'] = min(t['nearest'], abs(r - d))
                    if abs(r - d) <= TOL:
                        t['seen'] += 1
                        hits += 1
                    elif r < d - TOL:
                        t['blocked'] += 1               # something nearer is in the way
                    else:
                        t['missed'] += 1                # the beam goes straight past it
                if in_range:
                    t['scans'] += 1
                if hits >= 3:
                    t['hit_scans'] += 1

    rclpy.init()
    node = Look()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f'=== tour_why live: {node.scans} scans in {seconds:.0f} s')
    if not node.veh:
        print('  no vehicle odometry - is the traffic tab running? (ros2 topic list | grep model/)')
    verdicts = []
    for n, t in node.tally.items():
        if n not in node.veh:
            print(f'  {n}: no odometry, so nothing to compare')
            continue
        z = t['z']
        if z is not None and abs(z) > 0.5:
            print(f'  {n}: its z is {z:.1f} m. It is not on the warehouse floor, so the lidar cannot '
                  f'possibly see it (the models have no collision shape, so nothing holds them up). '
                  f'Everything below is a consequence of that.')
        elif z is not None:
            print(f'  {n}: z {z:.2f} m, so it is on the floor where it belongs.')
        if not t['scans']:
            print(f'  {n}: never within the lidar\'s reach in these {seconds:.0f} s '
                  f'(closest {t["near_d"]:.1f} m). Drive the robot nearer, or sample for longer.')
            continue
        total = max(t['seen'] + t['blocked'] + t['missed'], 1)
        print(f'  {n}: in reach in {t["scans"]} scans ({t["near_d"]:.1f}-{t["far_d"]:.1f} m away); the lidar '
              f'saw its body in {t["hit_scans"]} of those ({100.0 * t["hit_scans"] / t["scans"]:.0f} %). '
              f'Of {total} points along its outline that were in reach, {100.0 * t["seen"] / total:.0f} % '
              f'matched a return, {100.0 * t["blocked"] / total:.0f} % were behind something else, '
              f'{100.0 * t["missed"] / total:.0f} % had the beam go straight past. Closest a return ever '
              f'came to its outline: {t["nearest"]:.2f} m')
        verdicts.append((n, t['hit_scans'], t['scans']))
    for n, hit, sc in verdicts:
        if hit == 0:
            print(f'  VERDICT {n}: the lidar never saw it while it was in reach. Then Nav2 never saw it '
                  f'either, and the robot drives through it. That is the model, not the guard.')
        elif hit < 0.5 * sc:
            print(f'  VERDICT {n}: the lidar saw it in only {100.0 * hit / sc:.0f} % of the scans it was '
                  f'in reach for. Too patchy to track as a vehicle.')
        else:
            print(f'  VERDICT {n}: the lidar does see it. The miss is in the guard, not the model.')
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(__doc__)
        return 1
    if argv[0] == 'live':
        return live(float(argv[1]) if len(argv) > 1 else 10.0)
    label = argv[0]
    near = float(argv[1]) if len(argv) > 1 else 0.15

    ticks = read_csv(os.path.join(RUNS, 'tour', f'{label}.csv'))
    if not ticks:
        print(f'no ~/runs/tour/{label}.csv - was tour_test.py run with this label?')
        return 1
    guard = ByTime(read_csv(os.path.join(RUNS, 'guard', f'{label}.csv')))
    traffic_rows = read_csv(os.path.join(RUNS, 'traffic', f'{label}.csv'))
    agents = [k[4:] for k in ticks[0] if k.startswith('gap_')]
    traffic = {a: ByTime(traffic_rows, key=lambda r, a=a: r.get('name') == a) for a in agents}
    T = ByTime(ticks)

    # ---- every close moment, grouped into events (agent, first t, last t, worst gap, worst t)
    events = []
    for r in ticks:
        t = fnum(r.get('t'))
        for a in agents:
            g = fnum(r.get(f'gap_{a}'))
            if g != g or g > near:
                continue
            last = next((e for e in reversed(events) if e['agent'] == a), None)
            if last is not None and t - last['t1'] <= 2.0:
                last['t1'] = t
                if g < last['worst']:
                    last['worst'], last['worst_t'] = g, t
            else:
                events.append(dict(agent=a, t0=t, t1=t, worst=g, worst_t=t, lap=r.get('lap')))
    if not events:
        print(f'=== tour_why: {label} - nobody came within {near:.2f} m of the robot (body to body)')
        print()
        whole_run(label, ticks, guard, traffic_rows)
        return 0

    print(f'=== tour_why: {label} - {len(events)} times the robot came within {near:.2f} m of someone '
          f'(body to body; 0.00 = touching)')
    print()
    print(f'{"#":>2s} {"lap":>3s} {"agent":10s} {"worst":>6s} {"at":>8s} {"for":>5s}   where')
    for k, e in enumerate(events, 1):
        r = T.at(e['worst_t'])
        print(f'{k:2d} {str(e["lap"]):>3s} {e["agent"]:10s} {e["worst"]:6.2f} {e["worst_t"]:8.1f} '
              f'{e["t1"] - e["t0"] + 0.1:5.1f}s   {r.get("region", "") if r else ""}')
    print()

    for k, e in enumerate(events, 1):
        a = e['agent']
        print(f'--- {k}. {a}, lap {e["lap"]}: {e["worst"]:.2f} m at {e["worst_t"]:.1f} s')
        print(f'{"t":>8s} {"robot x":>8s} {"y":>6s} {"v":>5s} {"lim":>5s} {"gap":>6s} | '
              f'{a + " x":>10s} {"y":>6s} {"v":>5s} {"doing":14s} | {"guard":20s} {"hold":>4s} '
              f'{"people":>6s} {"hid":>4s} | {"vehicles":>8s} {"veh_d":>6s} {"veh_go":>6s} {"keep":>6s} {"wait":>4s}')
        t = e['worst_t'] - WINDOW
        while t <= e['worst_t'] + WINDOW:
            r = T.at(t)
            if r is None:
                t += STEP
                continue
            g = fnum(r.get(f'gap_{a}'))
            gr = guard.at(t)
            tr = traffic[a].at(t)
            lim = fnum(r.get('limit'))
            state = '' if gr is None else (gr.get('state', '') + '/' + gr.get('why', ''))
            veh = ['', '', '', '', '']
            ppl, hid = '', ''
            if gr is not None:
                veh = [gr.get('vehicles', ''), gr.get('veh_d', ''), gr.get('veh_go', ''),
                       gr.get('veh_keep', ''), gr.get('veh_hold', '')]
                ppl, hid = gr.get('people', ''), gr.get('hidden_rays', '')
            s_lim = '     ' if lim != lim else format(lim, '5.2f')
            if tr is None:
                s_ax, s_ay, s_av, s_doing = ' ' * 10, ' ' * 6, ' ' * 5, ' ' * 14
            else:
                s_ax = format(fnum(tr.get('x')), '10.2f')
                s_ay = format(fnum(tr.get('y')), '6.2f')
                s_av = format(abs(fnum(tr.get('v'), 0.0)), '5.2f')
                s_doing = format((tr.get('state') or 'walking')[:14], '14s')
            print(f'{fnum(r.get("t")):8.1f} {fnum(r.get("x")):8.2f} {fnum(r.get("y")):6.2f} '
                  f'{fnum(r.get("v")):5.2f} {s_lim} {g:6.2f} | {s_ax} {s_ay} {s_av} {s_doing} | '
                  f'{state[:20]:20s} {r.get("hold", ""):>4s} {ppl:>6s} {hid:>4s} | '
                  f'{veh[0]:>8s} {veh[1]:>6s} {veh[2]:>6s} {veh[3]:>6s} {veh[4]:>4s}')
            t += STEP
        print()

    whole_run(label, ticks, guard, traffic_rows)
    return 0


def whole_run(label, ticks, guard, traffic_rows):
    """What the guard knew about each vehicle over the whole run."""
    print('--- the guard and the vehicles over the whole run')
    g_rows = [r for r in read_csv(os.path.join(RUNS, 'guard', f'{label}.csv'))]
    if g_rows and 'vehicles' in g_rows[0]:
        seen = sum(1 for r in g_rows if fnum(r.get('vehicles'), 0) > 0)
        hold = sum(1 for r in g_rows if r.get('veh_hold') == '1')
        print(f'  followed at least one vehicle in {seen} of {len(g_rows)} scans ({100.0 * seen / len(g_rows):.0f} %); '
              f'gave way or kept clear in {hold} scans')
    else:
        print('  the guard log has no vehicle columns (guard before v10, or GUARD_VEHICLES=0)')
    forgotten = False
    for name in sorted({r.get('name') for r in traffic_rows if r.get('kind') == 'vehicle'}):
        rows = [r for r in traffic_rows if r.get('name') == name]
        still, longest, run_s = 0, 0.0, 0.0
        for r in rows:
            if abs(fnum(r.get('v'), 0.0)) < 0.05:
                still += 1
                run_s += 0.1
                longest = max(longest, run_s)
            else:
                run_s = 0.0
        held = sum(1 for r in rows if 'held' in (r.get('state') or '')) * 0.1
        forgotten = forgotten or longest > 30.0
        print(f'  {name}: stood still {still * 0.1:.0f} s in all, longest {longest:.0f} s in one go, '
              f'of which {held:.0f} s held up by someone in its way')
    if forgotten:
        print('  A vehicle that has stood still for more than 30 s (VEH_RECENT) goes back to being a plain '
              'obstacle to the guard, and a plain obstacle has no forks: the lidar cannot see them.')
    print()
    for line in sight(label, ticks, guard, traffic_rows):
        print(line)


SIGHT_R = 5.0           # m: a vehicle with any corner this close to the robot, in a straight line clear
                        # of the racks and areas, is "in sight" (VEH_NEAR: the guard's speed cap range)


def _blockers():
    """Map-frame rectangles a straight line of sight cannot pass: the racks and areas lanes.py
    takes from warehouse.sdf (walls left out), shrunk 5 cm so a line along a face still counts."""
    try:
        import lanes
    except Exception:
        return []
    ox, oy = lanes.MAP_OFFSET
    return [(x0 - ox + 0.05, y0 - oy + 0.05, x1 - ox - 0.05, y1 - oy - 0.05)
            for n, (x0, y0, x1, y1) in lanes.OBSTACLES.items() if not n.endswith('_wall')]


def _crosses(ax, ay, bx, by, box):
    """Does the segment a-b pass through the rectangle box (x0, y0, x1, y1)? (Liang-Barsky)"""
    x0, y0, x1, y1 = box
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
        if p == 0:
            if q < 0:
                return False
            continue
        r = q / p
        if p < 0:
            if r > t1:
                return False
            t0 = max(t0, r)
        else:
            if r < t0:
                return False
            t1 = min(t1, r)
    return t0 <= t1


def _dist_to_box(px, py, corners):
    """Distance from (px, py) to a convex quadrilateral (0 inside)."""
    sides = [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    cross = [(bx - ax) * (py - ay) - (by - ay) * (px - ax) for (ax, ay), (bx, by) in sides]
    if all(c >= 0 for c in cross) or all(c <= 0 for c in cross):
        return 0.0
    best = 9e9
    for (ax, ay), (bx, by) in sides:
        dx, dy = bx - ax, by - ay
        u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / max(dx * dx + dy * dy, 1e-12)))
        best = min(best, math.hypot(ax + u * dx - px, ay + u * dy - py))
    return best


def _vehicle_layer(label):
    """'on' / 'off' from the guard tab's start-up line, or None if there is no such log."""
    p = os.path.join(RUNS, 'logs', f'guard_{label}.log')
    if not os.path.exists(p):
        return None
    with open(p, errors='replace') as f:
        for ln in f:
            if 'people_guard: vehicles ' in ln:
                return 'on' if 'vehicles ON' in ln else 'off'
    return None


THIS_TOL = 0.5          # m: the guard's nearest vehicle (veh_d) matches this one's true distance


def sight(label, ticks, guard, traffic_rows):
    """v10.1: for each vehicle, how long it was in sight within SIGHT_R of the robot, and for how
    much of that the guard was following a vehicle (its 'vehicles' column above 0: a vehicle the
    layer acts on - moving, or moved in the last 30 s), and whether the nearest one it followed
    was this one (veh_d within THIS_TOL of the robot's true distance to this vehicle's body; a
    vehicle coming up behind the robot the same way is left out of veh_d on purpose). Low numbers
    with the vehicle moving mean the guard keeps losing the track: what track persistence fixes.
    Ticks with no guard row near them are left out (counted separately)."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import traffic as TR
    except Exception:
        return ['  (no traffic.py next to tour_why.py, so no line-of-sight check)']
    head = (f'--- in sight of the robot (a corner within {SIGHT_R:.0f} m, no rack or area in between), '
            f'and whether the guard was following a vehicle then')
    if not guard.rows:
        return [head, '  no guard log (~/runs/guard/<label>.csv), so not known']
    if 'vehicles' not in guard.rows[0][1] or _vehicle_layer(label) == 'off':
        return [head, '  the vehicle layer was off in this run (novehicles, or a guard before v10): '
                      'nothing to follow, so not applicable']
    boxes = _blockers()
    times = [fnum(r.get('t')) for r in ticks]
    steps = sorted(b - a for a, b in zip(times, times[1:]) if b > a)
    dt = steps[len(steps) // 2] if steps else 0.1
    out = [head]
    for name in sorted({r.get('name') for r in traffic_rows if r.get('kind') == 'vehicle'}):
        sp = TR.VEHICLES.get(name)
        if sp is None:
            continue
        veh = ByTime(traffic_rows, key=lambda r, n=name: r.get('name') == n)
        n_in = n_mov = n_fol = n_fol_mov = n_this_mov = n_nog = 0
        run, worst, worst_t0, prev_t = 0.0, 0.0, None, None
        for r in ticks:
            t, rx, ry = fnum(r.get('t')), fnum(r.get('x')), fnum(r.get('y'))
            if prev_t is not None and t - prev_t > 1.5 * dt:
                run = 0.0                               # a gap in the ticks: start again
            prev_t = t
            v = veh.at(t)
            if v is None or rx != rx or ry != ry:
                run = 0.0
                continue
            vx, vy, vyaw = fnum(v.get('x')), fnum(v.get('y')), fnum(v.get('yaw'), 0.0)
            body = TR.corners(vx, vy, vyaw, sp['L'], sp['W'])
            if not any(math.hypot(px - rx, py - ry) <= SIGHT_R
                       and not any(_crosses(rx, ry, px, py, b) for b in boxes) for px, py in [(vx, vy)] + body):
                run = 0.0
                continue
            g = guard.at(t)
            if g is None:
                n_nog += 1
                run = 0.0
                continue
            on = fnum(g.get('vehicles'), 0) > 0
            vd = fnum(g.get('veh_d'))
            this = on and vd == vd and abs(vd - _dist_to_box(rx, ry, body)) <= THIS_TOL
            moving = abs(fnum(v.get('v'), 0.0)) >= 0.05
            n_in += 1
            n_mov += moving
            n_fol += on
            n_fol_mov += on and moving
            n_this_mov += this and moving
            if on:
                run = 0.0
            else:
                run += dt
                if run > worst:
                    worst, worst_t0 = run, t - run + dt
        if not n_in:
            out.append(f'  {name}: never in sight within {SIGHT_R:.0f} m'
                       + (f' (with a guard row; {n_nog * dt:.1f} s without one)' if n_nog else ''))
            continue
        line = (f'  {name}: in sight {n_in * dt:.0f} s ({n_mov * dt:.0f} s of it moving); the guard was '
                f'following a vehicle for {100.0 * n_fol / n_in:.0f} % of that')
        if n_mov:
            line += (f' ({100.0 * n_fol_mov / n_mov:.0f} % while it moved; the nearest vehicle it followed '
                     f'was this one in {100.0 * n_this_mov / n_mov:.0f} %)')
        if worst_t0 is not None:
            line += f'; longest in sight but not followed {worst:.1f} s, from t {worst_t0:.1f}'
        if n_nog:
            line += f'; {n_nog * dt:.1f} s in sight had no guard row near it (left out)'
        out.append(line)
    if len(out) == 1:
        out.append('  no vehicles in ~/runs/traffic/<label>.csv')
    return out


if __name__ == '__main__':
    sys.exit(main())
