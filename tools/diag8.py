#!/usr/bin/env python3
"""diag8.py - where did the robot go, why did it stand still, and what was the VM doing?

    python3 ~/ros2_ws/src/tools/diag8.py [label]          (default: guard8_eye)
    python3 ~/ros2_ws/src/tools/diag8.py watch <label>    (stack.sh runs this: CPU and memory every 2 s)

Reads only what a run left in ~/runs (nothing is changed or written):
  guard/<label>.csv        robot pose and speed, and the guard's decision, at every scan
  people_trials.csv        the passes (result, duration, Nav2 recoveries)
  traces/<label>.csv       robot + worker while she was within 2.5 m
  nav2_last.log            the Nav2 tab: planner, controller, recoveries, costmaps, Collision Monitor
  eye/<label>.csv          camera frames with their sim AND wall time (puts Nav2's lines on the sim clock)
  eye_last.log             the eye tab (the detector address and the error it got)
and prints:
  1. the route: which part of the warehouse the robot was in and when (aisle 1, the front corridor
     below the racks, aisle 2, the back corridor, ...); the trials only ever use aisle 1 and the
     front corridor, so anything else is a detour Nav2 planned;
  2. every stop of 5 s or more, and who held the robot: the guard (WAIT), the Collision Monitor
     (Nav2 asked for speed, the monitor let none through) or Nav2 itself (asked for nothing
     although the guard said GO), with the distance to the nearest rack;
  3. the guard second by second around the pass that failed (or the longest stop);
  4. Nav2's own lines, counted, and its key events (collision ahead, recoveries, plans, loop
     overruns, dropped scans) on the sim clock;
  5. load: simulation speed minute by minute, camera frames processed, loop overruns;
  6. the detector link: which address the eye tried and the exact error.
"""
import bisect
import csv
import math
import os
import re
import sys
from collections import Counter, OrderedDict

RUNS = os.path.expanduser('~/runs')
VERSION = 'v10.3'

# The warehouse in the map frame (world - (4, 4)), as warehouse.sdf (same names as lanes.py).
OBSTACLES = OrderedDict([
    ('rack_1', (-3.0, 1.5, -2.0, 13.5)), ('rack_2', (1.0, 1.5, 2.0, 13.5)),
    ('rack_3', (5.0, 1.5, 6.0, 13.5)), ('rack_4', (9.0, 1.5, 10.0, 13.5)),
    ('packing_area', (-3.0, -3.5, 0.0, -1.5)), ('pallet_area', (0.5, -3.5, 1.5, -1.5)),
    ('office', (6.0, -3.5, 10.0, -1.5)),
    ('pallet_1', (-3.8, 4.4, -3.0, 5.6)), ('pallet_2', (10.0, 4.4, 10.8, 5.6)),
    ('front_wall', (-4.0, -4.1, 11.0, -3.9)), ('back_wall', (-4.0, 15.9, 11.0, 16.1)),
    ('left_wall', (-4.1, -4.0, -3.9, 16.0)), ('right_wall', (10.9, -4.0, 11.1, 16.0)),
])
NAV2_RADIUS = 0.30          # m, the robot's circle in Nav2's costmaps (robot_radius)
STILL_M = 0.15              # m: moving less than this ...
STILL_S = 5.0               # s: ... for this long is a stop


def fnum(v, default=float('nan')):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='', errors='replace') as f:
        return list(csv.DictReader(f))


def nearest_obstacle(x, y):
    best = (float('inf'), '')
    for name, (x0, y0, x1, y1) in OBSTACLES.items():
        d = math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))
        if d < best[0]:
            best = (d, name)
    return best


def region(x, y):
    if x != x or y != y:
        return 'unknown'
    for name, (x0, y0, x1, y1) in OBSTACLES.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return f'INSIDE {name}'
    if 1.5 <= y <= 13.5:
        if -2.0 < x < 1.0:
            return 'aisle 1'
        if 2.0 < x < 5.0:
            return 'aisle 2'
        if 6.0 < x < 9.0:
            return 'aisle 3'
        if x < -3.0:
            return 'west strip (1 m, pallet_1)'
        if x > 10.0:
            return 'east strip (1 m, pallet_2)'
    if y > 13.5:
        return 'back corridor'
    if y > -1.5:
        return 'front corridor'
    return 'by the packing area'


NORMAL = ('aisle 1', 'front corridor')     # the aisle-1 trials' route, if no goals are known


def expected_regions(rows, nav):
    """v10: the regions the run's own goals route through - lanes.route from where the robot
    was when each goal started to the goal Nav2 logged ('... poses to (x, y)'). A tour through
    both aisles is not a detour; leaving that route is."""
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import lanes
    except ImportError:
        return set(NORMAL)
    exp = set()
    for ln in nav:
        t = ln.get('t', float('nan'))
        if ln['ev'] != 'goal start' or t != t or not rows:
            continue
        nums = NUM.findall(ln['msg'].split(' to ')[-1])
        if len(nums) < 2:
            continue
        gx, gy = float(nums[-2]), float(nums[-1])
        r = min(rows, key=lambda r: abs(r['t'] - t))
        if abs(r['t'] - t) > 5.0:
            continue
        try:
            pts = [(r['x'], r['y'])] + [(p[0], p[1]) for p in lanes.route((r['x'], r['y']), (gx, gy))]
        except Exception:
            continue
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            n = max(int(math.hypot(bx - ax, by - ay) / 0.25), 1)
            for k in range(n + 1):
                exp.add(region(ax + (bx - ax) * k / n, ay + (by - ay) * k / n))
    return exp or set(NORMAL)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- wall clock -> sim clock
PAUSE_GAP = 10.0       # v10.3: s of wall time with nothing logged = paused (asleep), not slow


class Clock:
    """Maps wall time (Nav2's log lines) to sim time, from (wall, sim) pairs: the guard's log
    (v9 has a wall column) or the camera log."""

    def __init__(self, eye_rows):
        pairs = sorted((fnum(r.get('wall')), fnum(r.get('t'))) for r in eye_rows)
        self.pairs = [(w, s) for w, s in pairs if w == w and s == s]
        self.walls = [w for w, _ in self.pairs]

    def ok(self):
        return len(self.pairs) >= 2

    def sim(self, wall):
        if not self.ok():
            return float('nan')
        i = bisect.bisect_left(self.walls, wall)
        i = min(max(i, 1), len(self.pairs) - 1)
        (w0, s0), (w1, s1) = self.pairs[i - 1], self.pairs[i]
        rate = (s1 - s0) / (w1 - w0) if w1 > w0 else 1.0
        if wall < self.walls[0] or wall > self.walls[-1]:
            # outside the eye's time: use the overall rate from the nearest end
            (w0, s0) = self.pairs[0] if wall < self.walls[0] else self.pairs[-1]
            return s0 + (wall - w0) * self.mean_rate()
        return s0 + (wall - w0) * rate

    def mean_rate(self):
        """v10.3: sim s per wall s while it ran: stretches of more than PAUSE_GAP s of wall time with
        nothing logged (the Mac or the VM asleep) are left out (tour_v101t read 0.06x because of a
        2 h sleep in the middle)."""
        ds = dw = 0.0
        for (w0, s0), (w1, s1) in zip(self.pairs, self.pairs[1:]):
            if 0.0 < w1 - w0 <= PAUSE_GAP:
                ds += s1 - s0
                dw += w1 - w0
        return ds / dw if dw > 0 else 1.0

    def pauses(self):
        """[(sim t, wall s)] stretches of more than PAUSE_GAP s of wall time with nothing logged."""
        return [(s0, w1 - w0) for (w0, s0), (w1, _) in zip(self.pairs, self.pairs[1:]) if w1 - w0 > PAUSE_GAP]


# ---------------------------------------------------------------- Nav2 log
LINE = re.compile(r'^(?:\[(?P<proc>[^\]]+)\]\s+)?\[(?P<sev>DEBUG|INFO|WARN|ERROR|FATAL)\]\s+'
                  r'\[(?P<wall>\d{9,11}\.\d+)\]\s+\[(?P<node>[^\]]+)\]:\s?(?P<msg>.*)$')
ANSI = re.compile(r'\x1b\[[0-9;]*m')
NUM = re.compile(r'[-+]?\d+(?:\.\d+)?(?:e[-+]?\d+)?')

EVENTS = [   # (short name, substring) - checked in this order, first match wins
    ('collision ahead (RPP)', 'detected collision ahead'),
    ('controller gave up', 'Controller patience exceeded'),
    ('no progress', 'Failed to make progress'),
    ('goal start', 'Begin navigating'),
    ('goal succeeded', 'Goal succeeded'),
    ('goal canceled', 'Goal canceled'),
    ('goal canceled', 'Goal was canceled'),
    ('goal failed', 'Goal failed'),
    ('spin', 'Running spin'),
    ('backup', 'Running backup'),
    ('wait', 'Running wait'),
    ('recovery blocked', 'Collision Ahead'),
    ('recovery done', 'completed successfully'),
    ('costmap cleared', 'clear entirely'),
    ('costmap cleared', 'Clearing'),
    ('no path', 'failed to create plan'),
    ('no path', 'No valid path'),
    ('no path', 'Failed to create a plan'),
    ('loop overrun', 'missed its desired rate'),
    ('scan dropped (TF/queue)', 'Message Filter dropping'),
    ('TF too old/new', 'extrapolation'),
    ('TF too old/new', 'Extrapolation'),
    ('monitor: stop (guard hold)', 'Robot to stop due to PeopleHold'),
    ('monitor: stop', 'Robot to stop'),
    ('monitor: emergency braking', 'Robot to approach for'),
    ('monitor: slow down', 'Robot to slowdown'),
    ('monitor: back to normal', 'Robot to continue normal operation'),
    ('gate STOP', 'gate STOP'),
    ('new path', 'Passing new path to controller'),
]


def read_nav2(path):
    lines = []
    if not os.path.exists(path):
        return lines
    for raw in open(path, errors='replace'):
        m = LINE.match(ANSI.sub('', raw.rstrip('\n')))
        if not m:
            continue
        msg = m.group('msg')
        ev = next((name for name, pat in EVENTS if pat in msg), None)
        lines.append(dict(wall=float(m.group('wall')), sev=m.group('sev'), node=m.group('node'),
                          msg=msg, ev=ev))
    return lines


# ---------------------------------------------------------------- sections
def route(rows):
    """Contiguous stretches in one region, flicker under 1 s merged away."""
    segs = []
    for r in rows:
        reg = region(r['x'], r['y'])
        if segs and segs[-1][0] == reg:
            segs[-1][2] = r['t']
            segs[-1][4] = (r['x'], r['y'])
        else:
            segs.append([reg, r['t'], r['t'], (r['x'], r['y']), (r['x'], r['y'])])
    merged = []
    for s in segs:
        if merged and s[2] - s[1] < 1.0 and len(segs) > 1:
            merged[-1][2] = s[2]
            merged[-1][4] = s[4]
            continue
        if merged and merged[-1][0] == s[0]:
            merged[-1][2], merged[-1][4] = s[2], s[4]
        else:
            merged.append(list(s))
    return merged


def stops(rows):
    out, i = [], 0
    n = len(rows)
    for j in range(1, n + 1):
        moved = j < n and math.hypot(rows[j]['x'] - rows[i]['x'], rows[j]['y'] - rows[i]['y']) > STILL_M
        if j == n or moved:
            if rows[j - 1]['t'] - rows[i]['t'] >= STILL_S:
                out.append((i, j - 1))
            i = j if j < n else i
    return out


def holder(seg):
    """Who kept the robot still: share of scans with the guard waiting, with the Collision
    Monitor cutting a command, and with Nav2 commanding nothing while the guard said GO."""
    n = len(seg)
    wait = sum(1 for r in seg if r['state'] == 'wait')
    cm_cut = sum(1 for r in seg if r['state'] != 'wait' and r['cm_in'] == r['cm_in']
                 and r['cm_in'] > 0.03 and (r['cm_out'] != r['cm_out'] or r['cm_out'] < 0.3 * r['cm_in']))
    heard = sum(1 for r in seg if r['cm_in'] == r['cm_in'])
    nav2_zero = sum(1 for r in seg if r['state'] != 'wait' and r['cm_in'] == r['cm_in'] and r['cm_in'] <= 0.03)
    return n, wait, cm_cut, nav2_zero, heard


def goal_spans(nav, clock):
    """(start, end, result) of each Nav2 goal on the sim clock, from bt_navigator's lines."""
    spans, cur = [], None
    if not clock.ok():
        return spans
    for ln in nav:
        if ln['ev'] == 'goal start':
            if cur is not None:
                spans.append((cur, ln['t'], 'replaced'))
            cur = ln['t']
        elif ln['ev'] in ('goal succeeded', 'goal canceled', 'goal failed') and cur is not None:
            spans.append((cur, ln['t'], ln['ev'][5:]))
            cur = None
    if cur is not None:
        spans.append((cur, float('inf'), 'still running'))
    return spans


def in_goal(t0, t1, spans):
    """Share of [t0, t1] covered by a Nav2 goal (None if goals are unknown)."""
    if not spans or t1 <= t0:
        return None
    cover = sum(max(0.0, min(t1, b) - max(t0, a)) for a, b, _ in spans)
    return cover / (t1 - t0)


def kept(label, tab):
    """This run's tab log: ~/runs/logs/<tab>_<label>.log (stack.sh v8.2), else <tab>_last.log."""
    own = os.path.join(RUNS, 'logs', f'{tab}_{label}.log')
    return own if os.path.exists(own) else os.path.join(RUNS, f'{tab}_last.log')


def fmt_t(x):
    return f'{x:7.1f}' if x == x else '      ?'


def proc_name(pid):
    """A readable name for a process: the script for python, else the program."""
    try:
        cmd = open(f'/proc/{pid}/cmdline', 'rb').read().split(b'\0')
        cmd = [c.decode(errors='replace') for c in cmd if c]
    except OSError:
        return None
    if not cmd:
        return None
    prog = os.path.basename(cmd[0])
    if prog.startswith('python') and len(cmd) > 1:
        scr = next((os.path.basename(c) for c in cmd[1:] if c.endswith('.py') or '/' in c), cmd[1])
        return scr.replace('.py', '')
    if prog in ('ruby', 'ign') and any('gazebo' in c for c in cmd):
        return 'gazebo (' + ('gui' if any(c in ('-g', 'gui') for c in cmd) else 'server' if '-s' in cmd else 'sim') + ')'
    return prog[:24]


def watch(label, every=2.0):
    """Every 2 s (wall) until stopped: CPU busy (all cores), memory and swap, and the programs
    using the most CPU -> ~/runs/load/<label>.csv. stack.sh starts this; stack.sh stop ends it."""
    import time
    hz = os.sysconf('SC_CLK_TCK')
    out_dir = os.path.join(RUNS, 'load')
    os.makedirs(out_dir, exist_ok=True)
    f = open(os.path.join(out_dir, f'{label}.csv'), 'w', newline='')
    w = csv.writer(f)
    w.writerow(['wall', 'cpu_busy_pct', 'load1', 'mem_avail_mb', 'swap_used_mb', 'top'])

    def cpu_total():
        v = [int(x) for x in open('/proc/stat').readline().split()[1:]]
        return sum(v), v[3] + (v[4] if len(v) > 4 else 0)

    def proc_ticks():
        out = {}
        for d in os.listdir('/proc'):
            if not d.isdigit():
                continue
            try:
                st = open(f'/proc/{d}/stat').read()
                fields = st[st.rindex(')') + 2:].split()
                out[int(d)] = int(fields[11]) + int(fields[12])
            except (OSError, ValueError, IndexError):
                continue
        return out
    names = {}
    tot0, idle0 = cpu_total()
    p0, t0 = proc_ticks(), time.monotonic()
    try:
        while True:
            time.sleep(every)
            tot1, idle1 = cpu_total()
            p1, t1 = proc_ticks(), time.monotonic()
            busy = 100.0 * (1.0 - (idle1 - idle0) / max(tot1 - tot0, 1))
            per = Counter()
            for pid, ticks in p1.items():
                if pid in p0 and ticks > p0[pid]:
                    if pid not in names:
                        names[pid] = proc_name(pid)
                    if names[pid]:
                        per[names[pid]] += 100.0 * (ticks - p0[pid]) / hz / (t1 - t0)
            mem = {}
            for ln in open('/proc/meminfo'):
                k, v = ln.split(':', 1)
                mem[k] = int(v.split()[0]) / 1024.0
            w.writerow([f'{time.time():.1f}', f'{busy:.1f}', open('/proc/loadavg').read().split()[0],
                        f'{mem.get("MemAvailable", float("nan")):.0f}',
                        f'{mem.get("SwapTotal", 0.0) - mem.get("SwapFree", 0.0):.0f}',
                        ';'.join(f'{k}:{v:.0f}' for k, v in per.most_common(8) if v >= 1.0)])
            f.flush()
            tot0, idle0, p0, t0 = tot1, idle1, p1, t1
    except KeyboardInterrupt:
        pass
    finally:
        f.close()


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == 'watch':
        watch(argv[1] if len(argv) > 1 else 'load_last')
        return 0
    label = argv[0] if argv else 'guard8_eye'
    print(f'=== diag8 {VERSION}: {label}')
    raw = read_csv(os.path.join(RUNS, 'guard', f'{label}.csv'))
    rows = []
    for r in raw:
        rows.append(dict(t=fnum(r.get('t')), state=r.get('state', ''), why=r.get('why', ''),
                         x=fnum(r.get('robot_x')), y=fnum(r.get('robot_y')), yaw=fnum(r.get('robot_yaw')),
                         v=fnum(r.get('robot_v')), plan=r.get('plan_used', ''),
                         people=r.get('people', ''), hidden=fnum(r.get('hidden_rays'), 0.0),
                         pid=r.get('p_id', ''), px=fnum(r.get('p_x')), py=fnum(r.get('p_y')),
                         kind=r.get('p_kind', ''), fresh=r.get('fresh', ''), age=fnum(r.get('p_age')),
                         lim=fnum(r.get('v_limit')), cm_in=fnum(r.get('cm_in')), cm_out=fnum(r.get('cm_out')),
                         d_go=fnum(r.get('d_go')), hold=fnum(r.get('s_hold')), inside=r.get('inside', '')))
    rows = [r for r in rows if r['t'] == r['t'] and r['x'] == r['x']]
    rows.sort(key=lambda r: r['t'])
    eye_rows = read_csv(os.path.join(RUNS, 'eye', f'{label}.csv'))
    clock = Clock([r for r in raw if r.get('wall')] or eye_rows)    # guard v9 logs the wall clock
    nav_path = kept(label, 'nav2')
    nav = read_nav2(nav_path)
    for ln in nav:
        ln['t'] = clock.sim(ln['wall'])
    probe = [r for r in read_csv(os.path.join(RUNS, 'people_trials.csv')) if r.get('label') == label]
    traces = read_csv(os.path.join(RUNS, 'traces', f'{label}.csv'))
    if not rows:
        print(f'no guard log ~/runs/guard/{label}.csv: was the guard running with this label?')
        return 1
    print(f'  guard log {len(rows)} scans, sim {rows[0]["t"]:.1f}-{rows[-1]["t"]:.1f} s; Nav2 log '
          f'{len(nav)} lines ({os.path.basename(nav_path)}); camera log {len(eye_rows)} frames'
          + ('' if clock.ok() else ' (no wall clock in the logs: Nav2 lines stay on the wall clock)'))
    print()

    # ---- passes
    first_t = {}
    for tr in traces:
        g = tr.get('goal')
        t = fnum(tr.get('t'))
        if g and t == t:
            lo, hi = first_t.get(g, (t, t))
            first_t[g] = (min(lo, t), max(hi, t))
    bad = None
    if probe:
        print('--- passes (from people_trials.csv; trace = when she was within 2.5 m)')
        for p in probe:
            lo, hi = first_t.get(p.get('goal'), (float('nan'), float('nan')))
            print(f'  pass {p.get("pass", "?"):>2s}  {p.get("result", "?"):9s} {fnum(p.get("duration_s")):6.1f} s  '
                  f'Nav2 recoveries {p.get("recoveries", "?"):>2s}   trace sim {fmt_t(lo)} - {fmt_t(hi)}')
            if bad is None and p.get('result') != 'SUCCEEDED' and lo == lo:
                bad = (p, lo, hi)
        print()

    # ---- 1. route
    expected = expected_regions(rows, nav)
    print('--- 1. where the robot was (its goals route through: ' + ', '.join(sorted(expected)) + ')')
    for reg, t0, t1, a, b in route(rows):
        flag = '' if reg in expected else '   <-- off the route its goals take'
        print(f'  {t0:7.1f} - {t1:7.1f} s  {reg:38s} ({a[0]:5.2f},{a[1]:5.2f}) -> ({b[0]:5.2f},{b[1]:5.2f}){flag}')
    print()

    # ---- 2. stops
    st = stops(rows)
    spans = goal_spans(nav, clock)
    print(f'--- 2. stops of {STILL_S:.0f} s or more (moved < {STILL_M} m) and who held the robot')
    print('  (guard WAIT; monitor = Nav2 asked for speed, the Collision Monitor passed < 30 %;'
          ' Nav2 = asked for nothing while the guard said GO)')
    longest = None
    for i, j in st:
        seg = rows[i:j + 1]
        dur = seg[-1]['t'] - seg[0]['t']
        n, wait, cm_cut, nav2_zero, heard = holder(seg)
        x, y = seg[0]['x'], seg[0]['y']
        d, obst = nearest_obstacle(x, y)
        her = [math.hypot(r['px'] - r['x'], r['py'] - r['y']) for r in seg if r['px'] == r['px']]
        her_s = f'{min(her):.1f}-{max(her):.1f} m' if her else 'nobody tracked'
        share = in_goal(seg[0]['t'], seg[-1]['t'], spans)
        if share is not None and share < 0.5:
            print(f'  {seg[0]["t"]:7.1f} - {seg[-1]["t"]:7.1f} s ({dur:5.1f} s) at ({x:5.2f},{y:5.2f}): '
                  f'no goal active (before the first pass or between passes)')
            continue
        print(f'  {seg[0]["t"]:7.1f} - {seg[-1]["t"]:7.1f} s ({dur:5.1f} s) at ({x:5.2f},{y:5.2f}) heading '
              f'{math.degrees(seg[0]["yaw"]):+4.0f} deg, {region(x, y)}')
        print(f'      nearest: {obst} {d:.2f} m from the centre ({d - NAV2_RADIUS:+.2f} m outside Nav2\'s '
              f'{NAV2_RADIUS:.2f} m circle); her: {her_s}')
        print(f'      held by: guard WAIT {100 * wait / n:3.0f} %, monitor {100 * cm_cut / n:3.0f} %, '
              f'Nav2 {100 * nav2_zero / n:3.0f} %  (Collision Monitor heard in {100 * heard / n:.0f} % of scans)')
        if longest is None or dur > longest[2]:
            longest = (i, j, dur)          # only stops while a goal was active get here
    if not st:
        print('  none')
    print()

    # ---- 3. guard timeline around the failed pass / longest stop
    if bad is not None:
        t_from = bad[1] - 8.0
        t_to = rows[longest[1]]['t'] if longest and rows[longest[1]]['t'] > bad[1] else bad[2] + 30.0
        title = f'pass {bad[0].get("pass")} ({bad[0].get("result")})'
    elif longest is not None:
        t_from, t_to = rows[longest[0]]['t'] - 40.0, rows[longest[1]]['t']
        title = 'the longest stop'
    else:
        t_from = t_to = None
    if t_from is not None:
        t_to = min(t_to, t_from + 150.0)
        print(f'--- 3. the guard around {title}: one line per second, unchanged seconds folded')
        print('      t  robot x     y  head  v   | guard why      d_go  hold | her x     y   kind     '
              'hid fresh | limit cm_in cm_out | where')
        last_key, folded, nxt, last_print = None, 0, t_from, -1e9
        for r in rows:
            if r['t'] < nxt or r['t'] > t_to:
                continue
            nxt = r['t'] + 1.0
            moving = r['v'] > 0.05
            key = (r['state'], r['why'], region(r['x'], r['y']), r['kind'], r['hidden'] > 0, moving,
                   None if moving else (round(r['x'], 1), round(r['y'], 1)))
            # unchanged: fold it (while moving, still one line every 4 s to show the way it went)
            if key == last_key and (not moving or r['t'] - last_print < 3.9):
                folded += 1
                continue
            last_print = r['t']
            if folded:
                print(f'      ... {folded} s the same')
                folded = 0
            last_key = key
            her = (f'{r["px"]:5.2f} {r["py"]:5.2f} {r["kind"][:7]:7s}' if r['px'] == r['px']
                   else '   -     -   -      ')
            lim = f'{r["lim"]:4.2f}' if r['lim'] == r['lim'] else '  - '
            ci = f'{r["cm_in"]:5.2f}' if r['cm_in'] == r['cm_in'] else '   - '
            co = f'{r["cm_out"]:5.2f}' if r['cm_out'] == r['cm_out'] else '   - '
            hold = f'{r["hold"]:4.1f}' if r['hold'] == r['hold'] else '  - '
            print(f'  {r["t"]:7.1f} {r["x"]:5.2f} {r["y"]:5.2f} {math.degrees(r["yaw"]):+4.0f} {r["v"]:4.2f} | '
                  f'{r["state"]:4s} {r["why"][:9]:9s} {r["d_go"]:4.2f} {hold} | {her} {int(r["hidden"]):3d}   '
                  f'{r["fresh"]:1s}   | {lim} {ci} {co} | {region(r["x"], r["y"])[:22]}')
        if folded:
            print(f'      ... {folded} s the same')
        print('  (hid = lidar rays of people taken out of what Nav2 sees; 0 near her = Nav2 sees her)')
        print()

    # ---- 4. Nav2's words
    print('--- 4. Nav2 tab: warnings and errors by kind (numbers replaced by #)')
    cnt = Counter()
    for ln in nav:
        if ln['sev'] in ('WARN', 'ERROR', 'FATAL'):
            cnt[(ln['sev'], ln['node'], NUM.sub('#', ln['msg'])[:90])] += 1
    for (sev, node, msg), c in cnt.most_common(22):
        print(f'  {c:6d}  {sev:5s} {node[:22]:22s} {msg}')
    if not cnt:
        print('  none (or no ~/runs/nav2_last.log)')
    print()
    evs = [ln for ln in nav if ln['ev'] and ln['ev'] != 'new path']
    if evs:
        win = (t_from - 30.0, t_to + 30.0) if t_from is not None and clock.ok() else None
        w0 = nav[0]['wall']
        tt = (lambda ln: ln['t'] if clock.ok() else ln['wall'] - w0)
        evs = [ln for ln in evs if not win or win[0] <= tt(ln) <= win[1]]
        print('--- 4b. Nav2 events on the sim clock' + (f' ({win[0]:.0f}-{win[1]:.0f} s)' if win else
              ' (wall seconds from the first line: no camera log to convert)'))
        # the controller's "collision ahead" comes 20 times a second: bursts, one line each
        bursts = []
        for ln in evs:
            if ln['ev'] != 'collision ahead (RPP)':
                continue
            if bursts and tt(ln) - bursts[-1][1] < 2.0:
                bursts[-1][1] = tt(ln)
                bursts[-1][2] += 1
            else:
                bursts.append([tt(ln), tt(ln), 1])
        for a, b, k in bursts:
            print(f'  {a:7.1f} - {b:7.1f}  RPP "detected collision ahead!" x{k} (the controller refuses to drive)')
        groups = []
        for ln in evs:
            if ln['ev'] == 'collision ahead (RPP)':
                continue
            t = tt(ln)
            if groups and groups[-1][0] == ln['ev'] and t - groups[-1][2] < 2.0:
                groups[-1][2] = t
                groups[-1][3] += 1
            else:
                groups.append([ln['ev'], t, t, 1, ln['msg']])
        if len(groups) > 45:
            kinds = Counter()
            for g in groups:
                kinds[g[0]] += g[3]
            print('  other events (too many to list): ' + ', '.join(f'{k} x{v}' for k, v in kinds.most_common()))
            groups = groups[:20] + [['...', float('nan'), float('nan'), 0, '']] + groups[-20:]
        for ev, a, b, k, msg in groups:
            if ev == '...':
                print('  ...')
                continue
            span = f'{a:7.1f}' + (f' - {b:7.1f}' if k > 1 else '          ')
            print(f'  {span}  {ev:26s} x{k:<4d} {msg[:72]}')
        paths = [ln for ln in nav if ln['ev'] == 'new path']
        if paths and win:
            k = sum(1 for ln in paths if win[0] <= ln['t'] <= win[1])
            print(f'  ("Passing new path to controller": {k} times in this window, {len(paths)} in all)')
        print()

    # ---- 5. load
    print('--- 5. load')
    if clock.ok():
        pairs = clock.pairs
        print(f'  simulation speed overall {clock.mean_rate():.2f} x real time (while running)')
        for st, pw in clock.pauses():
            print(f'  PAUSED at sim t {st:.0f} s for {pw / 60:.0f} min of wall time (the Mac or the VM asleep?)')
        line, w_start = [], pairs[0][0]
        k = 0
        ew = sorted(fnum(r.get('wall')) for r in eye_rows)
        while w_start < pairs[-1][0]:
            seg = [p for p in pairs if w_start <= p[0] < w_start + 60.0]
            if len(seg) >= 2:
                rtf = (seg[-1][1] - seg[0][1]) / (seg[-1][0] - seg[0][0])
                fr = sum(1 for w in ew if w_start <= w < w_start + 60.0) / 60.0
                line.append(f'{k}:{rtf:.2f}x' + (f'/{fr:.1f}fps' if ew else ''))
            w_start += 60.0
            k += 1
        print('  minute by minute (sim speed' + (' / camera frames processed per s' if ew else '') + '): '
              + '  '.join(line))
        st_ = sorted(fnum(r.get('t')) for r in eye_rows)
        gaps = sorted(b - a for a, b in zip(st_, st_[1:]) if b > a)
        if gaps:
            med = gaps[len(gaps) // 2]
            p90 = gaps[int(0.9 * (len(gaps) - 1))]
            print(f'  sim time between processed frames: median {med:.2f} s, 90 % under {p90:.2f} s, '
                  f'longest {gaps[-1]:.1f} s (the camera renders one frame every 0.2 s at 5 Hz, 0.33 s at 3 Hz)')
        sdt = [abs(fnum(r.get('scan_dt'))) for r in eye_rows if fnum(r.get('scan_dt')) == fnum(r.get('scan_dt'))]
        if sdt:
            sdt.sort()
            print(f'  nearest scan to each frame: median {sdt[len(sdt) // 2]:.2f} s apart; '
                  f'{sum(1 for s in sdt if s > 0.25)} of {len(eye_rows)} frames had no scan within 0.25 s')
    load = read_csv(os.path.join(RUNS, 'load', f'{label}.csv'))
    if load:
        busy = [fnum(r.get('cpu_busy_pct')) for r in load]
        avail = [fnum(r.get('mem_avail_mb')) for r in load]
        swap = [fnum(r.get('swap_used_mb')) for r in load]
        per = {}
        for r in load:
            for item in filter(None, (r.get('top') or '').split(';')):
                name, _, pct = item.rpartition(':')
                per.setdefault(name, []).append(fnum(pct, 0.0))
        n = len(load)
        tops = sorted(((sum(v) / n, k) for k, v in per.items()), reverse=True)[:8]
        busy = [b for b in busy if b == b]
        print(f'  VM ({n} samples, every 2 s): CPU busy mean {sum(busy) / max(len(busy), 1):.0f} %, max '
              f'{max(busy, default=float("nan")):.0f} % of all cores; memory available min '
              f'{min(a for a in avail if a == a) if any(a == a for a in avail) else float("nan"):.0f} MB; '
              f'swap used max {max((x for x in swap if x == x), default=0.0):.0f} MB')
        print('  CPU by program (mean % of one core): ' + ', '.join(f'{k} {v:.0f}' for v, k in tops))
    over = Counter()
    for ln in nav:
        if ln['ev'] in ('loop overrun', 'scan dropped (TF/queue)', 'TF too old/new'):
            over[(ln['ev'], ln['node'])] += 1
    if over:
        for (ev, node), c in over.most_common(8):
            print(f'  {c:6d}  {ev} ({node})')
    else:
        print('  no loop overruns or dropped scans in the Nav2 log')
    print()

    # ---- 6. detector link
    print('--- 6. detector link (eye tab)')
    path = kept(label, 'eye')
    if os.path.exists(path):
        lines = open(path, errors='replace').read().splitlines()
        keep = [ln for ln in lines if any(k in ln for k in ('eye v', 'OFFLINE', 'online', 'ERROR', 'no TF'))]
        for ln in keep[:6]:
            print('  ' + ln[:160])
        status = [ln for ln in lines if 'frames/s' in ln]
        if status:
            print('  last status: ' + status[-1][:160])
    else:
        print('  no ~/runs/eye_last.log')
    if eye_rows:
        ok = sum(1 for r in eye_rows if r.get('ok') == '1')
        print(f'  detector answered {ok} of {len(eye_rows)} frames')
    print()

    # ---- verdicts
    print('--- in short')
    off = [s for s in route(rows) if s[0] not in expected]
    if off:
        print('  DETOUR: the robot left the route its goals take: '
              + '; '.join(f'{s[0]} {s[1]:.0f}-{s[2]:.0f} s' for s in off[:4]))
    else:
        print('  route: the robot stayed on the route its goals take (' + ', '.join(sorted(expected)) + ')')
    if longest is not None:
        i, j, dur = longest
        seg = rows[i:j + 1]
        n, wait, cm_cut, nav2_zero, heard = holder(seg)
        x, y = seg[0]['x'], seg[0]['y']
        d, obst = nearest_obstacle(x, y)
        who = max((('the guard (WAIT)', wait), ('the Collision Monitor', cm_cut), ('Nav2 itself', nav2_zero)),
                  key=lambda w: w[1])
        ca = [ln for ln in nav if ln['ev'] == 'collision ahead (RPP)' and clock.ok()
              and seg[0]['t'] - 1.0 <= ln['t'] <= seg[-1]['t'] + 1.0]
        print(f'  LONGEST STOP: {dur:.0f} s at ({x:.2f},{y:.2f}) in {region(x, y)}, {d:.2f} m from {obst}; '
              f'mostly held by {who[0]}' + (f'; RPP "collision ahead" {len(ca)} times meanwhile' if ca else ''))
    if t_from is not None:
        win_rows = [r for r in rows if t_from <= r['t'] <= t_to]
        near = [r for r in win_rows if r['px'] == r['px'] and math.hypot(r['px'] - r['x'], r['py'] - r['y']) < 3.0]
        seen = [r for r in near if r['hidden'] <= 0]
        clears = 0
        for a, b in zip(win_rows, win_rows[1:]):
            if a['state'] == 'wait' and b['state'] == 'go' and b['why'] == 'clear':
                clears += 1
        if near:
            print(f'  WORKER AND NAV2: while she was within 3 m ({len(near) / 10:.0f} s), Nav2 saw her (no rays '
                  f'hidden) in {100 * len(seen) / len(near):.0f} % of the scans; GO "clear" from a stop '
                  f'(path no longer crossing her walkway) {clears} times')
    if clock.ok():
        print(f'  LOAD: simulation ran at {clock.mean_rate():.2f} x real time'
              + (' with the camera on' if eye_rows else '')
              + (f'; paused {len(clock.pauses())} time(s), {sum(p for _, p in clock.pauses()) / 60:.0f} min'
                 if clock.pauses() else ''))
    if eye_rows:
        ok = sum(1 for r in eye_rows if r.get('ok') == '1')
        print(f'  DETECTOR: answered {ok} of {len(eye_rows)} frames')
    return 0


if __name__ == '__main__':
    sys.exit(main())
