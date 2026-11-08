#!/usr/bin/env python3
"""rewind.py (v10.14) - every time the robot stops for a person, the future it avoided.

When the people guard switches to WAIT for a person (yield, start, emergency, touch; not for a vehicle),
Rewind keeps where the robot was, how fast it was going and the plan it was following. For the next 4 s it
watches where everyone really went: the simulator's truth when there is one (the actors are scripted and
never react to the robot, so that is exactly where they would have gone), and, always, the people the robot
itself was tracking (on a real robot that is all there is). Then it drives the robot on in its imagination
with the guard's own model of going on (people_guard's NOMINAL profile, gentle near people as the guard's
is, along the plan: what the guard's d_go is made of) and measures the closest it would have come to anyone,
body to body (the robot's 0.6 x 0.4 m, a person's 0.25 m; 0.00 = touching):

    hit      0.00 m          would have hit: the stop prevented a collision
    near     up to 0.20 m    within 20 cm: inside the guard's SAFE distance, a near miss prevented
    tight    up to 0.50 m    legal, uncomfortable
    clear    beyond          over-cautious, in hindsight

The only thing replaced is the guard's guess of where people would go: by where they really went.

Live: Rewind.step() on every tick (robot_web). Replay: from_record() from the recorded guard log, i.e. what
the robot really did that day. The page asks for the list (listing()) and one stop's whole story (detail())
to act it out in 3D.
"""
import csv
import math

import numpy as np

VERSION = 'v10.14'
HORIZON = 4.0          # s: how far it imagines going on (the guard looks HORIZON = 4 s ahead too)
MERGE = 1.0            # s: a WAIT this soon after the last one is the same stop
PEOPLE_WAIT = ('yield', 'start', 'emergency', 'touch')
CLASSES = (('hit', 0.0, 'would have hit'), ('near', 0.20, 'within 20 cm'), ('tight', 0.50, 'tight'),
           ('clear', 1e9, 'clear'))
CRUISE = 0.8           # m/s: FollowPath's desired_linear_vel on this robot (the guard's nominal speed)
NEAR_R = 3.0           # m: someone this close and the guard models the robot setting off gently
GAP = 0.6              # s: a person not seen for longer than this is not joined up across the gap
KEEP = 400             # stops kept
NAMES = {'worker': 'the worker', 'visitor': 'the visitor', 'stepper': 'the stepper', 'picker_a': 'picker A',
         'picker_b': 'picker B', 'picker_c': 'picker C'}


def classify(gap):
    for key, top, _ in CLASSES:
        if gap <= top + 1e-9:
            return key
    return CLASSES[-1][0]


def label(cls):
    return next((t for k, _, t in CLASSES if k == cls), cls)


def who_name(n):
    if n is None:
        return 'someone'
    if isinstance(n, str) and not n.startswith('p') and not n.startswith('s'):
        return NAMES.get(n, n.replace('_', ' '))
    if isinstance(n, str) and n in NAMES:
        return NAMES[n]
    return 'the person I was tracking'


def people_from_shapes(shapes):
    """The page's shapes (people_guard's markers) -> [(id, x, y)] of the people the robot is tracking now
    (lost ones, drawn grey, are left out: nobody is seeing them)."""
    out = []
    for s in shapes or []:
        if s.get('k') != 'c':
            continue
        if s.get('ns') == 'person':
            c = s.get('rgba') or [1, 1, 1, 1]
            if abs(c[0] - c[1]) < 0.05 and abs(c[1] - c[2]) < 0.05 and 0.45 < c[0] < 0.75:
                continue
            out.append(('p%d' % s['id'], float(s['x']), float(s['y'])))
        elif s.get('ns') == 'standing':
            out.append(('s%d' % s['id'], float(s['x']), float(s['y'])))
    return out


class Tracks:
    """Where people were, frame by frame: name -> [(t, x, y)]. at(t) joins up between frames, never
    beyond what was seen, and not across a gap longer than GAP s."""

    def __init__(self):
        self.d = {}
        self.arr = None

    def add(self, t, people):
        for n, x, y in people:
            self.d.setdefault(n, []).append((float(t), float(x), float(y)))
        self.arr = None

    def empty(self):
        return not self.d

    def at(self, t):
        if self.arr is None:
            self.arr = {n: np.array(v) for n, v in self.d.items()}
        out = []
        for n, a in self.arr.items():
            i = int(np.searchsorted(a[:, 0], t))
            if i < len(a) and abs(a[i, 0] - t) < 1e-6:
                out.append((n, float(a[i, 1]), float(a[i, 2])))
            elif 0 < i < len(a) and a[i, 0] - a[i - 1, 0] <= GAP:
                u = (t - a[i - 1, 0]) / (a[i, 0] - a[i - 1, 0])
                out.append((n, float(a[i - 1, 1] + u * (a[i, 1] - a[i - 1, 1])),
                            float(a[i - 1, 2] + u * (a[i, 2] - a[i - 1, 2]))))
            elif i == len(a) and len(a) and t - a[-1, 0] <= 0.15:
                out.append((n, float(a[-1, 1]), float(a[-1, 2])))
            elif i == 0 and len(a) and a[0, 0] - t <= 0.15:
                out.append((n, float(a[0, 1]), float(a[0, 2])))
        return out

    def paths(self, t0, t1, step=0.1):
        """name -> [(dt, x, y)] sampled every step s (for the 3D replay)."""
        out = {}
        for k in range(int(round((t1 - t0) / step)) + 1):
            t = t0 + k * step
            for n, x, y in self.at(t):
                out.setdefault(str(n), []).append((round(k * step, 2), round(x, 3), round(y, 3)))
        return out


def avoided(pg, x, y, yaw, v, plan, t0, tracks, soft=True, cruise=CRUISE, horizon=HORIZON):
    """'If I had not stopped': the robot along its plan, by the guard's own model of going on, against
    where people really went (tracks). -> dict(gap, name, at, v_at, x, y, who_xy, path)"""
    n = int(round(horizon / pg.DT)) + 1
    k, a, delay, step = pg.soft_profiles()[2] if soft else pg.NOMINAL
    s_line, x_line, y_line = pg.path_ahead((x, y), yaw, plan)
    s = pg.travel(v, k * cruise, a, n, delay=delay, step=step)
    px, py = np.interp(s, s_line, x_line), np.interp(s, s_line, y_line)
    ahead = np.minimum(s + 0.05, s_line[-1])
    hx, hy = np.interp(ahead, s_line, x_line) - px, np.interp(ahead, s_line, y_line) - py
    yaws = np.where(np.hypot(hx, hy) > 1e-6, np.arctan2(hy, hx), yaw)
    vel = np.gradient(s, pg.DT) if n > 1 else np.zeros(n)
    best = {'gap': 9.0, 'name': None, 'at': None, 'v_at': 0.0, 'x': None, 'y': None, 'who_xy': None}
    for i in range(n):
        for name, qx, qy in tracks.at(t0 + i * pg.DT):
            g, _ = pg.body_gap((float(px[i]), float(py[i])), float(yaws[i]), qx, qy)
            if g < best['gap'] - 1e-9:
                best.update(gap=g, name=name, at=round(i * pg.DT, 2), v_at=round(float(vel[i]), 2),
                            x=round(float(px[i]), 3), y=round(float(py[i]), 3), who_xy=(round(qx, 3), round(qy, 3)))
    best['gap'] = round(max(best['gap'], 0.0), 3)
    best['path'] = [(round(i * pg.DT, 2), round(float(px[i]), 3), round(float(py[i]), 3), round(float(yaws[i]), 3),
                     round(float(vel[i]), 2)) for i in range(n)]
    return best


def real_gap(pg, poses, tracks):
    """poses [(t, x, y, yaw)] of the real robot -> (smallest body gap to anyone, who, how fast the robot was
    moving towards them: its fastest towards anyone it was touching, or towards the nearest at the closest moment)."""
    def towards(i, qx, qy):
        a, b = poses[max(0, i - 2)], poses[min(len(poses) - 1, i + 2)]
        if b[0] <= a[0]:
            return 0.0
        vx, vy = (b[1] - a[1]) / (b[0] - a[0]), (b[2] - a[2]) / (b[0] - a[0])
        dx, dy = qx - poses[i][1], qy - poses[i][2]
        d = math.hypot(dx, dy)
        return (vx * dx + vy * dy) / d if d > 1e-6 else math.hypot(vx, vy)
    best, touch_v = (9.0, None, None, 0.0, 0.0), None
    for i, (t, x, y, yaw) in enumerate(poses):
        for name, qx, qy in tracks.at(t):
            g, _ = pg.body_gap((x, y), yaw, qx, qy)
            if g < best[0]:
                best = (g, name, i, qx, qy)
            if g <= 0.0:
                touch_v = max(touch_v if touch_v is not None else -9.0, towards(i, qx, qy))
    if touch_v is not None:
        v = touch_v
    elif best[2] is not None and len(poses) > 1:
        v = towards(best[2], best[3], best[4])
    else:
        v = 0.0
    return round(max(best[0], 0.0), 3), best[1], round(max(v, 0.0), 2)


def story(st):
    """One stop in a sentence, first person."""
    if st.get('cls') is None:
        return 'I am still watching what happens next.'
    who = who_name(st.get('who'))
    if st['cls'] == 'hit':
        return (f"If I had carried on, I would have hit {who} {st['at']:.1f} seconds later, "
                f"at {st['v_at']:.1f} metres a second.")
    if st['cls'] == 'near':
        return f"If I had carried on, I would have passed {max(1, round(st['gap'] * 100))} centimetres from {who}."
    if st['cls'] == 'tight':
        return f"If I had carried on, I would have passed {st['gap']:.1f} metres from {who}: allowed, but too close."
    return (f"Looking back I didn't need that one: going on, I'd have kept {min(st['gap'], 9.0):.1f} metres "
            f"from everyone.")


class Rewind:
    """The stops of one shift and the futures they avoided."""

    def __init__(self, pg, horizon=HORIZON, cruise=CRUISE):
        self.pg, self.horizon, self.cruise = pg, horizon, cruise
        self.reset()

    def reset(self):
        self.stops = []                 # every stop, oldest first (dicts; 'cls' None until its 4 s are up)
        self.pending = []               # stops still watching what happens
        self.cur = None                 # the stop of the WAIT going on now
        self.last_wait = -1e9
        self.t = None
        self.n = 0
        self.v = 0                      # bumped when the list changes
        self.events = []                # finished stops, for the caller to announce

    # -------- live: one tick
    def step(self, t, robot, v, plan, guard, truth=None, own=None):
        """robot (x, y, yaw); v m/s; plan [(x, y)] or None; guard: /people_guard/state (dict) or None;
        truth: the simulator's agents (dicts with name, kind, x, y) or None; own: [(id, x, y)] tracked now."""
        if t is None or robot is None:
            return
        if self.t is not None and t < self.t - 5.0:            # time went back (a replay restarted)
            self.reset()
        self.t = t
        g = guard or {}
        waiting = g.get('state') == 'wait' and g.get('why') in PEOPLE_WAIT
        if waiting:
            if self.cur is None:
                last = self.stops[-1] if self.stops else None
                if last is not None and t - self.last_wait <= MERGE:
                    self.cur = last                              # the same stop, after a blink of GO
                else:
                    pm = g.get('person_m')
                    soft = g.get('v_limit') is not None or (pm is not None and pm < NEAR_R)
                    self.cur = self.new_stop(t, robot, v, plan, g.get('why'), g.get('say') or '', soft)
            self.cur['waited'] = round(t - self.cur['t'], 1)
            self.last_wait = t
        else:
            self.cur = None
        if self.pending:
            people = None if truth is None else [(a.get('name'), float(a['x']), float(a['y'])) for a in truth
                                                 if a.get('kind', 'person') == 'person' and a.get('x') is not None]
            for st in self.pending:
                w = st['_w']
                if people is not None:
                    w['truth'].add(t, people)
                if own is not None:
                    w['own'].add(t, own)
                w['real'].append((t, float(robot[0]), float(robot[1]), float(robot[2])))
            for st in [s for s in self.pending if t >= s['t'] + self.horizon - 1e-6]:
                self.finish(st)
                self.pending.remove(st)

    def new_stop(self, t, robot, v, plan, why, say, soft=True):
        self.n += 1
        st = {'id': self.n, 't': round(float(t), 2), 'x': round(float(robot[0]), 3), 'y': round(float(robot[1]), 3),
              'yaw': round(float(robot[2]), 3), 'v': round(float(v), 2), 'why': why, 'say': say, 'soft': bool(soft),
              'waited': 0.0, 'cls': None,
              '_w': {'plan': None if plan is None else [tuple(p) for p in plan], 'truth': Tracks(), 'own': Tracks(),
                     'real': []}}
        self.stops.append(st)
        self.pending.append(st)
        if len(self.stops) > KEEP:
            self.stops = self.stops[-KEEP:]
        self.v += 1
        return st

    def finish(self, st):
        """The 4 s are up: the verdict. Truth when there is any, else its own tracks."""
        w = st['_w']
        pg = self.pg
        t0 = st['t']
        res = {}
        for src in ('truth', 'own'):
            tr = w[src]
            if tr.empty():
                continue
            f = avoided(pg, st['x'], st['y'], st['yaw'], st['v'], w['plan'], t0, tr, st['soft'], self.cruise,
                        self.horizon)
            f['real_gap'], f['real_who'], f['real_v'] = real_gap(pg, w['real'], tr)
            f['people'] = tr.paths(t0, t0 + self.horizon)
            res[src] = f
        src = 'truth' if 'truth' in res else 'own' if 'own' in res else None
        if src is None:                                          # nobody seen at all: nothing to hit
            f = avoided(pg, st['x'], st['y'], st['yaw'], st['v'], w['plan'], t0, Tracks(), st['soft'],
                        self.cruise, self.horizon)
            f['real_gap'], f['real_who'], f['real_v'], f['people'] = 9.0, None, 0.0, {}
            res['own'], src = f, 'own'
        f = res[src]
        st.update(src=src, gap=f['gap'], cls=classify(f['gap']), who=f['name'], at=f['at'], v_at=f['v_at'],
                  hit_xy=None if f['x'] is None else (f['x'], f['y']), who_xy=f['who_xy'],
                  real_gap=f['real_gap'], real_who=f['real_who'], real_v=f['real_v'])
        if 'own' in res and src == 'truth':
            st['own_gap'], st['own_cls'] = res['own']['gap'], classify(res['own']['gap'])
        st['text'] = story(st)
        st['_d'] = {'path': f['path'], 'people': f['people'],
                    'own': res['own']['people'] if 'own' in res and src == 'truth' else None,
                    'real': [(round(t - t0, 2), round(x, 3), round(y, 3), round(yaw, 3)) for t, x, y, yaw in w['real']]}
        del st['_w']
        self.v += 1
        self.events.append(st)

    def take_events(self):
        ev, self.events = self.events, []
        return ev

    # -------- replay: the whole recorded shift at once
    def from_record(self, guard_csv, plan_at, truth_ts, truth_frames, world_worker, rec_t, rec_rb):
        """The recorded guard log (what the robot really did) -> every stop, finished, against the truth
        (the 10 Hz traffic log and the world's worker). Times are the recording's sim times."""
        self.reset()
        tr = Tracks()
        for t, fr in zip(truth_ts, truth_frames):
            ppl = [(a['name'], a['x'], a['y']) for a in fr if a.get('kind') == 'person']
            w = world_worker(float(t))
            ppl.append((w['name'], w['x'], w['y']))
            tr.add(float(t), ppl)
        if tr.empty():
            return 0
        t_end = float(truth_ts[-1])
        rows = []
        try:
            rows = list(csv.DictReader(open(guard_csv)))
        except OSError:
            return 0
        rec_t = np.asarray(rec_t, dtype=float)

        def num(r, k, d=None):
            try:
                return float(r[k])
            except (KeyError, TypeError, ValueError):
                return d
        cur, last_wait = None, -1e9
        for r in rows:
            t = num(r, 't')
            if t is None:
                continue
            if r.get('state') == 'wait' and r.get('why') in PEOPLE_WAIT:
                if cur is not None and t - last_wait <= MERGE:
                    cur['waited'] = round(t - cur['t'], 1)
                elif t + self.horizon <= t_end:
                    vl, d_go = num(r, 'v_limit'), num(r, 'd_go', 9.0)
                    px, py = num(r, 'p_x'), num(r, 'p_y')
                    rx, ry = num(r, 'robot_x', 0.0), num(r, 'robot_y', 0.0)
                    pm = None if px is None or py is None else math.hypot(px - rx, py - ry)
                    soft = vl is not None or (pm is not None and pm < NEAR_R)
                    say = (f'waiting for a person to pass (going now would pass {d_go:.1f} m from them)'
                           if d_go < 8.9 else 'waiting for a person')
                    cur = self.new_stop(t, (rx, ry, num(r, 'robot_yaw', 0.0)), max(num(r, 'robot_v', 0.0), 0.0),
                                        plan_at(t), r.get('why'), say, soft)
                    cur['pred'] = round(d_go, 3)              # what the guard predicted (centre to centre)
                    cur['_w']['truth'] = tr
                    i0, i1 = np.searchsorted(rec_t, t), np.searchsorted(rec_t, t + self.horizon, side='right')
                    cur['_w']['real'] = [(float(rec_t[i]), float(rec_rb[i][0]), float(rec_rb[i][1]), float(rec_rb[i][2]))
                                         for i in range(i0, i1)]
                    self.pending.remove(cur)
                    self.finish(cur)
                else:
                    cur = None
                last_wait = t
            else:
                if cur is not None and t - last_wait > MERGE:
                    cur = None
        self.events = []
        return len(self.stops)

    # -------- what the page reads
    def visible(self, now=None):
        """Finished stops (in a replay, only those already played: no spoilers)."""
        return [s for s in self.stops if s.get('cls') is not None
                and (now is None or s['t'] + self.horizon <= now + 1e-6)]

    def summary(self, now=None):
        vis = self.visible(now)
        out = {'n': len(vis), 'v': self.v, 'pending': sum(1 for s in self.stops if s.get('cls') is None)}
        for key, _, _ in CLASSES:
            k = [s for s in vis if s['cls'] == key]
            out[key] = len(k)
            out[key + '_s'] = round(sum(s['waited'] for s in k), 1)
        if vis:
            c = self.closest(now)
            out['closest'] = None if c is None else {'id': c['id'], 'cls': c['cls'], 'text': c['text']}
            last = vis[-1]
            out['last'] = {'id': last['id'], 'cls': last['cls'], 'text': last['text'], 't': last['t']}
        return out

    def closest(self, now=None):
        vis = self.visible(now)
        if not vis:
            return None
        return min(vis, key=lambda s: (s['gap'], -(s.get('v_at') or 0.0), s.get('at') or 9.0))

    def listing(self, now=None):
        keys = ('id', 't', 'x', 'y', 'why', 'waited', 'cls', 'gap', 'who', 'at', 'v_at', 'real_gap', 'real_v', 'src',
                'own_cls', 'text')
        return [{k: s.get(k) for k in keys} for s in self.visible(now)]

    def detail(self, i, now=None):
        s = next((s for s in self.visible(now) if s['id'] == i), None)
        if s is None:
            return None
        out = {k: v for k, v in s.items() if not k.startswith('_')}
        out.update(s.get('_d') or {})
        out['horizon'] = self.horizon
        out['who_name'] = who_name(s.get('who'))
        return out

    def scorecard(self, now=None):
        """The stops so far, in a sentence or two."""
        vis = self.visible(now)
        if not vis:
            return "I haven't stopped for anyone yet."
        n = len(vis)
        c = {k: sum(1 for s in vis if s['cls'] == k) for k, _, _ in CLASSES}
        src = 'where people really went' if any(s.get('src') == 'truth' for s in vis) else 'what I saw afterwards'
        parts = []
        if c['hit']:
            parts.append(f"{c['hit']} would have been {'a collision' if c['hit'] == 1 else 'collisions'}")
        if c['near']:
            parts.append(f"{c['near']} {'a near miss' if c['near'] == 1 else 'near misses'} within 20 centimetres")
        if c['tight']:
            parts.append(f"{c['tight']} too close for comfort")
        if c['clear']:
            parts.append(f"{c['clear']} over-cautious, looking back "
                         f"({sum(s['waited'] for s in vis if s['cls'] == 'clear'):.0f} seconds of waiting)")
        body = ', '.join(parts[:-1]) + (' and ' if len(parts) > 1 else '') + parts[-1]
        return f"I've stopped for people {n} time{'s' if n != 1 else ''}. Checking each against {src}: {body}."
