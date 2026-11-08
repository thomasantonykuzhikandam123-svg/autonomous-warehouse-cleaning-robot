#!/usr/bin/env python3
"""wet_floor.py (v10.13, 2 Nov) - the wet-floor guardian: a scrubber leaves the floor wet behind it, and the
robot knows two things nobody else does: where it has just cleaned, and who is about to walk there.

Plain words: it keeps a map of the floor it wet-cleaned and lets it dry (T_DRY). It learns, from its own
lidar tracks only, how often people are at each spot, counting only the time it could actually see that
spot. Where people are expected on the spot before it would dry, or someone is about to walk there, it
turns its water off and cleans dry; that strip still needs a wet pass when it is quiet, and it says so.
When someone it tracks is about to walk onto wet floor, it warns them.

The maths, per spot c of the floor (0.25 m cells):
    rate(c) = P(c) / (O(c) + O_MIN)
P = seconds a tracked person was within NEAR of c, O = seconds c was inside the lidar's view (so a spot
it never saw is not assumed busy: it simply does not know). rate x T_DRY = seconds someone is expected at
the spot while it dries; above H_OFF the water goes off there. O_MIN pulls a spot it has hardly watched
towards 'rare'. Also off when a tracked person is within NOW_R of the brush or predicted (their next
2.5 s, from the people guard) within PRED_R of it.

Nothing here moves the robot: it decides the water and what to say. robot_web.py runs it (live and in
replay) and shows it; on a real scrubber the water decision would go to the cleaning head.
Simulator truth (measure()) is only ever used to count who really stepped on wet floor, never to decide.
"""
import math

import numpy as np

VERSION = 'v10.13'
RES = 0.1            # m: the wet map's cell
HRES = 0.25          # m: the foot-traffic (habits) cell
BRUSH = 0.32         # m: half the cleaning width (coverage_recorder's clean_width 0.64)
T_DRY = 120.0        # s: how long a wet-cleaned floor stays wet (assumed; a parameter)
NEAR = 0.35          # m: a person within this of a spot is 'at' it
H_OFF = 1.0          # s: water off where people are expected at the spot this long while it dries
O_MIN = 30.0         # s: a spot watched less than this counts as rarely used, not busy
O_CAP = 1800.0       # s: when a spot has been watched this long, its history is halved (keeps learning)
NOW_R = 0.5          # m: water off with a tracked person this close to the brush ...
PRED_R = 0.6         # m: ... or predicted to pass this close (the guard's 2.5 s prediction)
OFF_HOLD = 1.0       # s: the water stays off this long after the last reason
WARN_TRACK_S = 20.0  # s: between two warnings to the same person
WARN_GAP_S = 5.0     # s: between any two warnings
WARN_MIN = 0.15      # wetness (1 = just cleaned, 0 = dry) worth a warning when they would reach it
PASS_S = 0.7         # s: a person walking past spends about this long within NEAR of a spot
DRY_PENDING = 200    # value in the exported wet grid: cleaned with the water off, needs a wet pass
RECENT = 1800.0      # s: floor wet-cleaned this recently does not need another wet pass
ENCOUNTER_GAP = 5.0  # s: back on wet floor within this is the same encounter (truth counting only)


def _disk(radius, res):
    r = int(math.ceil(radius / res))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    keep = (xx * xx + yy * yy) * res * res <= radius * radius
    return yy[keep], xx[keep]


def people_from_shapes(shapes):
    """The page's shapes (people_guard's markers) -> [{id, x, y, fresh, preds: [pts]}] for the people the
    robot tracks. Grey = lost from view (remembered): not fresh."""
    people, preds = {}, {}
    for s in shapes or []:
        ns = s.get('ns')
        if ns == 'person' and s.get('k') == 'c':
            c = s.get('rgba') or [1, 1, 1, 1]
            grey = abs(c[0] - c[1]) < 0.05 and abs(c[1] - c[2]) < 0.05 and 0.45 < c[0] < 0.75
            people[('p', s['id'])] = {'id': 'p%d' % s['id'], 'x': float(s['x']), 'y': float(s['y']),
                                      'fresh': not grey, 'preds': []}
        elif ns == 'standing' and s.get('k') == 'c':
            people[('s', s['id'])] = {'id': 's%d' % s['id'], 'x': float(s['x']), 'y': float(s['y']),
                                      'fresh': True, 'preds': []}
        elif ns == 'predicted_path' and s.get('pts'):
            preds.setdefault(int(s['id']) // 10, []).append(s['pts'])
    for (kind, i), p in people.items():
        if kind == 'p':
            p['preds'] = preds.get(i, [])
    return list(people.values())


def cleaning_now(mission, zones, x, y, v):
    """Is the scrub deck down? A job running, the robot inside the zone it is working on, moving."""
    m = mission or {}
    z = m.get('zone')
    if z not in zones or m.get('done') or m.get('paused') or m.get('awaiting_command'):
        return False
    x0, x1, y0, y1 = zones[z]
    return x0 - 0.3 <= x <= x1 + 0.3 and y0 - 0.3 <= y <= y1 + 0.3 and abs(v) > 0.05


class WetFloor:
    def __init__(self, w, h, res, ox, oy, policy='guardian', t_dry=T_DRY, h_off=H_OFF, now_r=NOW_R, pred_r=PRED_R):
        """w, h, res, ox, oy: the map's grid. policy 'guardian' (water off where people walk, warnings)
        or 'always' (the water always on, no warnings: today's scrubbers)."""
        self.now_r, self.pred_r = float(now_r), float(pred_r)
        self.ox, self.oy = float(ox), float(oy)
        span_x, span_y = w * res, h * res
        self.W, self.H = int(math.ceil(span_x / RES)), int(math.ceil(span_y / RES))
        self.HW, self.HH = int(math.ceil(span_x / HRES)), int(math.ceil(span_y / HRES))
        self.policy, self.t_dry, self.h_off = policy, float(t_dry), float(h_off)
        self.wet_t = np.full((self.H, self.W), -1e9)          # when it was last wet-cleaned
        self.dry = np.zeros((self.H, self.W), bool)           # cleaned dry, no wet pass since
        self.cleaned = np.zeros((self.H, self.W), bool)       # cleaned at all (this job)
        self.P = np.zeros((self.HH, self.HW))                 # person-seconds near each spot
        self.O = np.zeros((self.HH, self.HW))                 # seconds each spot was in view
        hx = self.ox + (np.arange(self.HW) + 0.5) * HRES
        hy = self.oy + (np.arange(self.HH) + 0.5) * HRES
        self.HX, self.HY = np.meshgrid(hx, hy)
        self.bdy, self.bdx = _disk(BRUSH, RES)
        self.ndy, self.ndx = _disk(NEAR, HRES)
        self.mdy, self.mdx = _disk(1.0, HRES)                  # for saying how busy the spot is
        self.t = None
        self.last = None              # last robot (x, y) painted
        self.pump = 'idle'
        self.why = ''
        self.why_last = ''            # why the water last went off (kept through a pause)
        self.off_until = -1e9
        self.rate_here = 0.0
        self.warn_t = -1e9
        self.warned_ids = {}          # person id -> last warning t
        self.warnings = []            # (t, x, y, text)
        self.n_warned = 0
        self.on_wet = {}              # person id -> on wet floor now (tracked)
        self.n_seen_on_wet = 0
        self.truth_on = {}            # truth agent name -> (on wet now, since)
        self.truth_stats = {'entries': 0, 'warned': 0, 'seconds': 0.0,
                            'by_kind': {}, 'by_name': {}, 'seconds_by_kind': {}, 'seconds_by_name': {}}
        self.truth_last = {}          # truth agent name -> last time on wet floor
        self.truth_t = None
        self.v = 0                    # bumped when the exported grids change
        self.v_wall = 0.0
        self.events = []              # ('say', text, extra) for the caller to show

    def reset_floor(self):
        """A new floor (a replay restarted): the wet map, the water, warnings and counts start again; what
        it learned about where people walk is kept."""
        self.wet_t[:] = -1e9
        self.dry[:] = False
        self.cleaned[:] = False
        self.t, self.last, self.pump, self.why, self.off_until = None, None, 'idle', '', -1e9
        self.warn_t, self.warned_ids, self.warnings, self.n_warned = -1e9, {}, [], 0
        self.on_wet, self.n_seen_on_wet, self.truth_on, self.truth_t = {}, 0, {}, None
        self.truth_stats = {'entries': 0, 'warned': 0, 'seconds': 0.0, 'by_kind': {}, 'by_name': {},
                            'seconds_by_kind': {}, 'seconds_by_name': {}}
        self.truth_last = {}
        self.events = []
        self.v += 1

    # ------------------------------------------------------------------ cells
    def cell(self, x, y):
        return int(math.floor((y - self.oy) / RES)), int(math.floor((x - self.ox) / RES))

    def hcell(self, x, y):
        return int(math.floor((y - self.oy) / HRES)), int(math.floor((x - self.ox) / HRES))

    def wetness(self, t, x=None, y=None):
        """1 = just cleaned, 0 = dry; for one spot, or the whole map."""
        if x is None:
            return np.clip(1.0 - (t - self.wet_t) / self.t_dry, 0.0, 1.0)
        r, c = self.cell(x, y)
        if not (0 <= r < self.H and 0 <= c < self.W):
            return 0.0
        return float(min(1.0, max(0.0, 1.0 - (t - self.wet_t[r, c]) / self.t_dry)))

    def rate(self):
        return self.P / (self.O + O_MIN)

    # ------------------------------------------------------------------ learning where people walk
    def observe(self, dt, view, people):
        """Learning only from what the lidar can see now: a spot's watched time and, there, the time a
        person it tracks was at it."""
        if dt <= 0.0 or not view or not view.get('pts') or len(view['pts']) < 3:
            return
        o = view['o']
        pts = np.asarray(view['pts'], dtype=float)
        ang = np.arctan2(pts[:, 1] - o[1], pts[:, 0] - o[0])
        rad = np.hypot(pts[:, 0] - o[0], pts[:, 1] - o[1])
        k = np.argsort(ang)
        ang, rad = ang[k], rad[k]
        dx, dy = self.HX - o[0], self.HY - o[1]
        d = np.hypot(dx, dy)
        near = d <= float(view.get('r') or 8.0)
        a = np.arctan2(dy[near], dx[near])
        r_at = np.interp(a, ang, rad, period=2 * math.pi)
        vis = np.zeros_like(near)
        vis[near] = d[near] < r_at - 0.1
        self.O[vis] += dt
        for p in people:
            if not p.get('fresh'):
                continue
            r, c = self.hcell(p['x'], p['y'])
            rr, cc = self.ndy + r, self.ndx + c
            ok = (rr >= 0) & (rr < self.HH) & (cc >= 0) & (cc < self.HW)
            rr, cc = rr[ok], cc[ok]
            seen = vis[rr, cc]
            self.P[rr[seen], cc[seen]] += dt
        big = self.O > O_CAP
        if big.any():
            self.P[big] *= 0.5
            self.O[big] *= 0.5

    # ------------------------------------------------------------------ one step (10 Hz)
    def step(self, t, x, y, cleaning, people, view=None):
        if self.t is not None and t < self.t - 5.0:               # time went back (a replay restarted)
            self.reset_floor()
        dt = 0.0 if self.t is None else min(max(t - self.t, 0.0), 0.5)
        self.t = t
        self.observe(dt, view, people)
        if not cleaning:
            self.last = None
            if self.pump != 'idle':
                self.pump, self.why = 'idle', ''
                self.v += 1
        else:
            self.decide(t, x, y, people)
            self.paint(t, x, y)
        if self.policy == 'guardian':
            self.warn(t, people)
        self.track_seen(t, people)
        return self.pump

    def decide(self, t, x, y, people):
        if self.policy == 'always':
            self.pump, self.why = 'on', ''
            return
        reason = None
        r, c = self.hcell(x, y)
        rr, cc = self.ndy + r, self.ndx + c                     # the brush's spot, as habits see it
        ok = (rr >= 0) & (rr < self.HH) & (cc >= 0) & (cc < self.HW)
        rate = float((self.P[rr[ok], cc[ok]] / (self.O[rr[ok], cc[ok]] + O_MIN)).max()) if ok.any() else 0.0
        self.rate_here = rate
        if rate * self.t_dry > self.h_off:
            rr, cc = self.mdy + r, self.mdx + c
            ok = (rr >= 0) & (rr < self.HH) & (cc >= 0) & (cc < self.HW)
            peak = float((self.P[rr[ok], cc[ok]] / (self.O[rr[ok], cc[ok]] + O_MIN)).max()) if ok.any() else rate
            per_min = max(rate, peak) * 60.0 / PASS_S
            reason = ('people walk across here about %d times a minute' % round(per_min) if per_min >= 1.5
                      else 'people walk across here every few minutes')
        for p in people:
            if not p.get('fresh'):
                continue
            if math.hypot(p['x'] - x, p['y'] - y) <= self.now_r + BRUSH:
                reason = 'someone is right here'
                break
            for pts in p.get('preds') or []:
                q = np.asarray(pts, dtype=float)
                if len(q) and float(np.hypot(q[:, 0] - x, q[:, 1] - y).min()) <= self.pred_r + BRUSH:
                    reason = 'someone is about to walk here'
                    break
            if reason == 'someone is about to walk here':
                break
        if reason is not None:
            self.off_until = t + OFF_HOLD
            self.why = reason
            self.why_last = reason
        new = 'off' if t < self.off_until else 'on'
        if new == 'off' and not self.why:          # cleaning again inside OFF_HOLD after a pause: the same reason
            self.why = self.why_last
        if new != self.pump:
            if new == 'off' and (self.pump == 'on' or self.pump == 'idle'):
                self.events.append(('water_off', self.why, {'x': round(x, 2), 'y': round(y, 2)}))
            self.pump = new
            if new == 'on':
                self.why = ''
            self.v += 1

    def paint(self, t, x, y):
        lx, ly = self.last if self.last is not None else (x, y)
        d = math.hypot(x - lx, y - ly)
        if d > 2.0:
            lx, ly, d = x, y, 0.0
        n = max(1, int(d / (2 * RES)) + 1)
        rs, cs = [], []
        for k in range(n + 1):
            f = k / n
            r0, c0 = self.cell(lx + (x - lx) * f, ly + (y - ly) * f)
            rs.append(self.bdy + r0)
            cs.append(self.bdx + c0)
        rr, cc = np.concatenate(rs), np.concatenate(cs)
        ok = (rr >= 0) & (rr < self.H) & (cc >= 0) & (cc < self.W)
        rr, cc = rr[ok], cc[ok]
        self.cleaned[rr, cc] = True
        if self.pump == 'on':
            self.wet_t[rr, cc] = t
            self.dry[rr, cc] = False
        else:
            stale = self.wet_t[rr, cc] < t - RECENT            # not wet-cleaned lately: it needs a wet pass
            self.dry[rr[stale], cc[stale]] = True
        self.last = (x, y)

    # ------------------------------------------------------------------ warnings
    def warn(self, t, people):
        if t - self.warn_t < WARN_GAP_S:
            return
        for p in people:
            if not p.get('fresh'):
                continue
            if t - self.warned_ids.get(p['id'], -1e9) < WARN_TRACK_S:
                continue
            under = self.wetness(t, p['x'], p['y'])
            ahead = None
            for pts in p.get('preds') or []:
                for i, (px, py) in enumerate(pts[1:], 1):
                    if self.wetness(t + 0.1 * i, px, py) >= WARN_MIN:
                        ahead = (i * 0.1, px, py)
                        break
                if ahead:
                    break
            if under >= WARN_MIN:
                text = "Careful, you're on wet floor: I've just cleaned there."
            elif ahead is not None:
                text = 'Careful, the floor ahead of you is wet: I cleaned it a moment ago.'
            else:
                continue
            self.warn_t = t
            self.warned_ids[p['id']] = t
            self.n_warned += 1
            self.warnings.append((t, p['x'], p['y'], text))
            self.warnings = self.warnings[-200:]
            self.events.append(('warn', text, {'x': round(p['x'], 2), 'y': round(p['y'], 2), 'id': p['id']}))
            return

    def track_seen(self, t, people):
        for p in people:
            if not p.get('fresh'):
                continue
            on = self.wetness(t, p['x'], p['y']) > 0.0
            if on and not self.on_wet.get(p['id']):
                self.n_seen_on_wet += 1
            self.on_wet[p['id']] = on

    # ------------------------------------------------------------------ simulator truth: counting only
    def measure(self, t, agents):
        """Who really stepped onto wet floor (the simulator's truth). Never used to decide anything.
        One encounter = stepping onto wet floor more than ENCOUNTER_GAP s after last being on it (so
        walking over a patchwork of wet and dry strips is one encounter, not ten)."""
        dt = 0.0 if self.truth_t is None else min(max(t - self.truth_t, 0.0), 0.5)
        self.truth_t = t
        st = self.truth_stats
        for a in agents or []:
            name, kind = a.get('name'), a.get('kind') or 'person'
            on = self.wetness(t, a['x'], a['y']) > 0.0
            was = self.truth_on.get(name, False)
            if on:
                st['seconds'] += dt
                st['seconds_by_kind'][kind] = st['seconds_by_kind'].get(kind, 0.0) + dt
                st['seconds_by_name'][name] = st['seconds_by_name'].get(name, 0.0) + dt
            new = on and not was and t - self.truth_last.get(name, -1e9) > ENCOUNTER_GAP
            if on:
                self.truth_last[name] = t
            if new:
                st['entries'] += 1
                st['by_kind'][kind] = st['by_kind'].get(kind, 0) + 1
                st['by_name'][name] = st['by_name'].get(name, 0) + 1
                if any(t - wt <= 6.0 and math.hypot(wx - a['x'], wy - a['y']) <= 2.0
                       for wt, wx, wy, _ in self.warnings[-20:]):
                    st['warned'] += 1
            self.truth_on[name] = on

    # ------------------------------------------------------------------ for the page
    def area(self, t):
        cell = RES * RES
        return {'wet_m2': round(float((self.wetness(t) > 0).sum()) * cell, 1),
                'dry_m2': round(float(self.dry.sum()) * cell, 1),
                'cleaned_m2': round(float(self.cleaned.sum()) * cell, 1)}

    def summary(self, t, truth=False):
        out = dict(self.area(t), version=VERSION, v=self.v, pump=self.pump, why=self.why, policy=self.policy,
                   t_dry=self.t_dry, warned=self.n_warned, seen_on_wet=self.n_seen_on_wet,
                   rate_here=round(self.rate_here, 3))
        if truth:
            st = self.truth_stats
            out['truth'] = {'entries': st['entries'], 'warned': st['warned'], 'seconds': round(st['seconds'], 1)}
        return out

    def grids(self, t):
        """uint8 maps for the page: wet (0 nothing, 1..100 wetness %, 200 cleaned dry) and habits
        (how often people are at each spot: 255 = always)."""
        wet = np.round(self.wetness(t) * 100).astype(np.uint8)
        wet[(wet == 0) & self.dry] = DRY_PENDING
        rate = self.rate()
        hab = np.clip(np.round(np.sqrt(np.clip(rate, 0, 1)) * 255), 0, 255).astype(np.uint8)
        hab[self.O < 1.0] = 0
        return wet, hab

    def take_events(self):
        ev, self.events = self.events, []
        return ev

    def finish_text(self):
        a = self.area(self.t or 0.0)
        if self.policy != 'guardian' or a['dry_m2'] < 0.5:
            return None
        return ('I cleaned %.1f m² dry where people walk; it needs a wet pass when it is quiet.' % a['dry_m2'])

    # ------------------------------------------------------------------ habits across shifts
    def save(self, path):
        np.savez_compressed(path, P=self.P, O=self.O, ox=self.ox, oy=self.oy, hres=HRES, version=VERSION)

    def load(self, path):
        try:
            d = np.load(path)
            if (d['P'].shape == self.P.shape and abs(float(d['ox']) - self.ox) < 1e-6
                    and abs(float(d['oy']) - self.oy) < 1e-6 and abs(float(d['hres']) - HRES) < 1e-9):
                self.P, self.O = d['P'].astype(float), d['O'].astype(float)
                return True
        except Exception:
            pass
        return False
