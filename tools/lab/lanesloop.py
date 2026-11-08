#!/usr/bin/env python3
"""lanesloop.py (v10.8, 13 Oct) - CLOSED-LOOP replica of 'clean aisle 1' with the real guard (GuardCore).

The world, the lidar raycast, traffic.py's people and drivers and the world's worker are
tourrep.py's (as tourloop.py). The robot drives aisle 1 the way coverage_nav2 does (transit to
lane -1.52, the five lanes up/down/up/down/up with a hop between them, then home to the dock),
at the speed the guard allows: stopped while it says WAIT, otherwise at most its speed limit and
cruise, 0.2 s start delay, 4 m/s^2 up, 6 m/s^2 down, the Collision Monitor's Brake circle.
Nav2's progress checker is mirrored by the guard with the live stack's 25 s.

Measured: total time, time per lane, every passage of the worker's walkway (y 4.0) with its
standing time, holds by person, body-to-body gaps and contacts (robot moving / standing).

    python3 lanesloop.py <guard.py> [phase=0] [seed=0] [cruise=0.8] [quiet=1] [set:NAME=value ...]
"""
import collections
import importlib.util
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                 # tourrep.py finds traffic.py and lanes.py (tools/, or $V10)
import tourrep as R  # noqa: E402

TR = R.TR
ROBOT_L, ROBOT_W = 0.6, 0.4
PERSON_R = 0.25
A_UP, A_DOWN, START_DELAY = 4.0, 6.0, 0.2
BRAKE_R, BRAKE_T = 0.32, 1.0
WALK_Y = 4.0
GOAL_PAUSE = 1.0          # s at each goal (turning round at a lane end, Nav2 taking the next goal)

ROUTE = [
    ('transit', [(0.0, 0.0), (-1.52, 1.10)]),
    ('lane -1.52 up', [(-1.52, 1.10), (-1.52, 13.90)]),
    ('hop', [(-1.52, 13.90), (-1.00, 13.90)]),
    ('lane -1.00 down', [(-1.00, 13.90), (-1.00, 1.10)]),
    ('hop', [(-1.00, 1.10), (-0.49, 1.10)]),
    ('lane -0.49 up', [(-0.49, 1.10), (-0.49, 13.90)]),
    ('hop', [(-0.49, 13.90), (0.03, 13.90)]),
    ('lane +0.03 down', [(0.03, 13.90), (0.03, 1.10)]),
    ('hop', [(0.03, 1.10), (0.54, 1.10)]),
    ('lane +0.54 up', [(0.54, 1.10), (0.54, 13.90)]),
    ('home', [(0.54, 13.90), (-0.40, 12.9), (-0.50, 11.5), (-0.50, 1.5), (0.0, 0.0)]),
]


def dense(pts, step=0.05):
    pts = [np.array(p, dtype=float) for p in pts]
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(int(float(np.hypot(*(b - a))) / step), 1)
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)


def run(guard_path, phase=0.0, seed=0, cruise=0.8, sets=None, quiet=True, brake=True, route=None,
        tmax=1500.0, trace=None):
    spec = importlib.util.spec_from_file_location('pg_lanes_%d' % abs(hash(guard_path)), guard_path)
    PG = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(PG)
    for k, v in (sets or {}).items():
        setattr(PG, k, v)
    grid = R.occupancy()
    core = PG.GuardCore(PG.StaticMap(R.W, R.H, R.RES, R.OX, R.OY, grid.ravel().tolist()))
    core.guard.prog_time = 25.0                     # stack.sh raises Nav2's allowance to 25 s
    rng = np.random.default_rng(seed)
    beams, rmax, r_min, noise, hz = 360, 12.0, 0.12, 0.01, 10.0
    a_min, a_inc = -math.pi, 2 * math.pi / beams
    rel = a_min + a_inc * np.arange(beams)
    segs = []
    for name, pts in (route or ROUTE):
        d = R.round_corners(pts, r=0.3) if len(pts) > 2 else dense(pts)
        L = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(d, axis=0).T))))
        segs.append((d, L, name))
    drivers = {n: TR.Driver(n) for n in TR.VEHICLES}
    for n, dv in drivers.items():
        sp = TR.VEHICLES[n]
        dv.pose = (sp['start'][0], sp['start'][1], sp['yaw0'])
    wps = {n: TR.person_waypoints(sp) for n, sp in TR.PEOPLE.items()}

    dt = 1.0 / TR.CONTROL_HZ
    t, k, s, v = 0.0, 0, 0.0, 0.0
    pause, start_wait = 0.0, 0.0
    next_scan = phase
    state, why, v_limit = 'go', 'clear', None
    brake_pts = np.zeros((0, 2))
    seg_t = []
    seg_t0 = phase
    gaps = collections.defaultdict(lambda: [9.0, 0.0, 0.0])
    episodes, open_ep = [], {}
    holds = collections.Counter()           # (who, why) -> s
    passages = []                           # walkway passages
    cur_pass = None
    go_events = []                          # (t, why, d_go, need) when WAIT -> GO for a crossing
    emergencies = []                        # (t, y, who, inside the worker's walkway band)
    prev_why = None
    prev_state = 'go'
    who_now = None
    last_look = None

    def free_ahead(seg_k, s_, pts, reach):
        d, L, _ = segs[seg_k]
        if not len(pts):
            return reach
        ss = np.arange(s_, min(s_ + reach, L[-1]) + 1e-9, 0.05)
        xs, ys = np.interp(ss, L, d[:, 0]), np.interp(ss, L, d[:, 1])
        dd = np.hypot(xs[:, None] - pts[None, :, 0], ys[:, None] - pts[None, :, 1]).min(axis=1)
        hit = np.nonzero(dd < BRAKE_R)[0]
        return reach if not len(hit) else float(ss[hit[0]] - s_)

    def pose_at(seg_k, s_):
        d, L, _ = segs[seg_k]
        i = int(np.searchsorted(L, s_))
        i = min(max(i, 1), len(d) - 1)
        f = (s_ - L[i - 1]) / max(L[i] - L[i - 1], 1e-9)
        p = d[i - 1] + f * (d[i] - d[i - 1])
        j = min(i + 4, len(d) - 1)
        h = math.atan2(d[j][1] - d[i - 1][1], d[j][0] - d[i - 1][0])
        plan = d[i - 1:] if len(d) - i >= 2 else None
        return float(p[0]), float(p[1]), h, plan

    while k < len(segs) and t < tmax:
        rx, ry, ryaw, plan = pose_at(k, s)
        people = {n: TR.person_at(wps[n], t) for n in wps}
        wx, wy, wvx = R.worker(t)
        truth = {n: (p[0], p[1]) for n, p in people.items()}
        truth['worker'] = (wx, wy)
        ppl_xy = list(truth.values())
        for n, dv in drivers.items():
            vv, ww = dv.step(t, dt, (rx, ry, ryaw) if t >= phase else None, ppl_xy)
            x, y, a = dv.pose
            dv.pose = (x + vv * math.cos(a) * dt, y + vv * math.sin(a) * dt, TR.wrap(a + ww * dt))
        if t + 1e-9 >= next_scan:
            next_scan += 1.0 / hz
            ang = rel + ryaw
            dx, dy = np.cos(ang), np.sin(ang)
            best = np.full(beams, np.inf)
            for b in R.BOXES.values():
                best = np.minimum(best, R.ray_aabb(rx, ry, dx, dy, b, r_min))
            for n, dv in drivers.items():
                x, y, a = dv.pose
                for wx2, wy2, a2, sx, sy in R.part_polys(n, x, y, a):
                    best = np.minimum(best, R.ray_obb(rx, ry, dx, dy, wx2, wy2, a2, sx, sy, r_min))
            legs = []
            for n, p in people.items():
                legs += R.legs_of(p[0], p[1], p[3] * math.cos(p[2]), p[3] * math.sin(p[2]), t)
            legs += R.legs_of(wx, wy, wvx, 0.0, t)
            for cx, cy in legs:
                best = np.minimum(best, R.ray_circle(rx, ry, dx, dy, cx, cy, 0.07, r_min))
            r = best + rng.normal(0.0, noise, beams)
            r[best >= rmax] = np.inf
            fin = np.isfinite(r) & (r < 2.5)
            brake_pts = np.stack([rx + r[fin] * dx[fin], ry + r[fin] * dy[fin]], axis=1)
            lp = (rx, ry, ryaw)
            core.scan(r, a_min, a_inc, r_min, rmax, lp, t, lp, plan)
            state, lk, patient = core.decide(lp[:2], lp[2], v, cruise, t, plan)
            last_look = lk
            g = core.guard
            why = g.why
            v_limit = g.v_limit
            vl = core.vlook
            veh_conflict = getattr(core, 'veh_hold', False) and not (vl.keep_out and not (vl.conflict and not vl.clear_it))
            keep_only = getattr(core, 'veh_hold', False) and not veh_conflict
            if state == 'wait' and keep_only and g.state == 'go':
                state, why = 'go', 'keep-out pass'
                v_limit = min(v_limit or cruise, PG.V_VEHICLE)
            # who is it about (the track that sets d_go), as a scripted name
            who_now = None
            if lk is not None and lk.who is not None:
                tr = next((q for q in core.people if q.id == lk.who), None)
                if tr is not None:
                    nm, dd = min(((n, math.hypot(tr.px - xy[0], tr.py - xy[1])) for n, xy in truth.items()),
                                 key=lambda z: z[1])
                    who_now = nm if dd < 1.0 else '?'
            if state == 'wait' and veh_conflict:
                who_now = 'vehicle'
            if prev_state == 'wait' and state == 'go' and why in ('crossing', 'clear') and t >= phase:
                go_events.append((round(t, 2), why, round(lk.d_go, 3), round(lk.need, 3), who_now,
                                  segs[k][2], round(ry, 2)))
            if state == 'wait' and why == 'emergency' and prev_why != 'emergency' and t >= phase:
                emergencies.append((round(t, 2), round(rx, 2), round(ry, 2), who_now,
                                    bool(-2.0 < rx < 1.0 and abs(ry - WALK_Y) < 0.76)))
            prev_why = why if state == 'wait' else None
            prev_state = state
            if trace and segs[k][2] == trace and abs(ry - WALK_Y) < 2.2 and -2.0 < rx < 1.0:
                gw = TR.circle_gap(TR.corners(rx, ry, ryaw, ROBOT_L, ROBOT_W), wx, wy, PERSON_R)
                z = lk.zone
                print('%7.2f %-4s %-10s y %5.2f v %4.2f lim %s d_go %.3f d_stop %.3f nom %.3f need %.2f inside %d zone %s who %s | worker x %5.2f vx %5.2f gap %.3f' % (
                    t, state, why, ry, v, '-' if v_limit is None else '%.2f' % v_limit, lk.d_go, lk.d_stop, lk.d_go_nom, lk.need, lk.inside,
                    '-' if z is None else '%.2f/%s/%.2f' % (z.s_hold, '-' if z.s_band is None else '%.2f' % z.s_band, z.s_out), who_now, wx, wvx, gw))
        if t < phase:
            t += dt
            continue
        if pause > 0.0:
            pause -= dt
            v = 0.0
            if pause <= 0.0:
                seg_t.append((segs[k][2], round(t - seg_t0, 2)))
                seg_t0 = t
                k += 1
                s = 0.0
            t += dt
            continue
        target = 0.0 if state == 'wait' else min(cruise, v_limit if v_limit is not None else cruise)
        if brake and target > 0.0:
            free = free_ahead(k, s, brake_pts, max(v, target) * BRAKE_T + 0.05)
            if free < max(v, target) * BRAKE_T:
                target = min(target, free / BRAKE_T)
        if v < 0.01 and target > 0.0:
            start_wait += dt
            if start_wait < START_DELAY:
                target = 0.0
        else:
            start_wait = 0.0
        v = min(target, v + A_UP * dt) if target >= v else max(target, v - A_DOWN * dt)
        s += v * dt
        if state == 'wait':
            holds[(who_now or '?', why)] += dt
        # the worker's walkway: passages through y 4.0 +- 1.6 inside aisle 1
        inw = -2.0 < rx < 1.0 and abs(ry - WALK_Y) < 1.6
        if inw and cur_pass is None:
            cur_pass = dict(t0=t, y0=ry, x=rx, stand=0.0, seg=segs[k][2], held_worker=0.0, gap_mv=9.0,
                            go=None)
        if cur_pass is not None:
            if v < 0.02:
                cur_pass['stand'] += dt
            if state == 'wait' and who_now == 'worker':
                cur_pass['held_worker'] += dt
            if v > 0.05:
                cur_pass['gap_mv'] = min(cur_pass['gap_mv'], TR.circle_gap(TR.corners(rx, ry, ryaw, ROBOT_L, ROBOT_W), wx, wy, PERSON_R))
            if go_events and cur_pass['go'] is None and go_events[-1][0] >= cur_pass['t0'] - 0.05 and go_events[-1][4] == 'worker':
                cur_pass['go'] = go_events[-1][2]
            if not inw:
                cur_pass.update(t1=t, y1=ry)
                passages.append(cur_pass)
                cur_pass = None
        poly = TR.corners(rx, ry, ryaw, ROBOT_L, ROBOT_W)
        moving = v > 0.05

        def contact(who, gp, pv, pxy=None):
            ep = open_ep.get(who)
            tow = 0.0
            if pxy is not None:
                dxp, dyp = pxy[0] - rx, pxy[1] - ry
                dn = math.hypot(dxp, dyp)
                if dn > 1e-6:
                    tow = v * (math.cos(ryaw) * dxp + math.sin(ryaw) * dyp) / dn
            if gp <= 0.0:
                if ep is None:
                    open_ep[who] = dict(who=who, t0=t, t1=t, v0=v, vmax=v, why=why, state=state, pv=pv,
                                        x=rx, y=ry, seg=segs[k][2], tow=tow)
                else:
                    ep['t1'] = t
                    ep['vmax'] = max(ep['vmax'], v)
                    ep['tow'] = max(ep['tow'], tow)
            elif ep is not None and t - ep['t1'] > 0.5:
                episodes.append(open_ep.pop(who))
        for n, p in people.items():
            gp = TR.circle_gap(poly, p[0], p[1], PERSON_R)
            contact(n, gp, p[3], (p[0], p[1]))
            G = gaps[n]
            if gp < G[0]:
                G[0] = gp
            if gp <= 0.0:
                G[1 if moving else 2] += dt
        gp = TR.circle_gap(poly, wx, wy, PERSON_R)
        contact('worker', gp, abs(wvx), (wx, wy))
        G = gaps['worker']
        if gp < G[0]:
            G[0] = gp
        if gp <= 0.0:
            G[1 if moving else 2] += dt
        for n, dv in drivers.items():
            x, y, a = dv.pose
            gp = min(TR.poly_gap(poly, q) for q in TR.outline(n, x, y, a))
            contact(n, gp, abs(dv.v) if hasattr(dv, 'v') else float('nan'))
            G = gaps[n]
            if gp < G[0]:
                G[0] = gp
            if gp <= 0.0:
                G[1 if moving else 2] += dt
        if s >= segs[k][1][-1]:
            pause = GOAL_PAUSE
            v = 0.0
        t += dt
    episodes += list(open_ep.values())
    done = k >= len(segs)
    res = dict(phase=phase, seed=seed, done=done, total=round(t - phase, 2), segs=seg_t,
               holds={f'{a}/{b}': round(c, 1) for (a, b), c in holds.items()},
               passages=[dict(seg=p['seg'], x=round(p['x'], 2), dur=round(p['t1'] - p['t0'], 2),
                              stand=round(p['stand'], 2), held_worker=round(p['held_worker'], 2),
                              gap_mv=round(p['gap_mv'], 3), go=p['go'])
                         for p in passages],
               gaps={n: [round(g[0], 3), round(g[1], 2), round(g[2], 2)] for n, g in gaps.items()},
               contacts=[dict(who=e['who'], t0=round(e['t0'], 2), t1=round(e['t1'], 2),
                              vmax=round(e['vmax'], 2), tow=round(e.get('tow', 0.0), 2), why=e['why'], seg=e['seg'],
                              x=round(e['x'], 2), y=round(e['y'], 2)) for e in episodes],
               go_events=go_events, emergencies=emergencies)
    if not quiet:
        print(json.dumps(res, indent=1))
    return res


if __name__ == '__main__':
    argv = sys.argv[1:]
    opts = dict(a.split('=', 1) for a in argv[1:] if '=' in a and not a.startswith('set:'))
    sets = {a[4:].split('=', 1)[0]: float(a[4:].split('=', 1)[1]) for a in argv[1:] if a.startswith('set:')}
    res = run(argv[0], phase=float(opts.get('phase', 0.0)), seed=int(opts.get('seed', 0)),
              cruise=float(opts.get('cruise', 0.8)), sets=sets, quiet=opts.get('quiet', '1') == '1')
    print(json.dumps(res))
