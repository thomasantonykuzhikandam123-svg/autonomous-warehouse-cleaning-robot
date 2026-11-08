#!/usr/bin/env python3
"""tourloop.py - CLOSED-LOOP replica of the Gazebo tour: the robot obeys the guard.

Same world as tourrep.py (warehouse boxes, traffic.py's drivers and people scripts, the world's
worker, a 360-beam lidar raycast against all of them at the lidar plane), but the robot drives
the tour's lane routes at the speed the guard allows: stopped while it says WAIT, otherwise at
most its /speed_limit and cruise, with a 0.2 s start delay, 4 m/s^2 up and 6 m/s^2 down (the
Collision Monitor's stop); and, like the Collision Monitor's Brake zone, never closer than 1 s
to anything the raw lidar sees within 0.32 m of its way (brake=0 turns that off). Each goal ends with a 1 s pause. The drivers react to the robot.

Not modelled: Nav2's detours. A hold that is only for a standing vehicle's fork ground (Nav2 is
shown that ground and plans round it in Gazebo) is driven through at V_VEHICLE instead, and
counted ('keep-out passes'). The robot keeps to its lane past standing people (Nav2 would bend
round their personal space), with the guard's speed caps.

Measured like tour_report: lap times, the guard's holds by reason, time under a speed limit by
source, and body-to-body gaps (robot 0.6 x 0.4 m; people 0.25 m; vehicles with forks): the worst
gap, and contact seconds with the robot moving and standing.

    python3 tourloop.py <guard.py> [laps=3] [seed=0] [cruise=0.8] [phase=0] [quiet=1]
                        [set:NAME=value ...]
"""
import collections
import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tourrep as R  # noqa: E402  (world, raycasting, routes; imports traffic as R.TR)

TR = R.TR
ROBOT_L, ROBOT_W = 0.6, 0.4
PERSON_R = 0.25
A_UP, A_DOWN, START_DELAY = 4.0, 6.0, 0.2
# the Collision Monitor's 'Brake' zone (collision_monitor_people.yaml): a 0.32 m circle driven along
# the robot's way; the command is scaled so the robot stays BRAKE_T s from anything the raw lidar sees
BRAKE_R, BRAKE_T = 0.32, 1.0


def run(guard_path, laps=3, seed=0, cruise=0.8, phase=0.0, sets=None, quiet=False, brake=True):
    BRAKE = brake
    brake_pts = np.zeros((0, 2))
    spec = importlib.util.spec_from_file_location('pg_loop', guard_path)
    PG = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(PG)
    for k, v in (sets or {}).items():
        setattr(PG, k, v)
    grid = R.occupancy()
    core = PG.GuardCore(PG.StaticMap(R.W, R.H, R.RES, R.OX, R.OY, grid.ravel().tolist()))
    rng = np.random.default_rng(seed)
    beams, rmax, r_min, noise, hz = 360, 12.0, 0.12, 0.01, 10.0
    a_min, a_inc = -math.pi, 2 * math.pi / beams
    rel = a_min + a_inc * np.arange(beams)

    # the route: laps x 3 goals, each a dense polyline
    segs = []
    for lap in range(laps):
        for gi, pts in R.polyline_route():
            d = R.round_corners(pts)
            L = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(d, axis=0).T))))
            segs.append((d, L, gi, lap + 1))

    drivers = {n: TR.Driver(n) for n in TR.VEHICLES}
    for n, d in drivers.items():
        sp = TR.VEHICLES[n]
        d.pose = (sp['start'][0], sp['start'][1], sp['yaw0'])
    wps = {n: TR.person_waypoints(sp) for n, sp in TR.PEOPLE.items()}

    dt = 1.0 / TR.CONTROL_HZ
    t = 0.0
    k, s, v = 0, 0.0, 0.0
    pause = 0.0
    start_wait = 0.0
    lap_t0 = {1: phase}
    lap_end = {}
    next_scan = phase
    state, why, v_limit = 'go', 'clear', None
    stats = collections.Counter()
    caps = collections.Counter()
    holds = collections.Counter()
    hold_events = collections.Counter()
    prev_hold = None
    gaps = collections.defaultdict(lambda: [9.0, 0.0, 0.0])      # worst, contact moving s, still s
    keep_pass = 0
    last = None
    episodes = []          # contact episodes: dict(who, t0, t1, v0, vmax, why, state, pv)
    open_ep = {}

    def free_ahead(seg_k, s_, pts, reach):
        """m the Brake circle can go along the route before it touches a lidar point (<= reach)."""
        d, L, gi, lap = segs[seg_k]
        if not len(pts):
            return reach
        ss = np.arange(s_, min(s_ + reach, L[-1]) + 1e-9, 0.05)
        xs, ys = np.interp(ss, L, d[:, 0]), np.interp(ss, L, d[:, 1])
        dd = np.hypot(xs[:, None] - pts[None, :, 0], ys[:, None] - pts[None, :, 1]).min(axis=1)
        hit = np.nonzero(dd < BRAKE_R)[0]
        return reach if not len(hit) else float(ss[hit[0]] - s_)

    def pose_at(seg_k, s_):
        d, L, gi, lap = segs[seg_k]
        i = int(np.searchsorted(L, s_))
        i = min(max(i, 1), len(d) - 1)
        f = (s_ - L[i - 1]) / max(L[i] - L[i - 1], 1e-9)
        p = d[i - 1] + f * (d[i] - d[i - 1])
        j = min(i + 4, len(d) - 1)
        h = math.atan2(d[j][1] - d[i - 1][1], d[j][0] - d[i - 1][0])
        plan = d[i - 1:] if len(d) - i >= 2 else None
        return float(p[0]), float(p[1]), h, plan

    while k < len(segs) and t < 4000.0:
        d, L, gi, lap = segs[k]
        rx, ry, ryaw, plan = pose_at(k, s)
        people = {n: TR.person_at(wps[n], t) for n in wps}
        wx, wy, wvx = R.worker(t)
        ppl_xy = [(p[0], p[1]) for p in people.values()] + [(wx, wy)]
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
            state, _, _ = core.decide(lp[:2], lp[2], v, cruise, t, plan)
            g = core.guard
            why = g.why
            v_limit = g.v_limit
            vl = core.vlook
            veh_conflict = getattr(core, 'veh_hold', False) and not (vl.keep_out and not (vl.conflict and not vl.clear_it))
            keep_only = getattr(core, 'veh_hold', False) and not veh_conflict
            if state == 'wait' and keep_only and g.state == 'go':
                state, why = 'go', 'keep-out pass'           # Nav2 would plan round the fork ground
                v_limit = min(v_limit or cruise, PG.V_VEHICLE)
                keep_pass += 1
            last = dict(veh_cap=getattr(core, 'veh_cap', None), cap=getattr(core, 'cap', None),
                        g_limit=g.v_limit, veh_conflict=veh_conflict, people_wait=g.state == 'wait')
        # the robot (stands at the start until t = phase: the traffic runs alone first)
        if t < phase:
            t += dt
            continue
        if pause > 0.0:
            pause -= dt
            v = 0.0
            if pause <= 0.0:
                k += 1
                s = 0.0
                if k < len(segs) and segs[k][3] != lap:
                    lap_end[lap] = t
                    lap_t0[segs[k][3]] = t
            t += dt
            continue
        target = 0.0 if state == 'wait' else min(cruise, v_limit if v_limit is not None else cruise)
        if BRAKE and target > 0.0:
            free = free_ahead(k, s, brake_pts, max(v, target) * BRAKE_T + 0.05)
            if free < max(v, target) * BRAKE_T:
                target = min(target, free / BRAKE_T)
                stats['brake'] += dt
        if v < 0.01 and target > 0.0:
            start_wait += dt
            if start_wait < START_DELAY:
                target = 0.0
        else:
            start_wait = 0.0
        v = min(target, v + A_UP * dt) if target >= v else max(target, v - A_DOWN * dt)
        s += v * dt
        # bookkeeping
        if state == 'wait':
            reason = 'vehicle' if (last and last['veh_conflict']) else ('person ' + why)
            holds[reason] += dt
            if prev_hold != reason:
                hold_events[reason] += 1
            prev_hold = reason
        else:
            prev_hold = None
            if v_limit is not None and v_limit < cruise - 0.05:
                src = []
                if last and last['veh_cap'] is not None and v_limit <= last['veh_cap'] + 1e-6:
                    src.append('vehicle cap')
                if last and last['cap'] is not None and v_limit <= last['cap'] + 1e-6:
                    src.append('standing person')
                if why == 'keep-out pass':
                    src.append('keep-out pass')
                caps[' + '.join(src) or 'people limit'] += dt
            else:
                stats['free'] += dt
        poly = TR.corners(rx, ry, ryaw, ROBOT_L, ROBOT_W)
        moving = v > 0.05

        def contact(who, gp, pv):
            ep = open_ep.get(who)
            if gp <= 0.0:
                if ep is None:
                    open_ep[who] = dict(who=who, t0=t, t1=t, v0=v, vmax=v, why=why, state=state,
                                        pv=pv, lap=lap)
                else:
                    ep['t1'] = t
                    ep['vmax'] = max(ep['vmax'], v)
            elif ep is not None and t - ep['t1'] > 0.5:
                episodes.append(open_ep.pop(who))
        for n, p in people.items():
            gp = TR.circle_gap(poly, p[0], p[1], PERSON_R)
            contact(n, gp, p[3])
            G = gaps[n]
            G[0] = min(G[0], gp)
            if gp <= 0.0:
                G[1 if moving else 2] += dt
        gp = TR.circle_gap(poly, wx, wy, PERSON_R)
        contact('worker', gp, abs(wvx))
        G = gaps['worker']
        G[0] = min(G[0], gp)
        if gp <= 0.0:
            G[1 if moving else 2] += dt
        for n, dv in drivers.items():
            x, y, a = dv.pose
            gp = min(TR.poly_gap(poly, q) for q in TR.outline(n, x, y, a))
            contact(n, gp, abs(dv.v) if hasattr(dv, 'v') else float('nan'))
            G = gaps[n]
            G[0] = min(G[0], gp)
            if gp <= 0.0:
                G[1 if moving else 2] += dt
        if s >= L[-1]:
            pause = 1.0
            v = 0.0
        t += dt
    episodes += list(open_ep.values())
    if k >= len(segs):
        lap_end[segs[-1][3]] = t
    lap_times = [lap_end[l] - lap_t0[l] for l in sorted(lap_end)]
    res = dict(laps=lap_times, holds=dict(holds), hold_events=dict(hold_events), caps=dict(caps),
               free=stats['free'], brake=stats['brake'], gaps={n: tuple(gv) for n, gv in gaps.items()}, keep_pass=keep_pass / hz,
               total=t, episodes=episodes)
    if not quiet:
        print_result(os.path.basename(guard_path), res)
    return res


def print_result(name, res):
    lt = res['laps']
    print(f'=== tourloop {name}: {len(lt)} laps, ' + ', '.join(f'{x:.1f}' for x in lt) +
          f' s (mean {np.mean(lt):.1f} s)')
    print('  held: ' + ', '.join(f'{k} {v:.0f} s ({res["hold_events"].get(k, 0)}x)'
                                 for k, v in sorted(res['holds'].items(), key=lambda kv: -kv[1])))
    print('  under a speed limit: ' + ', '.join(f'{k} {v:.0f} s' for k, v in
                                                 sorted(res['caps'].items(), key=lambda kv: -kv[1])) +
          f'; free {res["free"]:.0f} s; keep-out passes {res["keep_pass"]:.0f} s; '
          f'Collision Monitor braking {res.get("brake", 0):.0f} s')
    print('  gaps (worst, contact s moving / standing): ' + '; '.join(
        f'{n} {g[0]:.2f} ({g[1]:.1f}/{g[2]:.1f})' for n, g in sorted(res['gaps'].items())))
    eps = res.get('episodes', [])
    mv = [e for e in eps if e['vmax'] > 0.05]
    print(f'  contacts: {len(eps)}, {len(mv)} with the robot moving; robot speed at contact '
          f'max {max([e["vmax"] for e in eps] or [0]):.2f} m/s, over 0.3 m/s in '
          f'{sum(e["vmax"] > 0.3 for e in eps)}')
    for e in eps:
        print(f'    lap {e["lap"]} t {e["t0"]:6.1f}-{e["t1"]:6.1f} {e["who"]:9s} robot {e["v0"]:.2f} -> max '
              f'{e["vmax"]:.2f} m/s ({e["state"]} {e["why"]}), them {e["pv"]:.2f} m/s')


if __name__ == '__main__':
    argv = sys.argv[1:]
    opts = dict(a.split('=', 1) for a in argv[1:] if '=' in a and not a.startswith('set:'))
    sets = {a[4:].split('=', 1)[0]: float(a[4:].split('=', 1)[1]) for a in argv[1:] if a.startswith('set:')}
    run(argv[0], laps=int(opts.get('laps', 3)), seed=int(opts.get('seed', 0)),
        cruise=float(opts.get('cruise', 0.8)), phase=float(opts.get('phase', 0.0)), sets=sets,
        brake=opts.get('brake', '1') == '1')
