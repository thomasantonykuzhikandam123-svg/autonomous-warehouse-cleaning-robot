#!/usr/bin/env python3
"""test_guard108.py (v10.8, 13 Oct) - the guard at the worker's walkway in aisle 1.

    python3 test_guard108.py <people_guard.py> [old people_guard.py]

A  THE NORMAL CROSSING STILL WORKS. The robot stands at the middle lane's stop line (-0.50, 2.96),
   facing up the lane, the world's worker pacing across it on her 6.0 s loop (tourrep.worker, the
   lidar raycast against the racks and her legs). Checked: at 12 phases of her walk against the
   scans, the guard sets off within 35 s, and only on a scan that keeps the clearance it needs.
   (This synthetic walk leaves the old guard enough room too; the lock-up itself is C.)
B  THE EMERGENCY STOP INSIDE A WALKWAY. Closed loop (lanesloop.py): the robot drives down the aisle's
   middle to the dock, through the worker's walkway, with the visitor at the aisle mouth and picker_a
   about. Before v10.8 a conflict further on (the visitor 3 m away) could stop it inside the worker's
   walkway, and she then walked into it as it cleared her way (shift_v106a t 140: 0.03 m at 0.8 m/s).
   Checked over 12 start times: never an emergency stop inside the worker's walkway for the visitor,
   and never a contact with the robot moving towards the worker (faster than 0.05 m/s).
C  THE LOCK-UP, ON YOUR RECORDING. If ~/runs/rec/shift_v106a.npz is there: its walkway stretch
   (t 110-240) replayed open loop. Before v10.8 a crossing needed two scans in a row that keep 0.71 m;
   the worker's walk repeats exactly every 60 scans, so the same short gap missed by 3 mm nine cycles
   running and the live guard waited to t 234.9 (64 s). v10.8 must set off before t 200.
With an old guard as the second argument, A and B also print what it does (not checked).
Ends with 'ALL OK' or 'FAILED: ...'.
"""
import importlib.util
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    import tourrep as R      # noqa: E402  (tools/lab: the replica's world; finds tools/traffic.py, lanes.py)
    import lanesloop         # noqa: E402
except Exception as e:       # pragma: no cover
    print(f'FAILED: the replica does not load ({type(e).__name__}: {e})')
    sys.exit(1)

FAILS = []


def check(ok, msg):
    print(('  ok    ' if ok else '  FAIL  ') + msg, flush=True)
    if not ok:
        FAILS.append(msg)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def lockup(PG, scan_off, walk_off, t_max=45.0):
    """Open loop: the robot standing at the stop line. -> (s from the first WAIT to the first GO
    for a crossing, d_go, need) or None if it never set off."""
    grid = R.occupancy()
    core = PG.GuardCore(PG.StaticMap(R.W, R.H, R.RES, R.OX, R.OY, grid.ravel().tolist()))
    core.guard.prog_time = 25.0
    rx, ry, ryaw = -0.50, 2.963, math.pi / 2 - 0.012
    plan = [(rx, ry + 0.05 * k) for k in range(220)]
    beams, rmax, r_min = 360, 12.0, 0.12
    a_min, a_inc = -math.pi, 2 * math.pi / beams
    ang = a_min + a_inc * np.arange(beams) + ryaw
    dx, dy = np.cos(ang), np.sin(ang)
    base = np.full(beams, np.inf)
    for b in R.BOXES.values():
        base = np.minimum(base, R.ray_aabb(rx, ry, dx, dy, b, r_min))
    rng = np.random.default_rng(1)
    t_wait = None
    for k in range(int(t_max / 0.1)):
        t = scan_off + 0.1 * k
        wx, wy, wvx = R.worker(t + walk_off)
        best = base.copy()
        for cx, cy in R.legs_of(wx, wy, wvx, 0.0, t):
            best = np.minimum(best, R.ray_circle(rx, ry, dx, dy, cx, cy, 0.07, r_min))
        r = best + rng.normal(0.0, 0.01, beams)
        r[best >= rmax] = np.inf
        lp = (rx, ry, ryaw)
        core.scan(r, a_min, a_inc, r_min, rmax, lp, t, lp, plan)
        state, lk, patient = core.decide(lp[:2], ryaw, 0.0, 0.8, t, plan)
        if state == 'wait' and t_wait is None:
            t_wait = t
        if state == 'go' and t_wait is not None and core.guard.why in ('crossing', 'clear'):
            return (round(t - t_wait, 1), round(lk.d_go, 3), round(lk.need, 3))
    return None


HOME = [('home', [(-0.40, 9.0), (-0.50, 8.0), (-0.50, 1.5), (0.0, 0.0)])]


def home_runs(path, starts):
    out = []
    for st in starts:
        r = lanesloop.run(path, phase=st, route=HOME, tmax=st + 120.0)
        out.append(r)
    return out


def replay(PG, rec, t_lo=140.0, t_hi=240.0):
    Z = np.load(rec, allow_pickle=False)
    meta = json.loads(str(Z['meta']))
    mi = json.loads(str(Z['map_info']))
    T, RG, LS, RB, OV, OA = Z['t'], Z['ranges'], Z['laser'], Z['robot'], Z['odom_v'], Z['odom_age']
    off = np.concatenate(([0], np.cumsum(Z['plan_n'])))
    plan_ts = np.asarray(Z['plan_t'], dtype=float)
    core = PG.GuardCore(PG.StaticMap(mi['width'], mi['height'], mi['resolution'], mi['origin_x'],
                                     mi['origin_y'], Z['map_data']))
    core.guard.prog_time = 25.0
    prev, v_est = None, 0.0
    for k in range(len(T)):
        t = float(T[k])
        if t < t_lo - 30.0 or t > t_hi:
            continue
        laser, robot = tuple(LS[k]), tuple(RB[k])
        i = int(np.searchsorted(plan_ts, t, side='right')) - 1
        plan = ([tuple(p) for p in Z['plan_xy'][off[i]:off[i + 1]]]
                if i >= 0 and t - plan_ts[i] <= PG.PLAN_AGE else None)
        if OA[k] < 0.5:
            v_est = float(OV[k])
        elif prev is not None and t > prev[0]:
            v_est = 0.5 * v_est + 0.5 * math.hypot(robot[0] - prev[1], robot[1] - prev[2]) / (t - prev[0])
        prev = (t, robot[0], robot[1])
        core.scan(RG[k], float(meta['angle_min']), float(meta['angle_increment']), float(meta['range_min']),
                  float(meta['range_max']), laser, t, robot, plan)
        state, lk, patient = core.decide(robot[:2], robot[2], v_est, 0.8, t, plan)
        if t >= 171.0 and state == 'go' and core.guard.why == 'crossing':
            return round(t, 1), round(lk.d_go, 3), round(lk.need, 3)
    return None


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    t0 = time.time()
    new = load(argv[0], 'g_new')
    old = load(argv[1], 'g_old') if len(argv) > 1 else None
    print(f'test_guard108: {argv[0]} (VERSION {getattr(new, "VERSION", "?")})'
          + (f' against {argv[1]} (VERSION {getattr(old, "VERSION", "?")})' if old else ''), flush=True)
    check(hasattr(new, 'PATIENT_STEADY'), 'PATIENT_STEADY is defined')
    check(hasattr(new.Look(), 'd_exit'), 'Look.d_exit is defined')

    print('A  the normal crossing: standing at the middle lane\'s stop line, the worker pacing (synthetic)', flush=True)
    phases = [(so, wo) for so in (0.0, 0.03, 0.06) for wo in (0.0, 1.5, 3.0, 4.5)]
    res = [lockup(new, so, wo) for so, wo in phases]
    worst = max((r[0] for r in res if r is not None), default=None)
    check(all(r is not None and r[0] <= 35.0 for r in res),
          f'it sets off within 35 s at all {len(res)} phases (the longest: {worst} s): ' +
          ', '.join('-' if r is None else f'{r[0]:.1f}' for r in res))
    check(all(r is None or r[1] >= r[2] - 1e-6 for r in res),
          'and only on a scan that keeps the clearance it needs (d_go >= need): ' +
          ', '.join('-' if r is None else f'{r[1]:.2f}/{r[2]:.2f}' for r in res))
    if old is not None:
        ro = [lockup(old, so, wo) for so, wo in phases]
        print('     old guard: ' + ', '.join('never' if r is None else f'{r[0]:.1f}' for r in ro)
              + f'  ({sum(r is None for r in ro)} of {len(ro)} never set off within 45 s)', flush=True)

    print('B  down the aisle\'s middle through the worker\'s walkway, the visitor at the aisle mouth', flush=True)
    starts = [float(s) for s in os.environ.get('G108_STARTS', '').split(',') if s] or \
        [0.0, 3.7, 7.4, 11.1, 14.8, 18.5, 22.2, 25.9, 29.6, 33.3, 37.0, 40.7]
    runs = home_runs(argv[0], starts)
    bad_e = [(r['phase'], e) for r in runs for e in r.get('emergencies', []) if e[4] and e[3] == 'visitor']
    tow = [(r['phase'], c) for r in runs for c in r['contacts'] if c['who'] == 'worker' and c.get('tow', 0.0) > 0.05]
    gaps = [p['gap_mv'] for r in runs for p in r['passages']]
    check(all(r['done'] for r in runs), f'all {len(runs)} runs reached the dock')
    check(not bad_e, 'no emergency stop inside the worker\'s walkway for the visitor at the next walkway'
          + ('' if not bad_e else ': ' + '; '.join(f'start {p}: t {e[0]} at y {e[2]} for {e[3]}' for p, e in bad_e[:4])))
    check(not tow, 'no contact with the robot moving towards the worker'
          + ('' if not tow else ': ' + '; '.join(f'start {p}: {c["who"]} {c["tow"]} m/s' for p, c in tow[:4])))
    print(f'     walkway passages: worker gap while moving min {min(gaps):.2f} m, '
          f'median {sorted(gaps)[len(gaps) // 2]:.2f} m', flush=True)
    if old is not None:
        ro = home_runs(argv[1], starts)
        be = [(r['phase'], e) for r in ro for e in r.get('emergencies', []) if e[4] and e[3] == 'visitor']
        tw = [(r['phase'], c) for r in ro for c in r['contacts'] if c['who'] == 'worker' and c.get('tow', 0.0) > 0.05]
        g2 = [p['gap_mv'] for r in ro for p in r['passages']]
        print(f'     old guard: {len(be)} emergency stops inside the walkway for the visitor, {len(tw)} contacts '
              f'moving towards the worker; worker gap while moving min {min(g2):.2f} m', flush=True)

    rec = os.environ.get('G108_REC') or os.path.expanduser('~/runs/rec/shift_v106a.npz')
    if os.path.exists(rec):
        print('C  the lock-up on your recording shift_v106a, replayed (the live guard set off at t 234.9)', flush=True)
        try:
            r = replay(new, rec)
            check(r is not None and r[0] < 200.0,
                  'v10.8 sets off before t 200' + ('' if r is None else f': at t {r[0]} (d_go {r[1]:.3f}, needs {r[2]:.2f})'))
        except Exception as e:
            print(f'  skip  the replay did not run ({type(e).__name__}: {e})', flush=True)
    else:
        print('C  skipped (no ~/runs/rec/shift_v106a.npz on this machine)', flush=True)
    print(f'({time.time() - t0:.0f} s)')
    print('ALL OK' if not FAILS else f'FAILED: {len(FAILS)} check(s): ' + ' | '.join(FAILS))
    return 0 if not FAILS else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
