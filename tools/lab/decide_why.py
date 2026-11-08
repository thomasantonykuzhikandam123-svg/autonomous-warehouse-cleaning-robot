#!/usr/bin/env python3
"""decide_why.py (v10.3) - replay a recording through two guard versions side by side.

    python3 decide_why.py <label> <old guard.py> <new guard.py> [t,t,...] [window_s]

Feeds every recorded scan (with the poses, /odom speed and /plan the live guard had) through both
guards' GuardCore and prints, around each time given (default: tour_why's close calls), what each
would have decided: state/why, the speed limit sent to Nav2 and, for v10.3, where the limit came
from. Open loop: the robot moves as it did in the run (the live guard's decisions), so after the
first scan where the two disagree, the new guard's lines say what it would have done at that
moment, not what would have followed. The camera's persons are not recorded, so neither replay
sees camera-confirmed standing people (the live guard did). Then, over the whole run: the share of
scans with a vehicle speed cap, holds by reason, and how often the new guard would cap or hold.
"""
import importlib.util
import json
import math
import os
import sys

import numpy as np

RUNS = os.path.expanduser('~/runs')


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def read_csv(path):
    import csv
    try:
        with open(path, newline='') as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def fnum(x, d=float('nan')):
    try:
        return float(x)
    except (TypeError, ValueError):
        return d


def main(argv):
    label, old_path, new_path = argv[:3]
    times = [float(x) for x in argv[3].split(',')] if len(argv) > 3 and argv[3] else \
        [61.6, 92.0, 152.7, 214.3, 335.3, 351.3]
    win = float(argv[4]) if len(argv) > 4 else 3.0
    R = np.load(os.path.join(RUNS, 'rec', f'{label}.npz'), allow_pickle=False)
    meta = json.loads(str(R['meta']))
    mi = json.loads(str(R['map_info']))
    T, RG, LS, RB, OV, OA = R['t'], R['ranges'], R['laser'], R['robot'], R['odom_v'], R['odom_age']
    amin, ainc = float(meta['angle_min']), float(meta['angle_increment'])
    rmin, rmax = float(meta['range_min']), float(meta['range_max'])
    plans = []
    if 'plan_t' in R.files:
        off = np.concatenate(([0], np.cumsum(R['plan_n'])))
        plans = [(float(R['plan_t'][k]), R['plan_xy'][off[k]:off[k + 1]]) for k in range(len(R['plan_t']))]
    plan_ts = np.array([p[0] for p in plans]) if plans else np.zeros(0)
    glog = {}
    for r in read_csv(os.path.join(RUNS, 'guard', f'{label}.csv')):
        tt = fnum(r.get('t'))
        if tt == tt:
            glog[round(tt, 2)] = r
    mods = [load(old_path, 'g_old'), load(new_path, 'g_new')]
    cores = [m.GuardCore(m.StaticMap(mi['width'], mi['height'], mi['resolution'], mi['origin_x'],
                                     mi['origin_y'], R['map_data'])) for m in mods]
    names = [getattr(m, 'VERSION', '?') for m in mods]
    stats = [dict(scans=0, veh_cap=0, wait={}, limit=0, prox=0, start=0) for _ in mods]
    lines = {t0: [] for t0 in times}
    prev, v_est = None, 0.0
    for k in range(len(T)):
        t = float(T[k])
        laser, robot = tuple(LS[k]), tuple(RB[k])
        plan = None
        if len(plan_ts):
            i = int(np.searchsorted(plan_ts, t, side='right')) - 1
            if i >= 0 and t - plan_ts[i] <= mods[0].PLAN_AGE:
                plan = [tuple(p) for p in plans[i][1]]
        if OA[k] < 0.5:
            v_est = float(OV[k])
        elif prev is not None and t > prev[0]:
            v_est = 0.5 * v_est + 0.5 * math.hypot(robot[0] - prev[1], robot[1] - prev[2]) / (t - prev[0])
        prev = (t, robot[0], robot[1])
        g = glog.get(round(t, 2))
        cruise = fnum(g.get('cruise'), 0.8) if g is not None else 0.8
        cruise = cruise if cruise == cruise else 0.8
        out = []
        for m, core, st in zip(mods, cores, stats):
            core.scan(RG[k], amin, ainc, rmin, rmax, laser, t, robot, plan)
            state, _, _ = core.decide(robot[:2], robot[2], v_est, cruise, t, plan)
            gd = core.guard
            st['scans'] += 1
            st['veh_cap'] += int(getattr(core, 'veh_cap', None) is not None)
            st['limit'] += int(gd.v_limit is not None and gd.v_limit < cruise - 0.02)
            st['prox'] += int(getattr(core, 'prox', None) is not None)
            st['start'] += int(state == 'wait' and gd.why == 'start')
            if state == 'wait':
                why = 'vehicle' if getattr(core, 'veh_hold', False) else gd.why
                st['wait'][why] = st['wait'].get(why, 0) + 1
            src = []
            if getattr(core, 'veh_cap', None) is not None:
                src.append(f'veh {core.veh_cap:.2f}')
            if getattr(core, 'cap', None) is not None:
                src.append(f'person {core.cap:.2f}')
            if getattr(core, 'prox', None) is not None:
                src.append(f'prox {core.prox:.2f}')
            why = 'vehicle' if state == 'wait' and getattr(core, 'veh_hold', False) else gd.why
            lim = '-' if gd.v_limit is None else f'{gd.v_limit:.2f}'
            out.append(f'{state:4s} {why:9s} {lim:>4s} {" ".join(src):24s}')
        for t0 in times:
            if abs(t - t0) <= win:
                live = '' if g is None else f'{g.get("state", "")}/{g.get("why", "")}'
                lines[t0].append(f'  {t:6.1f} {robot[0]:6.2f} {robot[1]:6.2f} {v_est:4.2f} | {live:15s} | '
                                 + ' | '.join(out))
    print(f'=== decide_why: {label} replayed through {names[0]} ({old_path}) and {names[1]} ({new_path})')
    print('  open loop: the robot moves as it did; no camera in the replay')
    for t0 in times:
        print(f'\n--- around t {t0:.1f}')
        print(f'  {"t":>6s} {"x":>6s} {"y":>6s} {"v":>4s} | {"live guard":15s} | {names[0] + " state why lim (caps)":44s} | '
              f'{names[1] + " state why lim (caps)"}')
        for ln in lines[t0][::2]:
            print(ln)
    print('\n--- over the whole run (share of scans)')
    for nm, st in zip(names, stats):
        n = max(st['scans'], 1)
        w = ', '.join(f'{k} {100.0 * v / n:.1f} %' for k, v in sorted(st['wait'].items(), key=lambda kv: -kv[1]))
        print(f'  {nm}: vehicle cap {100.0 * st["veh_cap"] / n:.0f} %, any speed limit below cruise '
              f'{100.0 * st["limit"] / n:.0f} %, proximity cap {100.0 * st["prox"] / n:.1f} %; WAIT: {w}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
