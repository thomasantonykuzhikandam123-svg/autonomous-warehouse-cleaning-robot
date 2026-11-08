#!/usr/bin/env python3
"""test_guard1011.py (v10.11) - the touching-distance hold (people_guard.touch_hold).

    python3 test_guard1011.py <people_guard.py> [<runs folder> [label]]

A  the rule itself, synthetic: someone straight ahead within PROX_GAP0 while driving on closes in on
   them -> hold; beside or behind the robot's middle, moving off to the side, or further than
   PROX_GAP0 -> no hold; once holding, it keeps holding until the gap opens past PROX_GAP0 +
   TOUCH_HYST, and lets go when they step aside.
B  shift_v108a replayed (if the recording is there): from t 98.8, while picker_a crosses the front
   bumper head-on, v10.11 says WAIT 'touch' (v10.8 went on 'committed' at 0.15 m/s), and lets go
   by t 100.
"""
import importlib.util
import json
import math
import os
import sys
import types

import numpy as np

FAILS = []


def check(ok, what):
    print(('  ok   ' if ok else '  FAIL ') + what, flush=True)
    if not ok:
        FAILS.append(what)


def load(path):
    spec = importlib.util.spec_from_file_location('pg_t1011', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def person(i, x, y):
    return types.SimpleNamespace(id=i, px=x, py=y, x=x, y=y, wvx=0.0, wvy=0.0)


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    PG = load(os.path.abspath(os.path.expanduser(argv[0])))
    print(f'test_guard1011: {argv[0]} (VERSION {PG.VERSION})')
    check(hasattr(PG, 'touch_hold') and hasattr(PG, 'TOUCH_CLOSING'), 'touch_hold and TOUCH_CLOSING are defined')
    if FAILS:
        print('1 FAILED: not a v10.11 guard')
        return 1
    print('A  the rule, synthetic (robot at the origin facing +x; its front at x 0.30)')
    th = PG.touch_hold
    front = 0.30 + PG.BODY_R                      # a person's centre this far ahead: body gap 0
    check(th((0, 0), 0.0, [person(1, front + 0.05, 0.0)], [], []) is not None,
          'straight ahead, 0.05 m body to body: hold')
    check(th((0, 0), 0.0, [person(1, front + 0.05, 0.0)], [], [])[2] > 0.9, '... head-on (closing about 1)')
    check(th((0, 0), 0.0, [person(1, front + 0.25, 0.0)], [], []) is None,
          'straight ahead, 0.25 m: no hold (prox_limit slows it instead)')
    check(th((0, 0), 0.0, [person(1, 0.0, 0.2 + PG.BODY_R + 0.05)], [], []) is None,
          'beside its middle, 0.05 m: no hold (driving on clears their way)')
    check(th((0, 0), 0.0, [person(1, -0.40, 0.0)], [], []) is None, 'behind it: no hold')
    check(th((0, 0), 0.0, [], [(front + 0.05, 0.0, 0.6, 7)], []) is not None,
          'a standing person the camera confirmed, straight ahead: hold')
    check(th((0, 0), 0.0, [], [], [person(9, front + 0.05, 0.0)]) is not None,
          'someone the lidar saw walk who now stands still, straight ahead: hold')
    check(th((0, 0), 0.0, [person(1, front + 0.20, 0.0)], [], [], held=1) is not None,
          'holding for them: still holding at 0.20 m (until PROX_GAP0 + TOUCH_HYST)')
    check(th((0, 0), 0.0, [person(1, front + 0.30, 0.0)], [], [], held=1) is None, '... and lets go at 0.30 m')
    check(th((0, 0), 0.0, [person(1, 0.25, 0.2 + PG.BODY_R + 0.02)], [], [], held=1) is None,
          '... and lets go when they step to its side')
    runs = os.path.expanduser(argv[1]) if len(argv) > 1 else os.path.expanduser('~/runs')
    label = argv[2] if len(argv) > 2 else 'shift_v108a'
    rec = os.path.join(runs, 'rec', f'{label}.npz')
    if not os.path.exists(rec):
        print(f'  --   {rec} not here: B skipped')
    else:
        print(f'B  {label} replayed, t 60-101 (the guard warms up on the first 38 s)')
        Z = np.load(rec, allow_pickle=False)
        meta = json.loads(str(Z['meta']))
        mi = json.loads(str(Z['map_info']))
        T, RG, LS, RB, OV, OA = Z['t'], Z['ranges'], Z['laser'], Z['robot'], Z['odom_v'], Z['odom_age']
        off = np.concatenate(([0], np.cumsum(Z['plan_n'])))
        plan_ts = np.asarray(Z['plan_t'], dtype=float)
        core = PG.GuardCore(PG.StaticMap(mi['width'], mi['height'], mi['resolution'], mi['origin_x'],
                                         mi['origin_y'], Z['map_data']))
        core.guard.prog_time = 25.0
        prev, v_est, rows = None, 0.0, []
        for k in range(len(T)):
            t = float(T[k])
            if t < 60.0 or t > 101.0:
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
            state, lk, _ = core.decide(robot[:2], robot[2], v_est, 0.8, t, plan)
            rows.append((t, state, core.touch_wait))
        held = [t for t, s, w in rows if 98.75 <= t <= 99.95 and s == 'wait' and w]
        check(len(held) >= 10, f't 98.8-99.9 (picker_a across the front bumper, head-on): WAIT touch on {len(held)} of '
                               f'{sum(1 for t, s, w in rows if 98.75 <= t <= 99.95)} scans (v10.8: GO committed, 0.15 m/s)')
        check(all(not w for t, s, w in rows if t < 98.7), 'and never before it in this window (t 60-98.7)')
        check(any(s == 'go' for t, s, w in rows if 99.95 < t <= 101.0), 'it lets go once the gap opens (by t 100)')
    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
    return 0 if not FAILS else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
