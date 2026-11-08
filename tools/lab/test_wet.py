#!/usr/bin/env python3
"""test_wet.py <wet_floor.py> - the wet-floor guardian (v10.13) on made-up situations, no ROS.

    python3 ~/ros2_ws/src/tools/lab/test_wet.py ~/ros2_ws/src/tools/wet_floor.py

Drying; a sweep in an empty warehouse wets a 0.64 m strip; the worker's walkway, once watched, gets the
water off (and only there); a spot it never saw is not taken as busy; someone about to walk across the
brush turns it off; warnings (ahead, underfoot, not twice, not for someone walking away); the truth count
and 'warned in time'; the exported grids; habits saved and loaded; cleaning again just after a pause keeps
the reason; speed. Ends with ALL OK.
"""
import importlib.util
import math
import os
import sys
import tempfile
import time

FAILS = []


def check(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what, flush=True)
    if not cond:
        FAILS.append(what)


def load(path):
    spec = importlib.util.spec_from_file_location('wet_floor_t', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


MAP = (299, 398, 0.05, -3.99, -3.94)          # the warehouse map's grid


def circle(x, y, r=8.0, n=180):
    return {'o': [x, y], 'r': r, 'pts': [[x + r * math.cos(2 * math.pi * i / n), y + r * math.sin(2 * math.pi * i / n)]
                                         for i in range(n)]}


def worker(t):                                # the world's worker: aisle 1 at y 4.0, x -1.6..0.6, 6 s loop
    wp = [(0.0, -1.6), (2.2, 0.6), (3.0, 0.6), (5.2, -1.6), (6.0, -1.6)]
    p = t % 6.0
    for (t0, x0), (t1, x1) in zip(wp, wp[1:]):
        if t0 <= p <= t1:
            return x0 + (p - t0) / (t1 - t0) * (x1 - x0), 4.0, (x1 - x0) / (t1 - t0)
    return -1.6, 4.0, 0.0


def person(i, x, y, vx=0.0, vy=0.0, fresh=True):
    return {'id': 'p%d' % i, 'x': x, 'y': y, 'fresh': fresh,
            'preds': [[[x + vx * 0.1 * k, y + vy * 0.1 * k] for k in range(26)]] if (vx or vy) else []}


def sweep(W, x, y0, y1, t0, v=0.5, people=lambda t: [], view=None, dt=0.1):
    t, y = t0, y0
    pumps = []
    while (y1 - y) * (1 if y1 > y0 else -1) > 0:
        W.step(t, x, y, True, people(t), view(t, x, y) if view else None)
        pumps.append((round(y, 2), W.pump, W.why))
        y += v * dt * (1 if y1 > y0 else -1)
        t += dt
    return t, pumps


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    M = load(os.path.abspath(os.path.expanduser(argv[0])))
    print(f'test_wet: {argv[0]} ({M.VERSION})')

    print('A  drying')
    W = M.WetFloor(*MAP)
    W.step(0.0, 0.0, 5.0, True, [], None)
    W.step(0.1, 0.0, 5.05, True, [], None)
    check(abs(W.wetness(0.1, 0.0, 5.0) - 1.0) < 0.01 and abs(W.wetness(60.1, 0.0, 5.0) - 0.5) < 0.01
          and W.wetness(120.2, 0.0, 5.0) == 0.0, f'wet when cleaned, half dry after {M.T_DRY / 2:.0f} s, dry after {M.T_DRY:.0f} s')

    print('B  an empty warehouse')
    W = M.WetFloor(*MAP)
    t, pumps = sweep(W, 0.0, 2.0, 10.0, 0.0, view=lambda t, x, y: circle(x, y))
    a = W.area(t)
    check(all(p == 'on' for _, p, _ in pumps), 'the water stays on with nobody about')
    check(4.8 <= a['wet_m2'] <= 6.2 and a['dry_m2'] == 0.0, f"an 8 m lane wets a 0.64 m strip: {a['wet_m2']} m2 wet, {a['dry_m2']} m2 dry")
    W.step(t + 1, 0.0, 10.0, False, [], None)
    check(W.pump == 'idle', 'not cleaning: the water is idle')

    print("C  the worker's walkway, once watched")
    W = M.WetFloor(*MAP)
    t = 0.0
    while t < 120.0:                            # the robot stands at (0, 1) watching her for 2 minutes
        wx, wy, wv = worker(t)
        W.step(t, 0.0, 1.0, False, [person(1, wx, wy, wv, 0.0)], circle(0.0, 1.0))
        t += 0.1
    rate = W.P / (W.O + M.O_MIN)
    r, c = W.hcell(-0.5, 4.0)
    r2, c2 = W.hcell(-0.5, 7.0)
    check(rate[r, c] > 0.1 and rate[r2, c2] == 0.0, f'learned: at her walkway {rate[r, c]:.2f} of the time, 3 m away {rate[r2, c2]:.2f}')
    t, pumps = sweep(W, -0.5, 2.0, 7.0, t, view=lambda t, x, y: circle(x, y))    # she has gone (no people)
    off = [y for y, p, _ in pumps if p == 'off']
    check(off and 3.2 <= min(off) and max(off) <= 5.2 and len([1 for y, p, _ in pumps if p == 'on']) > 50,
          f'water off only across her walkway (and {M.OFF_HOLD:.0f} s after): y {min(off) if off else None}..{max(off) if off else None}')
    ev = [e for e in W.take_events() if e[0] == 'water_off']
    check(ev and 'times a minute' in ev[0][1], f"and it says why: {ev[0][1] if ev else None}")
    wet, hab = W.grids(t)
    r, c = W.cell(-0.5, 4.0)
    r3, c3 = W.cell(-0.5, 6.0)
    check(wet[r, c] == M.DRY_PENDING and 1 <= wet[r3, c3] <= 100, 'her walkway strip is marked cleaned dry, the rest wet')
    check(W.finish_text() and 'wet pass' in W.finish_text(), f'and it says it needs a wet pass: {W.finish_text()}')

    print('D  a spot it never saw')
    W = M.WetFloor(*MAP)
    t = 0.0
    while t < 120.0:                            # the same walk, but the robot's view stops at 1 m
        wx, wy, wv = worker(t)
        W.step(t, 0.0, 1.0, False, [person(1, wx, wy, wv, 0.0)], circle(0.0, 1.0, r=1.0))
        t += 0.1
    t, pumps = sweep(W, -0.5, 2.0, 7.0, t, view=lambda t, x, y: circle(x, y, r=1.0))
    check(all(p == 'on' for _, p, _ in pumps), 'never seen there: not taken as busy (the water stays on)')

    print('E  someone about to walk across the brush')
    W = M.WetFloor(*MAP)

    def walker(t):                              # from (2.5, 6.0) towards -x at 1 m/s, from t 4 s
        if t < 4.0:
            return []
        x = 2.5 - (t - 4.0)
        return [person(2, x, 6.0, -1.0, 0.0)] if x > -2.0 else []
    t, pumps = sweep(W, 0.0, 2.0, 9.0, 0.0, people=walker, view=lambda t, x, y: circle(x, y))
    off = [(y, w) for y, p, w in pumps if p == 'off']
    check(off and all(4.4 <= y <= 7.6 for y, _ in off) and any('about to walk' in w or 'right here' in w for _, w in off),
          f'off while they cross ({off[0][0] if off else None}..{off[-1][0] if off else None}): {off[0][1] if off else None}')

    print('F  warnings')
    W = M.WetFloor(*MAP)
    t, _ = sweep(W, 0.0, 2.0, 10.0, 0.0, view=lambda t, x, y: circle(x, y))
    W.take_events()
    t = 20.0
    W.step(t, 0.0, 10.0, False, [person(3, 1.2, 6.0, -1.0, 0.0)], circle(0.0, 10.0))
    ev = [e for e in W.take_events() if e[0] == 'warn']
    check(len(ev) == 1 and 'ahead' in ev[0][1], f"walking towards the wet strip: warned ({ev[0][1] if ev else None})")
    W.step(t + 1.0, 0.0, 10.0, False, [person(3, 1.0, 6.0, -1.0, 0.0)], circle(0.0, 10.0))
    check(not [e for e in W.take_events() if e[0] == 'warn'], 'not twice in a row')
    W.step(t + 7.0, 0.0, 10.0, False, [person(4, 1.0, 3.0, 1.0, 0.0)], circle(0.0, 10.0))
    check(not [e for e in W.take_events() if e[0] == 'warn'], 'not for someone walking away from it')
    W.step(t + 14.0, 0.0, 10.0, False, [person(5, 0.05, 8.0)], circle(0.0, 10.0))
    ev = [e for e in W.take_events() if e[0] == 'warn']
    check(len(ev) == 1 and 'on wet floor' in ev[0][1], f"standing on it: warned ({ev[0][1] if ev else None})")
    A = M.WetFloor(*MAP, policy='always')
    sweep(A, 0.0, 2.0, 10.0, 0.0, people=walker, view=lambda t, x, y: circle(x, y))
    A.step(20.0, 0.0, 10.0, False, [person(3, 1.2, 6.0, -1.0, 0.0)], circle(0.0, 10.0))
    check(A.n_warned == 0 and all(e[0] != 'warn' for e in A.take_events()), "'always' (today's scrubbers): water always on, no warnings")

    print('G  the truth count')
    W = M.WetFloor(*MAP)
    t, _ = sweep(W, 0.0, 2.0, 10.0, 0.0, view=lambda t, x, y: circle(x, y))
    t = 20.0
    for k in range(40):                         # someone crosses the strip at y 6, 1 m/s, from x 2 to -2
        x = 2.0 - k * 0.1
        if k == 5:
            W.step(t, 0.0, 10.0, False, [person(6, x, 6.0, -1.0, 0.0)], circle(0.0, 10.0))
        W.measure(t, [{'name': 'visitor', 'kind': 'person', 'x': x, 'y': 6.0}])
        t += 0.1
    st = W.truth_stats
    check(st['entries'] == 1 and st['warned'] == 1 and 0.4 <= st['seconds'] <= 1.2,
          f"one step onto wet floor, warned in time, {st['seconds']:.1f} s on it")
    W.measure(t + 200.0, [{'name': 'visitor', 'kind': 'person', 'x': 0.0, 'y': 6.0}])
    check(W.truth_stats['entries'] == 1, 'dry again later: not counted')

    print('H  the exported grids and the summary')
    W = M.WetFloor(*MAP)
    t, _ = sweep(W, 0.0, 2.0, 6.0, 0.0, view=lambda t, x, y: circle(x, y))
    wet, hab = W.grids(t)
    vals = set(int(v) for v in set(wet.ravel().tolist()))
    check(wet.dtype.name == 'uint8' and hab.dtype.name == 'uint8' and wet.shape == (W.H, W.W) and hab.shape == (W.HH, W.HW)
          and vals <= set(range(0, 101)) | {M.DRY_PENDING}, f'uint8, {wet.shape} and {hab.shape}, values 0-100 (and {M.DRY_PENDING})')
    s = W.summary(t, truth=True)
    check(set(s) >= {'wet_m2', 'dry_m2', 'cleaned_m2', 'pump', 'why', 'warned', 'truth', 'v', 't_dry'}, 'the summary has what the page needs')

    print('I  habits kept between shifts')
    W = M.WetFloor(*MAP)
    t = 0.0
    while t < 60.0:
        wx, wy, wv = worker(t)
        W.step(t, 0.0, 1.0, False, [person(1, wx, wy, wv, 0.0)], circle(0.0, 1.0))
        t += 0.1
    path = os.path.join(tempfile.mkdtemp(), 'habits.npz')
    W.save(path)
    W2 = M.WetFloor(*MAP)
    W3 = M.WetFloor(200, 200, 0.05, -3.99, -3.94)
    check(W2.load(path) and abs(W2.P.sum() - W.P.sum()) < 1e-6 and not W3.load(path), 'saved and loaded; another map refuses them')
    W2.step(0.0, -0.5, 3.0, True, [], circle(-0.5, 3.0))
    t2, pumps = sweep(W2, -0.5, 3.0, 5.0, 0.1, view=lambda t, x, y: circle(x, y))
    check(any(p == 'off' for _, p, _ in pumps), 'a new shift remembers her walkway from the first pass')

    print('J  the replay restarting')
    W = M.WetFloor(*MAP)
    sweep(W, 0.0, 2.0, 6.0, 100.0, view=lambda t, x, y: circle(x, y))
    W.step(10.0, 0.0, 2.0, False, [], None)
    check(W.area(10.0)['wet_m2'] == 0.0 and W.area(10.0)['cleaned_m2'] == 0.0, 'time going back: a clean floor again')

    print('K  shapes and the scrub deck')
    sh = [{'ns': 'person', 'id': 3, 'k': 'c', 'x': 1.0, 'y': 2.0, 'rgba': [1.0, 0.55, 0.1, 0.5]},
          {'ns': 'person', 'id': 4, 'k': 'c', 'x': 2.0, 'y': 2.0, 'rgba': [0.6, 0.6, 0.6, 0.4]},
          {'ns': 'predicted_path', 'id': 30, 'k': 'l', 'pts': [[1.0, 2.0], [1.1, 2.0]]},
          {'ns': 'standing', 'id': 0, 'k': 'c', 'x': 5.0, 'y': 5.0, 'rgba': [0.15, 0.85, 0.35, 0.6]},
          {'ns': 'walkway', 'id': 3, 'k': 'l', 'pts': [[0, 0], [1, 1]]}]
    pp = {p['id']: p for p in M.people_from_shapes(sh)}
    check(pp['p3']['fresh'] and pp['p3']['preds'] and not pp['p4']['fresh'] and pp['s0']['fresh'] and len(pp) == 3,
          'people from the guard: tracked, lost (grey), standing; predictions attached')
    Z = {'aisle_1': (-1.94, 0.96, 0.80, 14.40)}
    check(M.cleaning_now({'zone': 'aisle_1'}, Z, 0.0, 5.0, 0.4) and not M.cleaning_now({'zone': 'aisle_1'}, Z, 3.0, 5.0, 0.4)
          and not M.cleaning_now({'zone': 'aisle_1', 'paused': True}, Z, 0.0, 5.0, 0.4)
          and not M.cleaning_now({'zone': 'aisle_1'}, Z, 0.0, 5.0, 0.0)
          and not M.cleaning_now({'zone': 'start', 'awaiting_command': True}, Z, 0.0, 5.0, 0.4),
          'the deck is down only working in its zone, moving, job not paused')

    print('L  cleaning again just after a pause')
    W = M.WetFloor(*MAP)
    W.step(0.0, 0.0, 5.0, True, [person(5, 0.3, 5.0)], circle(0.0, 5.0))      # someone at the brush: off
    W.step(0.1, 0.0, 5.05, False, [], circle(0.0, 5.05))                       # a pause (a turn): idle
    W.step(0.2, 0.0, 5.1, True, [], circle(0.0, 5.1))                          # cleaning again inside the hold
    ev = [e[1] for e in W.take_events() if e[0] == 'water_off']
    check(W.pump == 'off' and W.why == 'someone is right here' and len(ev) == 2 and all(ev),
          f'still off for the same reason, and it says which: {ev}')

    print('M  speed')
    W = M.WetFloor(*MAP)
    ppl = [person(i, -1.0 + i, 4.0 + 0.3 * i, 0.5, 0.2) for i in range(5)]
    t0 = time.perf_counter()
    n = 0
    t = 0.0
    while n < 600:
        W.step(t, 0.0, 2.0 + 0.05 * n % 8, True, ppl, circle(0.0, 2.0))
        t += 0.1
        n += 1
    ms = (time.perf_counter() - t0) / n * 1000
    check(ms < 4.0, f'a step with 5 people and the view: {ms:.2f} ms (10 a second is plenty)')

    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
    return 0 if not FAILS else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
