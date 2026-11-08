#!/usr/bin/env python3
"""test_rewind.py <rewind.py> [people_guard.py] - Rewind (v10.14) on made-up situations, no ROS.

    python3 ~/ros2_ws/src/tools/lab/test_rewind.py ~/ros2_ws/src/tools/rewind.py

A person crossing just ahead: the stop is judged 'would have hit', with who, when and how fast; someone far
off: 'clear'; a near miss and a tight pass; the robot's own tracks when there is no simulator truth (and both
when there is); a blink of GO inside one stop; waiting for a vehicle is not a stop; time going back; the
recorded guard log of a shift (replay) and no spoilers; what the page gets (listing, detail, summary, the
closest call, the scorecard, the sentences); the real outcome: walked into a standing robot, or a robot
creeping into them; people from the guard's markers; speed. Ends with ALL OK.
"""
import csv
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


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def crossing(t0, x=1.6, y0=0.0, speed=1.0, at=None):
    """A person walking up the line x = x, at y0 at time t0 (+y at speed m/s); `at` = when they reach y 2.0."""
    if at is not None:
        y0 = 2.0 - speed * at
    return lambda t: (x, y0 + speed * (t - t0))


def run(RW, pg, people, t0=100.0, robot=(0.0, 2.0, 0.0), v=0.8, why='yield', truth=True, own=False, wait_s=6.0,
        go_blink=None, horizon=None, guard_extra=None):
    """The robot at `robot` heading +x along a straight plan; the guard says WAIT (why) from t0 for wait_s s
    (with an optional blink of GO (t_from, t_to)); people: {name: f(t) -> (x, y)}. 10 Hz for 6 s."""
    rw = RW.Rewind(pg)
    plan = [(robot[0] + 0.1 * i, robot[1]) for i in range(80)]
    t = t0
    while t < t0 + 6.01:
        waiting = t0 <= t < t0 + wait_s and not (go_blink and go_blink[0] <= t - t0 < go_blink[1])
        g = dict(state='wait' if waiting else 'go', why=why if waiting else 'clear', say='waiting', person_m=1.5,
                 v_limit=None, **(guard_extra or {}))
        tr = [{'name': n, 'kind': 'person', 'x': f(t)[0], 'y': f(t)[1]} for n, f in people.items()] if truth else None
        ow = [('p%d' % i, f(t)[0], f(t)[1]) for i, f in enumerate(people.values())] if own else None
        rw.step(t, robot, v if t <= t0 else 0.0, plan, g, tr, ow)
        t = round(t + 0.1, 3)
    return rw


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    path = os.path.abspath(os.path.expanduser(argv[0]))
    RW = load(path, 'rewind_t')
    pg = load(argv[1] if len(argv) > 1 else os.path.join(os.path.dirname(path), 'people_guard.py'), 'pg_t')
    print(f'test_rewind: {argv[0]} ({RW.VERSION}), guard {pg.VERSION}')

    print('A  the classes')
    check([RW.classify(g) for g in (0.0, 0.1, 0.2, 0.35, 0.5, 0.51, 3.0)] == ['hit', 'near', 'near', 'tight', 'tight',
                                                                          'clear', 'clear'],
          'would have hit 0, within 20 cm, tight to 0.50 m, clear beyond')

    print('B  a person crossing just ahead')
    rw = run(RW, pg, {'worker': crossing(100.0, at=1.3)})       # its nose reaches her line at about 1.3 s
    st = rw.stops[0] if rw.stops else {}
    check(len(rw.stops) == 1 and st.get('cls') == 'hit' and st.get('who') == 'worker' and st.get('src') == 'truth',
          f"one stop, judged {st.get('cls')} ({st.get('who')}, from {st.get('src')})")
    check(st.get('at') is not None and 0.9 <= st['at'] <= 1.7 and abs(st['v_at'] - 0.8) < 0.05 and st['gap'] == 0.0,
          f"when and how fast: {st.get('at')} s after it stopped, at {st.get('v_at')} m/s")
    check('would have hit the worker' in (st.get('text') or ''), f"said: {st.get('text')}")
    check(st.get('waited') is not None and 5.5 <= st['waited'] <= 6.0, f"it waited {st.get('waited')} s")
    ev = rw.take_events()
    check(len(ev) == 1 and ev[0]['id'] == st['id'] and rw.take_events() == [], 'announced once, when its 4 s were up')

    print('C  someone far off, a near miss, a tight pass')
    rw = run(RW, pg, {'visitor': crossing(100.0, x=6.0, at=8.0)})
    check(rw.stops and rw.stops[0]['cls'] == 'clear' and rw.stops[0]['gap'] > 0.5 and "didn't need" in rw.stops[0]['text'],
          f"far off: clear, {rw.stops[0]['gap'] if rw.stops else None} m: {rw.stops[0]['text'] if rw.stops else ''}")
    # the robot's body reaches x 1.6 + 0.25 + 0.3 = 2.15 at about 2.9 s: a person passing just after it
    found = {}
    for lead in [x * 0.05 for x in range(0, 40)]:
        r = run(RW, pg, {'picker_a': crossing(100.0, x=2.6, at=2.9 + lead)})
        c = r.stops[0]['cls'] if r.stops else None
        found.setdefault(c, (lead, r.stops[0]['gap'] if r.stops else None))
    check('near' in found and 'tight' in found and 'hit' in found,
          'as they cross later and later: hit, then within 20 cm, then tight: ' +
          ', '.join(f'{k} ({v[1]:.2f} m)' for k, v in found.items() if k))

    print('D  its own tracks, with no simulator')
    rw = run(RW, pg, {'worker': crossing(100.0, at=1.3)}, truth=False, own=True)
    st = rw.stops[0] if rw.stops else {}
    check(st.get('cls') == 'hit' and st.get('src') == 'own' and st.get('own_cls') is None,
          f"from what it saw afterwards: {st.get('cls')} (src {st.get('src')}): {st.get('text')}")
    rw = run(RW, pg, {'worker': crossing(100.0, at=1.3)}, truth=True, own=True)
    st = rw.stops[0] if rw.stops else {}
    check(st.get('src') == 'truth' and st.get('own_cls') == 'hit', 'with both: the truth decides, its own verdict is kept')

    print('E  one stop or two')
    rw = run(RW, pg, {'worker': crossing(100.0, at=1.0)}, go_blink=(1.0, 1.5))
    check(len(rw.stops) == 1, f'a blink of GO (0.5 s) inside it: still one stop ({len(rw.stops)})')
    rw = run(RW, pg, {'worker': crossing(100.0, at=1.0)}, go_blink=(1.0, 3.0))
    check(len(rw.stops) == 2, f'GO for 2 s: two stops ({len(rw.stops)})')
    rw = run(RW, pg, {'worker': crossing(100.0, at=1.0)}, why='vehicle')
    check(rw.stops == [], 'waiting for a vehicle is not a stop for a person')

    print('F  time going back')
    rw = run(RW, pg, {'worker': crossing(100.0, at=1.0)})
    rw.step(50.0, (0.0, 2.0, 0.0), 0.0, None, {'state': 'go', 'why': 'clear'}, [], [])
    check(rw.stops == [] and rw.pending == [], 'a replay starting again: no stops')

    print('G  a recorded shift (the guard log, the truth, the robot)')
    tmp = tempfile.mkdtemp()
    gcsv = os.path.join(tmp, 'guard.csv')
    with open(gcsv, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'state', 'why', 'd_go', 'robot_x', 'robot_y', 'robot_yaw', 'robot_v', 'v_limit', 'p_x', 'p_y'])
        for k in range(0, 300):
            t = 20.0 + 0.1 * k
            wait = 30.0 <= t < 36.0 or 40.0 <= t < 40.4 or 41.0 <= t < 43.0
            w.writerow([f'{t:.2f}', 'wait' if wait else 'go', 'yield' if wait else 'clear', '0.10', '0.0', '2.0', '0.0',
                        '0.8' if t <= 30.0 else '0.0', '', '1.6', '1.0'])
    cross = crossing(30.0, at=1.3)
    ts = [20.0 + 0.1 * k for k in range(300)]
    frames = [[{'name': 'worker', 'kind': 'person', 'x': cross(t)[0], 'y': cross(t)[1]}] for t in ts]
    rec_t = ts
    rec_rb = [(0.0, 2.0, 0.0) for _ in ts]
    rw = RW.Rewind(pg)
    n = rw.from_record(gcsv, lambda t: [(0.1 * i, 2.0) for i in range(80)], ts, frames,
                       lambda t: {'name': 'far', 'kind': 'person', 'x': 50.0, 'y': 50.0}, rec_t, rec_rb)
    check(n == 2 and rw.stops[0]['cls'] == 'hit' and rw.stops[0].get('pred') == 0.1,
          f"two stops (0.4 s apart is a new one only after more than {RW.MERGE} s: {n}); the first would have hit")
    check(len(rw.visible(33.0)) == 0 and len(rw.visible(34.1)) == 1 and len(rw.visible(60.0)) == 2,
          'shown only once played (no spoilers): none at 33 s, one at 34.1 s, both at 60 s')

    print('H  what the page gets')
    rw = run(RW, pg, {'worker': crossing(100.0, at=1.3), 'visitor': crossing(100.0, x=6.0, at=8.0)})
    rw2 = run(RW, pg, {'visitor': crossing(100.0, x=6.0, at=8.0)}, t0=120.0)
    rw.stops += rw2.stops
    rw.stops[-1]['id'] = 2
    lst = rw.listing()
    d = rw.detail(1)
    s = rw.summary()
    check(len(lst) == 2 and {'id', 't', 'cls', 'gap', 'who', 'text', 'real_gap'} <= set(lst[0]),
          'the list: id, time, verdict, gap, who, sentence, the real outcome')
    check(d is not None and len(d['path']) == 41 and set(d['people']) == {'worker', 'visitor'} and len(d['real']) >= 40
          and d['hit_xy'] and d['who_name'] == 'the worker' and d['horizon'] == 4.0,
          'one stop in full: the avoided path (41 steps), where everyone went, the real robot, the moment')
    check(s['n'] == 2 and s['hit'] == 1 and s['clear'] == 1 and s['closest']['id'] == 1 and s['last']['id'] == 2,
          f"the summary: {s['n']} stops, {s['hit']} hit, {s['clear']} clear; the closest call is the hit")
    card = rw.scorecard()
    check('2 times' in card and '1 would have been a collision' in card and 'over-cautious' in card,
          f'the scorecard: {card}')
    check(rw.detail(99) is None and RW.Rewind(pg).scorecard().startswith("I haven't"), 'no such stop; no stops yet')

    print('I  the real outcome')
    still = RW.Tracks()
    for k in range(41):
        still.add(100.0 + 0.1 * k, [('worker', 0.7 - 0.02 * k, 2.0)])          # walks into the robot's nose
    poses = [(100.0 + 0.1 * k, 0.0, 2.0, 0.0) for k in range(41)]
    g, who, v = RW.real_gap(pg, poses, still)
    check(g == 0.0 and who == 'worker' and v < 0.05, f'they walk into it standing still: 0 m, moving towards them {v} m/s')
    stand = RW.Tracks()
    for k in range(41):
        stand.add(100.0 + 0.1 * k, [('picker_a', 0.75, 2.0)])
    creep = [(100.0 + 0.1 * k, 0.15 * 0.1 * k, 2.0, 0.0) for k in range(41)]
    g, who, v = RW.real_gap(pg, creep, stand)
    check(g == 0.0 and 0.1 <= v <= 0.2, f'it creeps into someone standing: 0 m, moving towards them {v} m/s')

    print('J  people from the guard')
    sh = [{'ns': 'person', 'id': 3, 'k': 'c', 'x': 1.0, 'y': 2.0, 'rgba': [1.0, 0.55, 0.1, 0.5]},
          {'ns': 'person', 'id': 4, 'k': 'c', 'x': 2.0, 'y': 2.0, 'rgba': [0.6, 0.6, 0.6, 0.4]},
          {'ns': 'standing', 'id': 0, 'k': 'c', 'x': 5.0, 'y': 5.0, 'rgba': [0.15, 0.85, 0.35, 0.6]},
          {'ns': 'predicted_path', 'id': 30, 'k': 'l', 'pts': [[1.0, 2.0], [1.1, 2.0]]}]
    check(RW.people_from_shapes(sh) == [('p3', 1.0, 2.0), ('s0', 5.0, 5.0)], 'tracked and standing; lost (grey) left out')

    print('K  speed')
    rw = RW.Rewind(pg)
    plan = [(0.1 * i, 2.0) for i in range(80)]
    ppl = {f'p{i}': crossing(100.0, x=1.0 + i, at=1.0 + 0.3 * i) for i in range(5)}
    t0 = time.perf_counter()
    n = 0
    for k in range(600):
        t = 100.0 + 0.1 * k
        waiting = (k // 50) % 2 == 0
        tr = [{'name': nm, 'kind': 'person', 'x': f(t)[0], 'y': f(t)[1]} for nm, f in ppl.items()]
        rw.step(t, (0.0, 2.0, 0.0), 0.5, plan, {'state': 'wait' if waiting else 'go', 'why': 'yield'}, tr,
                [(nm, x, y) for nm, (x, y) in ((nm, f(t)) for nm, f in ppl.items())])
        n += 1
    ms = (time.perf_counter() - t0) / n * 1000
    check(ms < 10.0 and len(rw.stops) == 6, f'a tick with 5 people: {ms:.2f} ms on average, verdicts included ({len(rw.stops)} stops)')

    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
    return 0 if not FAILS else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
