#!/usr/bin/env python3
"""picker_report.py (v8.2) - the standing-picker test, compared across runs.

    python3 ~/ros2_ws/src/tools/picker_report.py <label> [<label> ...]
    e.g.  picker_report.py picker_base picker_cam

For each run: how the robot passed the standing picker and the look-alike post on the way up
aisle 2 (closest centre-to-centre distance, speed there, top speed within 1 m), and the far lane
on the way down as a control; whether the camera found her and whether it ever called the post a
person (from eye.py's log); when the guard first treated her as a standing person (guard log,
v9). Reads ~/runs/picker_trials.csv, ~/runs/eye/<label>.csv, ~/runs/guard/<label>.csv.
Nothing is changed or written.
"""
import csv
import math
import os
import statistics
import sys

RUNS = os.path.expanduser('~/runs')


def fnum(v, default=float('nan')):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def read(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='', errors='replace') as f:
        return list(csv.DictReader(f))


def med(xs):
    xs = [x for x in xs if x == x]
    return statistics.median(xs) if xs else float('nan')


def mx(xs):
    xs = [x for x in xs if x == x]
    return max(xs) if xs else float('nan')


def mn(xs):
    xs = [x for x in xs if x == x]
    return min(xs) if xs else float('nan')


def passes_table(rows, labels):
    print('--- passing them (centre-to-centre; the robot is 0.6 x 0.4 m, so 0.56 m is touching)')
    print(f'{"run":14s} {"way":5s} {"object":7s} {"n":>2s} {"closest med":>11s} {"min":>5s} {"speed there":>11s} '
          f'{"top <1 m":>8s} {"top <1.5 m":>10s} {"s <1.5 m":>8s} {"pass s":>6s}')
    for lab in labels:
        rs = [r for r in rows if r.get('label') == lab]
        if not rs:
            print(f'{lab:14s} (no rows in picker_trials.csv)')
            continue
        for way in ('up', 'down'):
            for obj in sorted({r['object'] for r in rs}, key=lambda o: (o != 'picker', o)):
                sel = [r for r in rs if r['dir'] == way and r['object'] == obj]
                if not sel:
                    continue
                cl = [fnum(r['closest_m']) for r in sel]
                print(f'{lab:14s} {way:5s} {obj:7s} {len(sel):2d} {med(cl):11.2f} {mn(cl):5.2f} '
                      f'{med([fnum(r["v_at_closest"]) for r in sel]):11.2f} '
                      f'{mx([fnum(r["vmax_1m"]) for r in sel]):8.2f} {med([fnum(r["vmax_1_5m"]) for r in sel]):10.2f} '
                      f'{med([fnum(r["s_within_1_5m"]) for r in sel]):8.1f} '
                      f'{med([fnum(r["duration_s"]) for r in sel]):6.1f}')
        bad = [r for r in rs if r['result'] != 'SUCCEEDED']
        if bad:
            print(f'{"":14s} passes that did not succeed: ' + ', '.join(f'{r["pass"]} {r["result"]}' for r in bad[::2]))
    print()


def eye_table(labels):
    print('--- the camera (eye.py): frames with the object in view, 0.8-8 m')
    for lab in labels:
        rows = [r for r in read(os.path.join(RUNS, 'eye', f'{lab}.csv')) if r.get('ok') == '1']
        if not rows:
            print(f'  {lab:14s} no camera log with detector answers')
            continue
        out = []
        for obj, word in (('picker', 'found her'), ('post', 'called it a person')):
            if f'{obj}_view' not in rows[0]:
                continue
            vis = [r for r in rows if r.get(f'{obj}_view') == '1']
            hit = [r for r in vis if r.get(f'{obj}_hit') == '1']
            near = [r for r in vis if fnum(r.get(f'{obj}_range')) < 4.0]
            hn = [r for r in near if r.get(f'{obj}_hit') == '1']
            out.append(f'{obj}: in view {len(vis)} frames, {word} in {100 * len(hit) / len(vis) if vis else 0:.0f} %'
                       f' ({100 * len(hn) / len(near) if near else 0:.0f} % within 4 m)')
        print(f'  {lab:14s} ' + ('; '.join(out) if out else 'no scene columns (eye.py before v8.2?)'))
    print()


def guard_table(labels):
    print('--- the guard (v9): standing people the camera confirmed')
    for lab in labels:
        rows = read(os.path.join(RUNS, 'guard', f'{lab}.csv'))
        if not rows or 'standing' not in rows[0]:
            print(f'  {lab:14s} no v9 guard log')
            continue
        st = [r for r in rows if fnum(r.get('standing'), 0) > 0]
        caps = [fnum(r.get('cap')) for r in rows if r.get('cap')]
        if not st:
            print(f'  {lab:14s} never (camera off, or nobody confirmed standing)')
            continue
        first = st[0]
        print(f'  {lab:14s} in {len(st)} of {len(rows)} scans; first at {fnum(first["t"]):.1f} s, '
              f'{fnum(first["stand_d"]):.1f} m from the robot, personal space {fnum(first["stand_r"]):.2f} m; '
              f'speed cap down to {mn(caps):.2f} m/s')
    print()


def main(argv=None):
    labels = sys.argv[1:] if argv is None else argv
    if not labels:
        print('usage: picker_report.py <label> [<label> ...]')
        return 1
    rows = read(os.path.join(RUNS, 'picker_trials.csv'))
    print(f'=== picker_report: {", ".join(labels)}\n')
    passes_table(rows, labels)
    eye_table(labels)
    guard_table(labels)
    return 0


if __name__ == '__main__':
    sys.exit(main())
