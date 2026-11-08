#!/usr/bin/env python3
"""tour_report.py (v10.3) - compare tour runs (tour_test.py) side by side; v10.3: and cleaning shifts (tour_test.py <label> watch).

    python3 ~/ros2_ws/src/tools/tour_report.py <label> [<label> ...]

First which code ran (the guard's version, vehicles and camera on or off, from the guard tab's
log ~/runs/logs/guard_<label>.log; v10.2: and traffic.py's version, from the traffic tab's log:
before traffic.py v10.2 the vehicles sank through the floor, out of the lidar's sight). Per
person and vehicle: the closest the robot's body came to theirs (0 = touching; the worst lap and
the typical lap), the robot's speed there, its top speed and time within 1 m, and contacts (with
the robot moving or standing, and whether it was driving towards them). Then the laps (time,
goals, how often the guard held the robot), the vehicles' side (how long they stood waiting for
the robot, and from v10.2 their height: ~/runs/traffic/<label>.csv), the guard's vehicle decisions
(~/runs/guard/<label>.csv) and the emergency-stop test.
"""
import csv
import json
import os
import re
import sys

RUNS = os.path.expanduser('~/runs')
NEAR = 1.0


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='', errors='replace') as f:
        return list(csv.DictReader(f))


def fnum(v, default=float('nan')):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def median(xs):
    xs = sorted(x for x in xs if x == x)
    if not xs:
        return float('nan')
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def f2(v):
    return '   -' if v != v else f'{v:4.2f}'


def code_ran(label):
    """What the guard tab printed when it started: its version, vehicles and camera on or off."""
    p = os.path.join(RUNS, 'logs', f'guard_{label}.log')
    if not os.path.exists(p):
        return f'no ~/runs/logs/guard_{label}.log, so not known (was the stack started with this label?)'
    ver = veh = cam = None
    with open(p, errors='replace') as f:
        for ln in f:
            m = re.search(r'people_guard (v\d+(?:\.\d+)*)', ln)
            if m and ver is None:
                ver = m.group(1)
            if 'people_guard: vehicles ' in ln:
                veh = 'on' if 'vehicles ON' in ln else 'off'
            if 'people_guard: camera ' in ln:
                cam = 'on' if 'camera ON' in ln else 'off'
    out = f'guard {ver or "?"}, vehicles {veh or "?"}, camera {cam or "?"}'
    if cam == 'on':
        out += ', ' + detector(label)
    if ver is not None and tuple(int(x) for x in ver[1:].split('.')) < (10, 1):
        if veh == 'off':
            out += '   (vehicle layer off: v10 and v10.1 behave the same)'
        else:
            out += '   <- older than v10.1: the two layer fixes were not in this run (install_v101.sh)'
    return out


def traffic_ran(label):
    """traffic.py's version (the traffic tab's first line) and whether it held the vehicles up."""
    p = os.path.join(RUNS, 'logs', f'traffic_{label}.log')
    if not os.path.exists(p):
        return 'traffic ? (no traffic tab log)'
    ver = None
    with open(p, errors='replace') as f:
        for ln in f:
            m = re.search(r'traffic (v\d+(?:\.\d+)*):', ln)
            if m:
                ver = m.group(1)
                break
    if ver is None:
        return 'traffic ?'
    if tuple(int(x) for x in ver[1:].split('.')) < (10, 2):
        return f'traffic {ver}: VEHICLES SANK (about 1 cm/s), out of the lidar\'s sight after a minute or two'
    return f'traffic {ver} (vehicles held on the floor)'


def detector(label):
    """Did the eye reach the YOLO detector on the Mac? "camera on" alone only says the guard was
    willing to use it: with the detector down the camera confirms nobody."""
    p = os.path.join(RUNS, 'logs', f'eye_{label}.log')
    if not os.path.exists(p):
        return 'detector not known (no eye log)'
    ever, up, drops = False, False, 0
    with open(p, errors='replace') as f:
        for ln in f:
            if 'eye: detector online' in ln:
                ever, up = True, True
            elif 'eye: detector OFFLINE' in ln:
                drops += int(up)
                up = False
    if not ever:
        return 'detector NEVER ONLINE: the camera confirmed nobody (start eye_server.py on the Mac)'
    out = 'detector online'
    if drops:
        out += f', but it dropped out {drops}x during the run (see the eye tab)'
    return out


def main(argv=None):
    labels = (sys.argv[1:] if argv is None else argv)
    if not labels:
        print(__doc__)
        return 1
    trials = read_csv(os.path.join(RUNS, 'tour_trials.csv'))
    print('=== tour_report: ' + ', '.join(labels))
    print()
    print('--- which code ran (the guard tab\'s log, ~/runs/logs/guard_<label>.log)')
    for lab in labels:
        print(f'  {lab:16s} {code_ran(lab)}')
        print(f'  {"":16s} {traffic_ran(lab)}')
    agents = []
    for r in trials:
        if r['label'] in labels and r['agent'] not in agents:
            agents.append(r['agent'])
    print()
    print('--- body to body (the robot is 0.6 x 0.4 m; a person 0.25 m in radius; vehicles with their forks)')
    print(f'{"agent":10s} {"run":16s} {"laps":>4s} {"worst":>6s} {"typical":>7s} {"v there":>7s} {"v<1m max":>8s} '
          f'{"s<1m/lap":>8s} {"contact s (moving/still/towards)":>33s}')
    for a in agents:
        for lab in labels:
            rows = [r for r in trials if r['label'] == lab and r['agent'] == a]
            if not rows:
                continue
            cl = [fnum(r['closest_m']) for r in rows]
            k = min(range(len(rows)), key=lambda i: cl[i] if cl[i] == cl[i] else 9e9)
            vmax = max((fnum(r['vmax_near'], 0.0) for r in rows if fnum(r['s_near'], 0.0) > 0), default=float('nan'))
            cm = sum(fnum(r['contact_moving_s'], 0.0) for r in rows)
            cs = sum(fnum(r['contact_still_s'], 0.0) for r in rows)
            ci = sum(fnum(r['drove_into_s'], 0.0) for r in rows)
            forks = [fnum(r.get('fork_closest_m')) for r in rows if r.get('fork_closest_m')]
            fk = f'  forks worst {min(forks):.2f} m' if forks else ''
            print(f'{a:10s} {lab:16s} {len(rows):4d} {f2(cl[k]):>6s} {f2(median(cl)):>7s} '
                  f'{f2(fnum(rows[k]["v_at_closest"])):>7s} {f2(vmax):>8s} '
                  f'{median([fnum(r["s_near"], 0.0) for r in rows]):8.1f} '
                  f'{cm:9.1f} / {cs:4.1f} / {ci:4.1f}{fk}   (worst in lap {rows[k]["lap"]}, {rows[k]["where"]})')
    for lab in labels:          # v10.1: a label run twice used to add its laps to the old ones
        n_laps = len(read_csv(os.path.join(RUNS, 'tour', f'{lab}_laps.csv')))
        n_rows = max((sum(1 for r in trials if r['label'] == lab and r['agent'] == a) for a in agents),
                     default=0)
        if n_laps and n_rows > n_laps:
            print(f'  NOTE {lab}: tour_trials.csv holds {n_rows} laps under this label, but its last run had '
                  f'{n_laps}. The rows above pool several runs (tour_test.py before v10.1 added each run '
                  f'to the old one). For one run\'s numbers, run again under a new label. (Don\'t delete '
                  f'tour_trials.csv: it holds every other label\'s rows too, the baseline\'s included.)')
    print()
    print('--- laps')
    for lab in labels:
        laps = read_csv(os.path.join(RUNS, 'tour', f'{lab}_laps.csv'))
        if not laps:
            print(f'  {lab}: no ~/runs/tour/{lab}_laps.csv')
            continue
        dur = [fnum(r['duration_s']) for r in laps]
        if laps[0]['results'].startswith('SHIFT'):         # v10.3: tour_test.py <label> watch
            r = laps[0]
            print(f'  {lab:16s} cleaning shift, {dur[0]:5.1f} s measured; coverage_nav2: '
                  f'{r["results"][6:]}; guard held the robot {fnum(r["guard_waits"], 0.0):.0f}x, '
                  f'{fnum(r["guard_wait_s"], 0.0):.0f} s in all')
            continue
        bad = [r for r in laps if any(x != 'SUCCEEDED' for x in r['results'].split('/'))]
        waits = [fnum(r['guard_waits'], 0.0) for r in laps]
        wait_s = [fnum(r['guard_wait_s'], 0.0) for r in laps]
        print(f'  {lab:16s} {len(laps)} laps, {median(dur):5.1f} s typical (longest {max(dur):5.1f} s); '
              f'guard held the robot {sum(waits):.0f}x, {sum(wait_s):.0f} s in all; '
              + ('every goal reached' if not bad else 'goals NOT reached: ' + '; '.join(
                  f'lap {r["lap"]} {r["results"]}' for r in bad)))
    print()
    print('--- the vehicles\' side (traffic tab: how long they stood waiting)')
    for lab in labels:
        rows = read_csv(os.path.join(RUNS, 'traffic', f'{lab}.csv'))
        veh = {}
        for r in rows:
            if r.get('kind') != 'vehicle':
                continue
            d = veh.setdefault(r['name'], dict(robot=0.0, person=0.0, n=0))
            d['n'] += 1
            st = r.get('state', '')
            if 'held:robot' in st and abs(fnum(r.get('v'), 0.0)) < 0.05:
                d['robot'] += 0.1
            elif 'held:person' in st and abs(fnum(r.get('v'), 0.0)) < 0.05:
                d['person'] += 0.1
        if not veh:
            print(f'  {lab}: no ~/runs/traffic/{lab}.csv (was the traffic tab running with this label?)')
            continue
        print(f'  {lab:16s} ' + '; '.join(f'{n} stood {d["robot"]:.0f} s waiting for the robot, '
                                          f'{d["person"]:.0f} s for people' for n, d in veh.items()))
        zs = {}
        for r in rows:
            if r.get('kind') == 'vehicle' and r.get('z', '') != '':
                zs.setdefault(r['name'], []).append(fnum(r['z']))
        if zs:
            t0 = min(fnum(r.get('t')) for r in rows)
            late = {n: [fnum(r['z']) for r in rows if r.get('name') == n and r.get('z', '') != ''
                        and fnum(r.get('t')) >= t0 + 10.0] for n in zs}
            print(f'  {"":16s} height (odometry, after its first 10 s): ' + '; '.join(
                f'{n} {min(v):+.3f} to {max(v):+.3f} m' + ('  <- OFF THE FLOOR' if max(-min(v), max(v)) > 0.05 else '')
                for n, v in late.items() if v))
        else:
            print(f'  {"":16s} height not logged (traffic before v10.2: the vehicles sank, so these numbers '
                  f'are against vehicles the lidar mostly could not see)')
    print()
    print('--- the guard\'s vehicle decisions (~/runs/guard/<label>.csv)')
    for lab in labels:
        g = read_csv(os.path.join(RUNS, 'guard', f'{lab}.csv'))
        if not g or 'veh_hold' not in g[0]:
            print(f'  {lab:16s} no vehicle columns (guard before v10, or GUARD_VEHICLES=0)' if g else
                  f'  {lab:16s} no guard log')
            continue
        t = [fnum(r['t']) for r in g]
        dt = median([b - a for a, b in zip(t, t[1:]) if b > a]) or 0.1
        hold = [r.get('veh_hold') == '1' for r in g]
        starts = sum(1 for a, b in zip(hold, hold[1:]) if b and not a)
        keep = [fnum(r.get('veh_keep')) for r in g]
        inside = sum(1 for k in keep if k == k and k < 0)
        seen = sum(1 for r in g if fnum(r.get('vehicles'), 0) > 0)
        print(f'  {lab:16s} vehicles followed in {100.0 * seen / len(g):.0f} % of scans; gave way / kept '
              f'clear {starts}x, {sum(hold) * dt:.0f} s; path into a standing vehicle\'s fork ground '
              f'in {inside} scans')
    print()
    print('--- emergency stop (lap 2)')
    for lab in labels:
        p = os.path.join(RUNS, 'tour', f'{lab}_estop.json')
        if not os.path.exists(p):
            print(f'  {lab:16s} not tested')
            continue
        e = json.load(open(p))
        if 'stop_t' in e:
            print(f'  {lab:16s} pressed at {e["v0"]:.2f} m/s: standstill after {e["stop_t"] - e["press_t"]:.2f} s '
                  f'and {e["stop_d"]:.2f} m; ' + (f'moving again {e["resume_t"] - e["release_t"]:.1f} s after release'
                                                 if 'resume_t' in e else 'did not move again in that lap'))
        else:
            print(f'  {lab:16s} pressed at {e.get("v0", float("nan")):.2f} m/s but the robot never stood still '
                  f'while it was pressed - look at the nav2 tab (gate lines)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
