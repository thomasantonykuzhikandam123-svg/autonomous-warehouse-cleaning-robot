#!/usr/bin/env python3
"""guard_report.py - one compact report of a people-guard test, to paste to Claude.

    python3 ~/ros2_ws/src/tools/guard_report.py <label>

Reads what the run left in ~/runs:
  people_trials.csv        the probe's rows (every label, for comparison)
  traces/<label>.csv       robot + worker whenever she was within 2.5 m
  guard/<label>.csv        every guard decision (people_guard.py v5 / v6 / v7)
  guard_last.log           the guard tab's GO / WAIT lines
  nav2_last.log            the Nav2 tab's output (stack.sh ... guard)
and prints: a comparison table, one line per pass (where the robot waited
and how far from her line, lurches), comfort per pass (closest in front of
her and behind her back, hardest speed-up and slow-down near her), a 0.2 s
timeline around the closest moment of every pass with a contact (plus the
guard's raw rows), how fast the robot really got going after each GO
compared with the guard's robot model, where the Collision Monitor's
emergency braking fired, the safety gate's stops, and counts of the Nav2
and Collision Monitor lines that matter. Nothing is changed or written.
"""
import bisect
import csv
import os
import sys

RUNS = os.path.expanduser('~/runs')
LANE_X = (-1.25, 0.25)        # northbound (up) and southbound (down) lane in aisle 1
HER_Y = 4.0                   # the worker's walking line (map y)
NAV2_PATTERNS = [
    ('RPP collision ahead', 'detected collision ahead'),
    ('controller failed', 'Controller patience exceeded'),
    ('no progress', 'Failed to make progress'),
    ('planner failed', 'failed to generate a valid path'),
    ('monitor: stop (guard hold)', 'Robot to stop due to PeopleHold'),
    ('monitor: emergency braking', 'Robot to approach for'),
    ('monitor: back to normal', 'Robot to continue normal operation'),
    ('goal aborted', 'Goal failed'),
    ('gate: robot stopped', 'gate STOP'),
]


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def fnum(v, default=float('nan')):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def comparison(probe, label):
    print('--- all labels in people_trials.csv (contact = centre distance < 0.56 m)')
    print(f'{"label":16s} passes contacts robot-moving-at-closest mean_s')
    order = []
    for r in probe:
        if r.get('label') and r['label'] not in order:
            order.append(r['label'])
    for lab in order:
        rows = [r for r in probe if r['label'] == lab]
        c = sum(1 for r in rows if r.get('contact') == 'YES')
        a = sum(1 for r in rows if r.get('contact') == 'YES' and r.get('approaching_at_min') == 'yes')
        mean = sum(fnum(r.get('duration_s'), 0.0) for r in rows) / max(len(rows), 1)
        mark = '  <- this run' if lab == label else ''
        print(f'{lab:16s} {len(rows):6d} {c:8d} {a:22d} {mean:6.1f}{mark}')
    print()


def per_pass(probe, traces, decisions, label):
    rows = [r for r in probe if r.get('label') == label]
    if not rows:
        print(f'--- no probe rows for {label} yet\n')
        return []
    dts = [fnum(d['t']) for d in decisions]
    print('--- per pass  (lane offset: how far the robot was off its lane in the aisle;')
    print('              waits / switches: guard decisions while she was within 2.5 m;')
    print('              held_at: closest the robot stood to her walking line (want >= 0.76 m);')
    print('              lurches: GO lasting under 0.6 s before the next WAIT, not counting')
    print('              the one slow edge-up (v7) after 6 s at a stop line without a comfortable gap)')
    print('pass goal     dir  result     dur  min_d contact moving  lane_off  waits wait_s switches '
          'held_at lurches')
    contact_passes = []
    totals = dict(in_band=0, lurches=0, edges=0)
    for r in rows:
        tr = [t for t in traces if t['goal'] == r['goal']]
        up = bool(tr) and fnum(tr[0]['robot_yaw'], 0.0) > 0
        off = [abs(fnum(t['robot_x']) - LANE_X[0 if up else 1]) for t in tr
               if 1.5 < fnum(t['robot_y']) < 7.0]
        waits = wait_s = switches = lurches = 0
        go_since, go_edge = None, False
        if tr and decisions:
            t0, t1 = fnum(tr[0]['t']) - 3.0, fnum(tr[-1]['t']) + 0.5
            i0, i1 = bisect.bisect_left(dts, t0), bisect.bisect_right(dts, t1)
            seg = decisions[i0:i1]
            for a, b in zip(seg, seg[1:]):
                if a['state'] != b['state']:
                    switches += 1
                    waits += b['state'] == 'wait'
                    if b['state'] == 'go':
                        go_since = fnum(b['t'])
                        go_edge = b.get('why') == 'approach' and b.get('edge') == '1'
                        totals['edges'] += go_edge
                    elif go_since is not None and fnum(b['t']) - go_since < 0.6 and not go_edge:
                        lurches += 1
                if a['state'] == 'wait':
                    wait_s += fnum(b['t']) - fnum(a['t'])
        still = [abs(fnum(t['robot_y']) - HER_Y) for t in tr if fnum(t['robot_speed'], 1.0) < 0.05]
        held = min(still) if still else float('nan')
        totals['in_band'] += int(bool(still) and held < 0.76)
        totals['lurches'] += lurches
        print(f'{r["pass"]:>4s} {r["goal"]:8s} {"up" if up else "down":4s} {r["result"][:9]:9s} '
              f'{fnum(r["duration_s"]):5.1f} {fnum(r["min_dist_m"]):5.2f} {r["contact"]:7s} '
              f'{r["approaching_at_min"]:6s} {max(off) if off else float("nan"):8.2f} '
              f'{waits:6d} {wait_s:6.1f} {switches:8d} {held:7.2f} {lurches:7d}')
        if r.get('contact') == 'YES':
            contact_passes.append((r, tr))
    print(f'  passes where the robot stood inside her band (< 0.76 m from her line): '
          f'{totals["in_band"]} of {len(rows)};  lurches: {totals["lurches"]}'
          + (f'  (edge-ups after a long wait: {totals["edges"]})' if totals['edges'] else ''))
    print()
    return contact_passes


def comfort(probe, traces, decisions, label):
    """Per pass: closest centre distance while the robot was in front of her (in her
    walking direction, or while she turned) and while it was behind her back; the
    hardest speed-up and slow-down (m/s^2 over 0.1 s, from /odom in the guard log)
    while she was within 2 m."""
    rows = [r for r in probe if r.get('label') == label]
    if not rows or not traces:
        return
    dts = [fnum(d['t']) for d in decisions]
    print('--- comfort per pass  (front: robot in her walking direction or she was turning;')
    print('              behind: robot behind her back; centre distances, contact < 0.56 m;')
    print('              speed-up / slow-down: hardest change of the robot speed within 2 m of her)')
    print('pass dir   front_min behind_min  speed-up slow-down  (m/s^2)')
    fronts, behinds, ups, downs = [], [], [], []
    for r in rows:
        tr = [t for t in traces if t['goal'] == r['goal']]
        if not tr:
            continue
        up = fnum(tr[0]['robot_yaw'], 0.0) > 0
        front = behind = float('nan')
        for a, b in zip(tr, tr[1:]):
            dt = fnum(b['t']) - fnum(a['t'])
            if not 0.05 < dt < 0.25:
                continue
            hx = (fnum(b['worker_x']) - fnum(a['worker_x'])) / dt
            hy = (fnum(b['worker_y']) - fnum(a['worker_y'])) / dt
            rx, ry = fnum(b['robot_x']) - fnum(b['worker_x']), fnum(b['robot_y']) - fnum(b['worker_y'])
            d = fnum(b['dist'])
            if (hx * hx + hy * hy) ** 0.5 > 0.3 and rx * hx + ry * hy < 0:
                behind = d if behind != behind else min(behind, d)
            else:
                front = d if front != front else min(front, d)
        acc = dec = float('nan')
        if decisions:
            t0, t1 = fnum(tr[0]['t']), fnum(tr[-1]['t'])
            i0, i1 = bisect.bisect_left(dts, t0), bisect.bisect_right(dts, t1)
            tt = [fnum(t['t']) for t in tr]
            seg = decisions[i0:i1]
            for a, b in zip(seg, seg[1:]):
                dt = fnum(b['t']) - fnum(a['t'])
                if not 0.05 < dt < 0.25:
                    continue
                j = bisect.bisect_left(tt, fnum(b['t']))
                near = [k for k in (j - 1, j) if 0 <= k < len(tr) and abs(tt[k] - fnum(b['t'])) < 0.15]
                if not near or fnum(tr[near[0]]['dist']) > 2.0:
                    continue
                dv = (fnum(b['robot_v']) - fnum(a['robot_v'])) / dt
                acc = dv if acc != acc else max(acc, dv)
                dec = -dv if dec != dec else max(dec, -dv)
        for lst, v in ((fronts, front), (behinds, behind), (ups, acc), (downs, dec)):
            if v == v:
                lst.append(v)
        print(f'{r["pass"]:>4s} {"up" if up else "down":4s} {front:10.2f} {behind:10.2f} '
              f'{acc:9.1f} {dec:9.1f}')
    if fronts:
        print(f'  closest in front of her {min(fronts):.2f} m (v6 run: 0.98-1.16 m overall), '
              f'behind her {min(behinds) if behinds else float("nan"):.2f} m; hardest speed-up '
              f'{max(ups) if ups else float("nan"):.1f}, slow-down '
              f'{max(downs) if downs else float("nan"):.1f} m/s^2')
    print()


def braking(decisions, traces):
    """Where the Collision Monitor slowed the robot down (its emergency braking):
    rows where it let through less than 80 % of the commanded forward speed. The
    first 0.25 s after a GO don't count: the monitor drops the guard's hold a moment
    after the guard does (v7.1; in guard7_lanes 5 of 8 'brakings' were that)."""
    if not decisions or 'cm_in' not in decisions[0]:
        return
    tt = [fnum(t['t']) for t in traces]
    events, cur = [], None
    go_t, prev = None, None
    for d in decisions:
        cin, cout = fnum(d.get('cm_in')), fnum(d.get('cm_out'))
        t = fnum(d['t'])
        if d['state'] == 'go' and prev != 'go':
            go_t = t
        prev = d['state']
        hit = (d['state'] == 'go' and cin == cin and cout == cout and cin > 0.1 and cout < 0.8 * cin
               and (go_t is None or t - go_t > 0.25))
        if hit:
            if cur is not None and t - cur['t1'] <= 0.35:
                cur['t1'], cur['ratio'] = t, min(cur['ratio'], cout / cin)
            else:
                cur = dict(t0=t, t1=t, ratio=cout / cin, x=fnum(d['robot_x']), y=fnum(d['robot_y']),
                           yaw=fnum(d['robot_yaw']))
                events.append(cur)
    print(f'--- Collision Monitor emergency braking (it let through < 80 % of the command): '
          f'{len(events)} times')
    for e in events[:20]:
        k = bisect.bisect_left(tt, e['t0'])
        near = [j for j in (k - 1, k) if 0 <= j < len(traces) and abs(tt[j] - e['t0']) < 0.15]
        her = f'{fnum(traces[near[0]]["dist"]):4.2f} m from her' if near else 'she was > 2.5 m away'
        print(f'  [{e["t0"]:7.1f}] {e["t1"] - e["t0"] + 0.1:3.1f} s, down to {e["ratio"]:.0%}, robot at '
              f'({e["x"]:5.2f}, {e["y"]:5.2f}) heading {e["yaw"]:+.1f} rad, {her}')
    print()


def gate_lines():
    path = os.path.join(RUNS, 'nav2_last.log')
    if not os.path.exists(path):
        return
    lines = [ln for ln in open(path, errors='replace').read().splitlines() if 'gate ' in ln]
    stops = [ln for ln in lines if 'gate STOP' in ln]
    start = [ln for ln in stops if 'no message from people_guard.py yet' in ln]
    print(f'--- safety gate (guard_gate.py): {len(stops)} stop(s)'
          + (f', {len(start)} of them at start-up (normal: before the guard is running)' if start else ''))
    for ln in stops[:10]:
        print('  ' + ln.strip()[-110:])
    print()


def timelines(contact_passes, decisions):
    if not contact_passes:
        print('--- no contacts: no timelines needed\n')
        return
    dts = [fnum(d['t']) for d in decisions]
    for r, tr in contact_passes:
        if not tr:
            continue
        k = min(range(len(tr)), key=lambda i: fnum(tr[i]['dist']))
        tmin = fnum(tr[k]['t'])
        print(f'--- pass {r["pass"]} ({r["goal"]}): closest {fnum(tr[k]["dist"]):.2f} m at t {tmin:.1f}; '
              f'0.2 s steps from -3 s to +1 s')
        print('     t   ph  robot_x robot_y  v    her_x   d   | guard: state why       d_go d_stop can  '
              'saw_x  saw_y  saw_v  kind')
        last = None
        for t in tr:
            tt = fnum(t['t'])
            if not (tmin - 3.0 <= tt <= tmin + 1.0) or (last is not None and tt - last < 0.19):
                continue
            last = tt
            g = ''
            if decisions:
                i = bisect.bisect_left(dts, tt)
                cand = [j for j in (i - 1, i) if 0 <= j < len(decisions)]
                if cand:
                    j = min(cand, key=lambda j: abs(dts[j] - tt))
                    d = decisions[j]
                    if abs(dts[j] - tt) < 0.15:
                        g = (f'{d["state"]:5s} {d["why"]:9s} {fnum(d["d_go"]):5.2f} {fnum(d["d_stop"]):5.2f} '
                             f'{d["can_go_for"] or "-":>4s} {fnum(d["p_x"]):6.2f} {fnum(d["p_y"]):6.2f} '
                             f'{fnum(d["p_speed"]):5.2f}  {d["p_kind"]}')
            print(f'{tt:7.1f} {tt % 6.0:4.1f} {fnum(t["robot_x"]):7.2f} {fnum(t["robot_y"]):7.2f} '
                  f'{fnum(t["robot_speed"]):4.2f} {fnum(t["worker_x"]):6.2f} {fnum(t["dist"]):5.2f} | {g}')
        print()


def raw_rows(contact_passes, decisions):
    """The guard's own log rows, 2.5 s before to 0.5 s after each closest moment."""
    if not contact_passes or not decisions:
        return
    keep = [k for k in ('t', 'state', 'why', 'd_go', 'd_go_nom', 'd_stop', 'can_go_for', 'who',
                        'who_stop', 'robot_x', 'robot_y', 'robot_v', 'people', 'p_id', 'p_x',
                        'p_y', 'p_vx', 'p_vy', 'p_kind', 'patient', 's_hold', 's_band', 'inside',
                        'fresh', 'p_age', 'room', 'd_comf', 'comfy', 'prog_left', 'v_limit',
                        'cm_in', 'cm_out', 'edge') if k in decisions[0]]
    for r, tr in contact_passes:
        if not tr:
            continue
        k = min(range(len(tr)), key=lambda i: fnum(tr[i]['dist']))
        tmin = fnum(tr[k]['t'])
        print(f'--- pass {r["pass"]}: guard log rows {tmin - 2.5:.1f} .. {tmin + 0.5:.1f}')
        print(','.join(keep))
        for d in decisions:
            if tmin - 2.5 <= fnum(d['t']) <= tmin + 0.5:
                print(','.join(d.get(c, '') for c in keep))
        print()


def model_distance(T, v_top, accel, delay, step=0.0):
    """Distance after T s from a stop: wait `delay`, step to `step` m/s, then speed up
    at `accel` to v_top."""
    T = max(T - delay, 0.0)
    v0 = min(step, v_top)
    t_acc = (v_top - v0) / accel
    if T < t_acc:
        return v0 * T + 0.5 * accel * T * T
    return v0 * t_acc + 0.5 * accel * t_acc * t_acc + v_top * (T - t_acc)


def calibration(decisions):
    """How far the robot really got 1, 2 and 3 s after each WAIT -> GO, against the
    guard's robot model (v6: slow and fast; v5.1 logs: its slow robot)."""
    if len(decisions) < 50 or 'cruise' not in decisions[0]:
        return
    ts = [fnum(d['t']) for d in decisions]
    got = {1.0: [], 2.0: [], 3.0: []}
    for i in range(1, len(decisions)):
        if decisions[i - 1]['state'] == 'wait' and decisions[i]['state'] == 'go' \
                and fnum(decisions[i]['robot_v'], 1.0) < 0.1:
            t0 = ts[i]
            for T in got:
                j = bisect.bisect_left(ts, t0 + T)
                if j < len(decisions) and abs(ts[j] - t0 - T) < 0.15:
                    seg = decisions[i:j + 1]
                    if all(d['state'] == 'go' for d in seg):
                        L = sum(((fnum(b['robot_x']) - fnum(a['robot_x'])) ** 2 +
                                 (fnum(b['robot_y']) - fnum(a['robot_y'])) ** 2) ** 0.5
                                for a, b in zip(seg, seg[1:]))
                        got[T].append(L)
    cruise = fnum(decisions[-1].get('cruise'), 0.0)
    v6 = 's_hold' in decisions[0]
    v7 = 'v_limit' in decisions[0]
    print(f'--- getting going after WAIT -> GO from a stop (guard cruise {cruise:.2f} m/s)')
    if v7:
        print('  after   robot really (median, n)   guard model near people (gentle start): slow .. fast')
    elif v6:
        print('  after   robot really (median, n)   guard model: slow .. fast (the real robot should be in between)')
    else:
        print('  after   robot really (median, n)   guard slow-robot model (v5.1)')
    for T, L in got.items():
        if v7:
            model = (f'{model_distance(T, 0.95 * cruise, 0.68, 0.30, 0.20):5.2f} .. '
                     f'{model_distance(T, 1.05 * cruise, 1.04, 0.10, 0.30):4.2f} m')
        elif v6:
            model = (f'{model_distance(T, 0.95 * cruise, 4.0, 0.25):5.2f} .. '
                     f'{model_distance(T, 1.05 * cruise, 8.0, 0.10):4.2f} m')
        else:
            model = f'{model_distance(T, 0.75 * cruise, 0.8, 0.0):5.2f} m'
        if L:
            L.sort()
            print(f'  {T:.0f} s     {L[len(L) // 2]:5.2f} m ({len(L):3d})           {model}')
        else:
            print(f'  {T:.0f} s     no clean starts            {model}')
    print()


def nav2_counts():
    path = os.path.join(RUNS, 'nav2_last.log')
    if not os.path.exists(path):
        print('--- no ~/runs/nav2_last.log (start the stack with: stack.sh start <label> guard)\n')
        return
    text = open(path, errors='replace').read()
    print('--- Nav2 tab (nav2_last.log): how often each line appeared')
    for name, pat in NAV2_PATTERNS:
        print(f'  {name:26s} {text.count(pat)}')
    print()


def guard_tail():
    path = os.path.join(RUNS, 'guard_last.log')
    if not os.path.exists(path):
        return
    lines = open(path, errors='replace').read().splitlines()
    for ln in lines[:10]:
        if 'cruise speed' in ln or 'people_guard v' in ln or 'nav2_msgs' in ln:
            print('  ' + ln)
    print(f'--- guard tab: {sum("WAIT" in ln for ln in lines)} WAIT and {sum("GO " in ln for ln in lines)} '
          f'GO lines; the last 25:')
    for ln in lines[-25:]:
        print('  ' + ln)
    print()


def main():
    if len(sys.argv) < 2:
        print('usage: guard_report.py <label>')
        return 1
    label = sys.argv[1]
    probe = read_csv(os.path.join(RUNS, 'people_trials.csv'))
    traces = read_csv(os.path.join(RUNS, 'traces', f'{label}.csv'))
    decisions = read_csv(os.path.join(RUNS, 'guard', f'{label}.csv'))
    decisions.sort(key=lambda d: fnum(d['t']))
    print(f'=== guard_report {label}: {len(traces)} trace rows, {len(decisions)} guard decisions\n')
    comparison(probe, label)
    contact_passes = per_pass(probe, traces, decisions, label)
    comfort(probe, traces, decisions, label)
    timelines(contact_passes, decisions)
    raw_rows(contact_passes, decisions)
    calibration(decisions)
    braking(decisions, traces)
    gate_lines()
    nav2_counts()
    guard_tail()
    return 0


if __name__ == '__main__':
    sys.exit(main())
