#!/usr/bin/env python3
"""lane_report.py (v10.7, 12 Oct): how well a cleaning run actually swept the aisles' lanes,
and where its time went.

    python3 ~/ros2_ws/src/tools/lane_report.py shift_v105a shift_v106a

Reads ~/runs/cov_<label>/ (coverage_recorder: trail.csv, map.npy, passes.npy, summary.json) and
~/runs/logs/clean_<label>.log. For each run and aisle:
  lanes swept  - the share of the lanes' centre lines painted (0.25 m cells, 0.32 m brush: what
                 coverage_nav2 v10.5 calls an aisle's "swept" figure; 90 % counts as cleaned)
  floor        - the share of the floor between the rack ends a 0.30 m-radius robot can reach that
                 got painted (the recorder's own painting); "max" is what the lanes paint if
                 driven exactly
  per lane     - how much of each lane's length the robot drove within 0.25 m of the lane
  other aisles - time spent in aisles that were not in the job (a detour round someone)
and from the log: blockages gone round (BYPASS), detours refused (DETOUR), stretches put off
(DEFER), the patch pass (PATCH) and coverage_nav2's own summary line.
v10.6: the TIME BUDGET of the job (command to COVERAGE COMPLETE), from the guard's decisions
(~/runs/guard/<label>.csv): moving, held by the guard (and why), standing while the guard said
GO (Nav2 itself: something in the way, turning, a goal ending); the longest stands, with who was
near (~/runs/traffic/<label>.csv); and the v10.6 events: the simulation frozen (SIM PAUSED, real
time not counted), lane ends left to a vehicle (VEHICLE), stops short of something in a lane
(CLEARANCE), the guard silent (SAFETY), Nav2 not taking goals (NAV2), a second patch round.
v10.7: turns on the spot before a lane (ALIGN), getting unstuck from a rack (UNSTICK), and any goal
sent before the first COMMAND (there must be none: shift_v106a drove the built-in mission).
numpy only (no scipy); written for the VM's Python 3.10.
"""
import bisect
import csv
import json
import math
import os
import re
import sys

import numpy as np

RUNS = os.path.expanduser('~/runs')
SPAWN = (4.0, 4.0)
AISLES = {'aisle_1': (2.06, 4.96), 'aisle_2': (6.06, 8.96), 'aisle_3': (10.06, 13.01)}   # world
WALL_MARGIN, SPACING, END_MARGIN = 0.42, 0.55, 0.40
RACK_Y = (5.5, 17.5)                                                                     # world
CELL, BRUSH, ROBOT_R = 0.25, 0.32, 0.30


def lanes_of(z):
    """Lane x (map frame), as coverage_nav2.aisle_lanes places them."""
    xl, xr = AISLES[z]
    lo, hi = xl + WALL_MARGIN, xr - WALL_MARGIN
    n = max(2, int(math.ceil((hi - lo) / SPACING)) + 1)
    return [round(lo + (hi - lo) * i / (n - 1), 3) - SPAWN[0] for i in range(n)]


def lane_span():
    return (RACK_Y[0] - END_MARGIN - SPAWN[1], RACK_Y[1] + END_MARGIN - SPAWN[1])


def paint_cells(pts):
    """The painted grid exactly as coverage_nav2 paints it (segments between poses)."""
    vis = set()
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg > 2.0:                       # a jump in the trail: do not paint across it
            continue
        steps = max(1, int(seg / (CELL * 0.5)))
        for k in range(steps + 1):
            px, py = x0 + (x1 - x0) * k / steps, y0 + (y1 - y0) * k / steps
            for cx in range(int(math.floor((px - BRUSH) / CELL)), int(math.floor((px + BRUSH) / CELL)) + 1):
                for cy in range(int(math.floor((py - BRUSH) / CELL)), int(math.floor((py + BRUSH) / CELL)) + 1):
                    if math.hypot(cx * CELL + CELL / 2 - px, cy * CELL + CELL / 2 - py) <= BRUSH:
                        vis.add((cx, cy))
    return vis


def swept_pct(vis, lanes):
    lo, hi = lane_span()
    j0, j1 = int(math.floor(lo / CELL)), int(math.floor(hi / CELL))
    tot = done = 0
    for x in lanes:
        i = int(math.floor(x / CELL))
        for j in range(j0, j1 + 1):
            tot += 1
            done += (i, j) in vis
    return 100.0 * done / max(1, tot)


def on_lane_pct(xy, x):
    """% of the lane's length driven within 0.25 m of it."""
    lo, hi = lane_span()
    near = xy[np.abs(xy[:, 0] - x) <= 0.25]
    near = near[(near[:, 1] >= lo) & (near[:, 1] <= hi)]
    nb = int(round((hi - lo) / 0.25))
    hit = np.zeros(nb, bool)
    idx = np.clip(((near[:, 1] - lo) / 0.25).astype(int), 0, nb - 1)
    hit[idx] = True
    return 100.0 * hit.mean()


def disc(r_cells):
    r = int(math.ceil(r_cells))
    return [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1) if dx * dx + dy * dy <= r_cells ** 2]


def shift_and(mask, offs):
    out = mask.copy()
    H, W = mask.shape
    pad = np.pad(mask, max(max(abs(a), abs(b)) for a, b in offs), constant_values=False)
    p = (pad.shape[0] - H) // 2
    for dy, dx in offs:
        out &= pad[p + dy:p + dy + H, p + dx:p + dx + W]
    return out


def shift_or(mask, offs):
    out = mask.copy()
    H, W = mask.shape
    pad = np.pad(mask, max(max(abs(a), abs(b)) for a, b in offs), constant_values=False)
    p = (pad.shape[0] - H) // 2
    for dy, dx in offs:
        out |= pad[p + dy:p + dy + H, p + dx:p + dx + W]
    return out


def floor_figures(d, lanes_by_zone):
    """Per aisle: (% of coverable floor between the rack ends painted, % the lanes paint driven exactly)."""
    try:
        M = np.load(os.path.join(d, 'map.npy'))
        P = np.load(os.path.join(d, 'passes.npy'))
        S = json.load(open(os.path.join(d, 'summary.json')))
    except Exception:
        return {}
    res = S['map']['resolution']
    ox, oy = S['map']['origin']
    free = (M == 0)
    nav = shift_and(free, disc(ROBOT_R / res))               # the robot's centre fits
    cov = free & shift_or(nav, disc(BRUSH / res))            # ... and its brush reaches
    H, W = M.shape
    X = ox + (np.arange(W) + 0.5) * res
    Y = oy + (np.arange(H) + 0.5) * res
    XX, YY = np.meshgrid(X, Y)
    out = {}
    lo, hi = lane_span()
    for z, lanes in lanes_by_zone.items():
        xl, xr = AISLES[z][0] - SPAWN[0], AISLES[z][1] - SPAWN[0]
        box = cov & (XX >= xl) & (XX <= xr) & (YY >= RACK_Y[0] - SPAWN[1]) & (YY <= RACK_Y[1] - SPAWN[1])
        if not box.any():
            continue
        design = np.zeros_like(box)
        for x in lanes:
            design |= (np.abs(XX - x) <= BRUSH) & (YY >= lo - BRUSH) & (YY <= hi + BRUSH)
        out[z] = (100.0 * (box & (P > 0)).sum() / box.sum(), 100.0 * (box & design).sum() / box.sum())
    return out


def from_log(label):
    p = os.path.join(RUNS, 'logs', f'clean_{label}.log')
    if not os.path.exists(p):
        return None
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    c = dict(job=None, bypass=0, detour=0, defer=0, patch=None, swept=None, complete=None, revisit=0, skip=0)
    for ln in open(p, errors='replace'):
        ln = ansi.sub('', ln)
        m = re.search(r"COMMAND: clean (\[[^\]]*\])", ln)
        if m:
            c['job'] = re.findall(r"'(\w+)'", m.group(1))
        c['bypass'] += 'BYPASS ' in ln and '-> going round' in ln
        c['detour'] += 'DETOUR:' in ln
        c['defer'] += ' DEFER ' in f' {ln}' or ']: DEFER' in ln
        c['skip'] += ']: SKIP wp' in ln
        if 'REVISIT:' in ln and 'still dirty' in ln:
            c['revisit'] += 1
        m = re.search(r'PATCH: (\d+) stretch', ln)
        if m:
            c['patch'] = int(m.group(1))
        elif 'PATCH: every lane is swept' in ln:
            c['patch'] = 0
        m = re.search(r'lanes swept \(centre line painted\): (.*)', ln)
        if m:
            c['swept'] = m.group(1).strip()
        m = re.search(r'=== COVERAGE COMPLETE === (.*)', ln)
        if m:
            c['complete'] = m.group(1).strip()
    return c


ANSI = re.compile(r'\x1b\[[0-9;]*m')


def log_events(label):
    """[(wall, text)] of coverage_nav2's lines in ~/runs/logs/clean_<label>.log."""
    p = os.path.join(RUNS, 'logs', f'clean_{label}.log')
    out = []
    if not os.path.exists(p):
        return out
    for ln in open(p, errors='replace'):
        m = re.search(r'\[(\d{10}\.\d+)\] \[coverage_nav2\]: (.*)', ANSI.sub('', ln))
        if m:
            out.append((float(m.group(1)), m.group(2).strip()))
    return out


def budget(label, ev):
    """v10.6: where the job's time went, from the guard's decisions (one row per scan)."""
    gp = os.path.join(RUNS, 'guard', f'{label}.csv')
    if not os.path.exists(gp) or not ev:
        print('  time budget: no guard CSV or no log (needs ~/runs/guard/<label>.csv and the clean log)')
        return
    G = [r for r in csv.DictReader(open(gp)) if r.get('wall') not in ('', None)]
    if len(G) < 10:
        print('  time budget: the guard CSV is too short')
        return
    gt = [float(r['t']) for r in G]
    gw = [float(r['wall']) for r in G]

    def w2s(w):
        i = bisect.bisect_left(gw, w)
        if i <= 0:
            return gt[0]
        if i >= len(gw):
            return gt[-1]
        return gt[i - 1] + (gt[i] - gt[i - 1]) * (w - gw[i - 1]) / max(1e-9, gw[i] - gw[i - 1])
    t_cmd = next((w2s(w) for w, s in ev if s.startswith('COMMAND')), None)
    t_end = next((w2s(w) for w, s in ev if s.startswith('=== COVERAGE COMPLETE')), None)
    w_cmd = next((w for w, s in ev if s.startswith('COMMAND')), None)
    w_end = next((w for w, s in ev if s.startswith('=== COVERAGE COMPLETE')), None)
    if t_cmd is None:
        print('  time budget: no COMMAND line in the log')
        return
    done = t_end is not None
    t_end = t_end if done else gt[-1]
    cats, stands, cur, prev = {}, [], None, None
    for r in G:
        t = float(r['t'])
        if t < t_cmd or t > t_end:
            continue
        dt = 0.1 if prev is None else max(0.0, min(0.5, t - prev))
        prev = t
        v = float(r.get('robot_v') or 0)
        if v >= 0.05:
            cap = r.get('v_limit') not in ('', None) and float(r['v_limit']) < float(r.get('cruise') or 0.8) - 0.02
            c = 'moving, slowed by the guard' if cap else 'moving'
        elif r.get('state') == 'wait':
            c = f"held by the guard: {r.get('why') or '?'}"
        else:
            c = f"standing, guard GO ({r.get('why') or '?'})"
        cats[c] = cats.get(c, 0.0) + dt
        if v < 0.05:
            if cur is None:
                cur = [t, t, {}, (float(r.get('robot_x') or 0), float(r.get('robot_y') or 0))]
            cur[1] = t
            cur[2][c] = cur[2].get(c, 0.0) + dt
        elif cur is not None:
            stands.append(cur)
            cur = None
    if cur is not None:
        stands.append(cur)
    tot = max(1e-9, t_end - t_cmd)
    wall = (w_end - w_cmd) if (w_cmd is not None and w_end is not None) else None
    frozen = sum(float(m.group(1)) for _, s in ev for m in [re.search(r'SIM PAUSED: .*?for (\d+) s', s)] if m)
    head = f'  time budget, command to {"COVERAGE COMPLETE" if done else "the end of the record (NOT complete)"}: {tot:.0f} s of sim time'
    if wall:
        head += (f' ({wall / 60:.1f} min of real time' + (f', {frozen / 60:.1f} of it the simulation frozen'
                                                           if frozen else '') + ')')
    print(head)
    for c, s in sorted(cats.items(), key=lambda kv: -kv[1]):
        print(f'      {c:40s} {s:6.1f} s  {100 * s / tot:4.1f} %')
    TR = {}
    tp = os.path.join(RUNS, 'traffic', f'{label}.csv')
    if os.path.exists(tp):
        for r in csv.DictReader(open(tp)):
            try:
                TR.setdefault(r['name'], []).append((float(r['t']), float(r['x']), float(r['y']),
                                                     r.get('state') or ''))     # map frame, like the guard's
            except (KeyError, ValueError):
                pass

    def near(t, x, y):
        out = []
        for k, rows in TR.items():
            ts = [a[0] for a in rows]
            i = min(bisect.bisect_left(ts, t), len(ts) - 1)
            _, ax, ay, st = rows[i]
            d = math.hypot(ax - x, ay - y)
            if d < 3.5:
                out.append(f'{k} {d:.1f} m' + (f' ({st})' if st else ''))
        return out
    longest = sorted(stands, key=lambda s: -(s[1] - s[0]))[:6]
    if longest:
        print('      longest stands (sim s, robot map x,y; what the guard said; who was within 3.5 m):')
    for a, b, c, (x, y) in longest:
        if b - a < 3.0:
            break
        why = ', '.join(f'{k} {v:.0f} s' for k, v in sorted(c.items(), key=lambda kv: -kv[1])[:2])
        nb = near((a + b) / 2, x, y) if TR else []
        print(f'        {a:6.1f}-{b:6.1f} {b - a:5.1f} s at ({x:5.2f},{y:5.2f}): {why}'
              + (f' | near: {"; ".join(nb)}' if nb else ''))


def events(ev):
    """v10.6: what coverage_nav2 had to deal with, from its log (v10.7: ALIGN, UNSTICK, early goals)."""
    n = dict(vehicle=0, clearance=0, safety=0, nav2=0, round2=0, sim=0, align=0, unstick=0, early=0)
    commanded = False
    for _, s in ev:
        commanded = commanded or s.startswith('COMMAND: ')
        n['early'] += (not commanded) and bool(re.match(r'\d+/\d+: ', s))
        n['align'] += s.startswith('ALIGN: ') and 'turning on the spot' in s
        n['unstick'] += s.startswith('UNSTICK wp')
        n['vehicle'] += s.startswith('VEHICLE:')
        n['clearance'] += s.startswith('CLEARANCE:') and ('stopping short' in s or 'waiting short' in s)
        n['safety'] += s.startswith('SAFETY: no word')
        n['nav2'] += s.startswith('NAV2:') and 'taking goals again' not in s
        n['round2'] += s.startswith('PATCH round 2')
        n['sim'] += s.startswith('SIM PAUSED')
    return (f"  v10.6 events: vehicle put-offs {n['vehicle']}; stopped short of something in a lane "
            f"{n['clearance']}; guard silent {n['safety']}; Nav2 not taking goals {n['nav2']}; second patch "
            f"round {n['round2']}; simulation frozen {n['sim']}\n"
            f"  v10.7 events: turned on the spot before a lane {n['align']}; got unstuck from a rack "
            f"{n['unstick']}; goals before the first command {n['early']}"
            + ("  <- SHOULD BE 0" if n['early'] else ''))


def report(label):
    d = os.path.join(RUNS, f'cov_{label}')
    tp = os.path.join(d, 'trail.csv')
    if not os.path.exists(tp):
        print(f'{label}: no {tp} (shift.sh renames ~/runs/latest to cov_{label} when the run ends; '
              f'by hand: mv ~/runs/latest ~/runs/cov_{label} after stack.sh stop)')
        return
    rows = list(csv.DictReader(open(tp)))
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    t = np.array([float(r['sim_t']) for r in rows])
    log = from_log(label)
    job = (log or {}).get('job') or [z for z in AISLES
                                      if np.any((xy[:, 0] >= AISLES[z][0] - SPAWN[0]) & (xy[:, 0] <= AISLES[z][1] - SPAWN[0]))]
    zones = [z for z in job if z in AISLES]
    vis = paint_cells([tuple(p) for p in xy])
    lanes_by_zone = {z: lanes_of(z) for z in zones}
    floor = floor_figures(d, lanes_by_zone)
    print(f'=== {label}: {len(rows)} poses, {t[-1] - t[0]:.0f} s of sim time; job: {", ".join(job) if job else "?"}')
    for z in zones:
        f = floor.get(z)
        ft = f'; floor between the rack ends painted {f[0]:.1f} % (max {f[1]:.1f} %)' if f else ''
        print(f'  {z}: lanes swept {swept_pct(vis, lanes_by_zone[z]):5.1f} %{ft}')
        print('      on each lane (x: % of its length): '
              + ', '.join(f'{x:+.2f}: {on_lane_pct(xy, x):3.0f}' for x in lanes_by_zone[z]))
    ylo, yhi = RACK_Y[0] - SPAWN[1], RACK_Y[1] - SPAWN[1]
    dt = np.diff(t, append=t[-1])
    for z in AISLES:
        if z in zones:
            continue
        xl, xr = AISLES[z][0] - SPAWN[0], AISLES[z][1] - SPAWN[0]
        inside = (xy[:, 0] >= xl) & (xy[:, 0] <= xr) & (xy[:, 1] > ylo + 0.3) & (xy[:, 1] < yhi - 0.3)
        if inside.any():
            print(f'  {z} (not in the job): {dt[inside].sum():.0f} s inside it, a detour')
    if log:
        print(f'  log: went round {log["bypass"]} blockage(s); refused {log["detour"]} detour(s); '
              f'put off {log["defer"]}, revisits {log["revisit"]}, skipped {log["skip"]}; patch pass '
              + ('not run' if log['patch'] is None else f'{log["patch"]} stretch(es)'))
        if log['swept']:
            print(f'  coverage_nav2: lanes swept {log["swept"]}')
        if log['complete']:
            print(f'  coverage_nav2: {log["complete"]}')
    ev = log_events(label)
    if ev:
        print(events(ev))
    budget(label, ev)


def main():
    labels = sys.argv[1:] or ['shift_v105a']
    for lab in labels:
        report(lab)
    return 0


if __name__ == '__main__':
    sys.exit(main())
