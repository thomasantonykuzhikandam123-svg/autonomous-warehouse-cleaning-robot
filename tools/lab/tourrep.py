#!/usr/bin/env python3
"""tourrep.py - open-loop replica of the Gazebo tour, to test the guard's VEHICLE PERCEPTION.

The robot drives the tour's lane routes (lanes.py) at a fixed speed, ignoring the guard. The
traffic is traffic.py's own Driver logic (closed loop on the robot and people), the people are
traffic.py's scripts plus the world's worker, and the lidar is raycast against the warehouse
boxes, the people's legs and each vehicle's parts AT THE LIDAR PLANE (from traffic.vehicle_sdf).
Every scan goes through the real GuardCore. Then the same 'in sight / followed' measure as
tour_why, plus where in the vehicle pipeline each vehicle was lost.

    python3 tourrep.py <guard.py> [laps=3] [beams=360] [rmax=12] [noise=0.01] [hz=10] [seed=0]
                       [speed=0.8] [fov=360] [lag=0] [phase=0] [set:NAME=value ...]
"""
import collections
import importlib.util
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
V10 = os.environ.get('V10') or next((p for p in (os.path.join(_HERE, '..', 'v10'), os.path.join(_HERE, '..'))
                                     if os.path.exists(os.path.join(p, 'traffic.py'))),
                                    os.path.join(_HERE, '..', 'v10'))   # v10.3: on the VM, tools/lab/.. = tools
sys.path.insert(0, V10)
import lanes  # noqa: E402
import traffic as TR  # noqa: E402

OX, OY, RES, W, H = -4.0, -4.0, 0.05, 300, 400
BOXES = {n: (x0 - 4, y0 - 4, x1 - 4, y1 - 4) for n, (x0, y0, x1, y1) in lanes.OBSTACLES.items()}
SIGHT_BOXES = [(x0 + 0.05, y0 + 0.05, x1 - 0.05, y1 - 0.05) for n, (x0, y0, x1, y1) in BOXES.items()
               if not n.endswith('_wall')]

# vehicle parts at the lidar plane (0.28 m): model-frame (cx, cy, sx, sy), from traffic.vehicle_sdf
LIDAR_Z = 0.28


def parts_at_plane(name, z_cut=None):
    """The model's boxes cut at height z_cut in its own frame (the lidar plane, or deeper into the
    model when it has sunk: z_cut = LIDAR_Z + depth)."""
    z_cut = LIDAR_Z if z_cut is None else z_cut
    sp = TR.VEHICLES[name]
    L, W_, Hh, fork = sp['L'], sp['W'], sp['H'], sp['fork']
    boxes = []
    if fork > 0:
        boxes += [(-0.1, 0, L - 0.2, W_, 0.55, 0.9), (-L / 2 + 0.15, 0, 0.3, W_, 0.75, 1.1),
                  (L / 2 - 0.05, W_ / 2 - 0.12, 0.1, 0.08, 1.15, 2.1),
                  (L / 2 - 0.05, -W_ / 2 + 0.12, 0.1, 0.08, 1.15, 2.1),
                  (0.0, 0, L - 0.3, W_, Hh, 0.06),
                  (L / 2 + fork / 2, 0.25, fork, 0.12, 0.05, 0.05),
                  (L / 2 + fork / 2, -0.25, fork, 0.12, 0.05, 0.05),
                  (L / 2 + 0.02, 0, 0.06, 0.8, 0.4, 0.7)]
    else:
        boxes += [(0, 0, L, W_, 0.45, 0.7), (-L / 2 + 0.25, 0, 0.1, W_ - 0.1, 1.0, 0.5),
                  (-L / 2 - 0.05, 0, 0.1, 0.15, 0.3, 0.1)]
    return [(cx, cy, sx, sy) for cx, cy, sx, sy, z, sz in boxes if z - sz / 2 <= z_cut <= z + sz / 2]


PARTS = {n: parts_at_plane(n) for n in TR.VEHICLES}


def occupancy():
    g = np.zeros((H, W), dtype=np.int8)
    for x0, y0, x1, y1 in BOXES.values():
        i0, i1 = int((x0 - OX) / RES), int(math.ceil((x1 - OX) / RES))
        j0, j1 = int((y0 - OY) / RES), int(math.ceil((y1 - OY) / RES))
        g[max(j0, 0):min(j1, H), max(i0, 0):min(i1, W)] = 100
    return g


def ray_aabb(x, y, dx, dy, box, rmin):
    x0, y0, x1, y1 = box
    with np.errstate(divide='ignore', invalid='ignore'):
        tx1, tx2 = (x0 - x) / dx, (x1 - x) / dx
        ty1, ty2 = (y0 - y) / dy, (y1 - y) / dy
        tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
        tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    tmin = np.where(np.abs(dx) < 1e-12, np.where((x0 <= x) & (x <= x1), np.minimum(ty1, ty2), np.inf), tmin)
    tmin = np.where(np.abs(dy) < 1e-12, np.where((y0 <= y) & (y <= y1), np.minimum(tx1, tx2), np.inf), tmin)
    return np.where((tmax >= tmin) & (tmin > rmin), tmin, np.inf)


def ray_obb(x, y, dx, dy, cx, cy, yaw, sx, sy, rmin):
    c, s = math.cos(-yaw), math.sin(-yaw)
    px, py = x - cx, y - cy
    lx, ly = c * px - s * py, s * px + c * py
    ldx, ldy = c * dx - s * dy, s * dx + c * dy
    return ray_aabb(lx, ly, ldx, ldy, (-sx / 2, -sy / 2, sx / 2, sy / 2), rmin)


def ray_circle(x, y, dx, dy, cx, cy, r, rmin):
    fx, fy = x - cx, y - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    tt = -b - np.sqrt(np.maximum(disc, 0.0))
    return np.where((disc >= 0) & (tt > rmin), tt, np.inf)


def legs_of(x, y, vx, vy, t):
    sp = math.hypot(vx, vy)
    if sp < 0.05:
        return [(x, y + 0.09), (x, y - 0.09)]
    ux, uy = vx / sp, vy / sp
    s = 0.18 * math.sin(2 * math.pi * t / 1.1)
    return [(x + ux * s - uy * 0.09, y + uy * s + ux * 0.09),
            (x - ux * s + uy * 0.09, y - uy * s - ux * 0.09)]


def worker(t):
    wp = [(0.0, -1.6), (2.2, 0.6), (3.0, 0.6), (5.2, -1.6), (6.0, -1.6)]
    p = t % 6.0
    for (t0, x0), (t1, x1) in zip(wp, wp[1:]):
        if t0 <= p <= t1:
            f = (p - t0) / (t1 - t0)
            return x0 + f * (x1 - x0), 4.0, (x1 - x0) / (t1 - t0)
    return -1.6, 4.0, 0.0


def body_corners(name, x, y, yaw):
    sp = TR.VEHICLES[name]
    return TR.corners(x, y, yaw, sp['L'], sp['W'])


def part_polys(name, x, y, yaw, parts=None):
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for cx, cy, sx, sy in (PARTS[name] if parts is None else parts):
        wx, wy = x + c * cx - s * cy, y + s * cx + c * cy
        out.append((wx, wy, yaw, sx, sy))
    return out


def dist_to_parts(px, py, name, x, y, yaw):
    """Distance from points (n,) to the vehicle's lidar-plane parts (0 inside)."""
    best = np.full(np.shape(px), np.inf)
    for wx, wy, a, sx, sy in part_polys(name, x, y, yaw):
        c, s = math.cos(-a), math.sin(-a)
        lx = c * (px - wx) - s * (py - wy)
        ly = s * (px - wx) + c * (py - wy)
        dx = np.maximum(np.abs(lx) - sx / 2, 0.0)
        dy = np.maximum(np.abs(ly) - sy / 2, 0.0)
        best = np.minimum(best, np.hypot(dx, dy))
    return best


def crosses(ax, ay, bx, by, box):
    x0, y0, x1, y1 = box
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
        if p == 0:
            if q < 0:
                return False
            continue
        r = q / p
        if p < 0:
            if r > t1:
                return False
            t0 = max(t0, r)
        else:
            if r < t0:
                return False
            t1 = min(t1, r)
    return t0 <= t1


def in_sight(name, x, y, yaw, rx, ry):
    pts = [(x, y)] + body_corners(name, x, y, yaw)
    return any(math.hypot(px - rx, py - ry) <= 5.0 and not any(crosses(rx, ry, px, py, b) for b in SIGHT_BOXES)
               for px, py in pts)


def polyline_route():
    """One lap as a dense polyline [(x, y, goal_index)], corners rounded (RPP-like)."""
    goals = [((0.0, 0.0), (-1.25, 12.5)), ((-1.25, 12.5), (4.25, 12.0)), ((4.25, 12.0), (0.0, 0.0))]
    segs = []
    for gi, (s, g) in enumerate(goals):
        pts = [s] + [(p[0], p[1]) for p in lanes.route(s, g)]
        segs.append((gi, pts))
    return segs


def round_corners(pts, r=0.6, step=0.05):
    """Dense polyline through pts with corners cut by about r (like a lookahead)."""
    pts = [np.array(p, dtype=float) for p in pts]
    dense = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        L = float(np.hypot(*(b - a)))
        n = max(int(L / step), 1)
        for k in range(1, n + 1):
            dense.append(a + (b - a) * k / n)
    dense = np.array(dense)
    if len(dense) < 3:
        return dense
    # moving average over +-r/step points (cuts corners), ends pinned
    k = max(int(r / step), 1)
    out = dense.copy()
    for i in range(len(dense)):
        lo, hi = max(0, i - k), min(len(dense), i + k + 1)
        w = min(i - lo, hi - 1 - i)
        out[i] = dense[i - w:i + w + 1].mean(axis=0)
    return out


def main(argv):
    path = argv[0]
    opts = dict(a.split('=', 1) for a in argv[1:] if '=' in a and not a.startswith('set:'))
    sets = {a[4:].split('=', 1)[0]: float(a[4:].split('=', 1)[1]) for a in argv[1:] if a.startswith('set:')}
    laps = int(opts.get('laps', 3))
    beams = int(opts.get('beams', 360))
    rmax = float(opts.get('rmax', 12.0))
    noise = float(opts.get('noise', 0.01))
    hz = float(opts.get('hz', 10.0))
    seed = int(opts.get('seed', 0))
    speed = float(opts.get('speed', 0.8))
    fov = math.radians(float(opts.get('fov', 360)))
    lag = float(opts.get('lag', 0.0))            # s: laser pose used is this old (TF lag)
    phase = float(opts.get('phase', 0.0))        # s the traffic runs before the robot starts
    verbose = int(opts.get('v', 0))
    dump = opts.get('dump')                      # label: write ~/runs/{rec,traffic,guard}/<label>.*
    occ_extra = opts.get('occ')                  # 'wallfat': the map's back wall 0.3 m thicker inwards
    sink = float(opts.get('sink', 0.0))          # m/s the vehicles sink at (traffic.py v10 in Gazebo: 0.0098)
    sunk_parts = {}
    rng = np.random.default_rng(seed)
    D = dict(t=[], ranges=[], laser=[], robot=[], plans=[], traffic=[], guard=[])

    spec = importlib.util.spec_from_file_location('pg', path)
    PG = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(PG)
    for k, v in sets.items():
        setattr(PG, k, v)
    grid = occupancy()
    if occ_extra == 'wallfat':
        j0 = int((15.6 - OY) / RES)
        grid[j0:, :] = 100
    if occ_extra == 'nowall':                    # the map has no back wall: its points are 'off the map'
        j0 = int((15.85 - OY) / RES)
        grid[j0:, :] = 0
    core = PG.GuardCore(PG.StaticMap(W, H, RES, OX, OY, grid.ravel().tolist()))

    if fov >= 2 * math.pi - 1e-6:
        a_min, a_inc = -math.pi, 2 * math.pi / beams
    else:
        a_min, a_inc = -fov / 2, fov / (beams - 1)
    rel = a_min + a_inc * np.arange(beams)
    r_min = 0.12

    # robot path: laps x 3 goals, each a dense polyline driven at `speed`, 1 s pause at each goal
    segs = polyline_route()
    plan_pts = []          # (t_start, t_end, dense pts, goal index)
    t = phase
    for lap in range(laps):
        for gi, pts in segs:
            d = round_corners(pts)
            L = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(d, axis=0).T))))
            plan_pts.append((t, t + L[-1] / speed, d, L, gi, lap + 1))
            t += L[-1] / speed + 1.0
    t_end = t

    def robot_at(tt):
        for t0, t1, d, L, gi, lap in plan_pts:
            if tt <= t1 + 1.0:
                s = min(max(tt - t0, 0.0), t1 - t0) * speed
                i = int(np.searchsorted(L, s))
                i = min(max(i, 1), len(d) - 1)
                f = (s - L[i - 1]) / max(L[i] - L[i - 1], 1e-9)
                p = d[i - 1] + f * (d[i] - d[i - 1])
                j = min(i + 4, len(d) - 1)
                h = math.atan2(d[j][1] - d[i - 1][1], d[j][0] - d[i - 1][0])
                moving = t0 <= tt <= t1
                plan = d[i - 1:] if len(d) - i >= 2 else None
                return float(p[0]), float(p[1]), h, (speed if moving else 0.0), plan, gi, lap
        d = plan_pts[-1][2]
        return float(d[-1][0]), float(d[-1][1]), 0.0, 0.0, None, 2, laps

    drivers = {n: TR.Driver(n) for n in TR.VEHICLES}
    for n, d in drivers.items():
        sp = TR.VEHICLES[n]
        d.pose = (sp['start'][0], sp['start'][1], sp['yaw0'])
    wps = {n: TR.person_waypoints(sp) for n, sp in TR.PEOPLE.items()}

    dt_ctrl = 1.0 / TR.CONTROL_HZ
    t = 0.0
    next_scan = phase
    hist = collections.deque()       # (t, robot pose) for TF lag
    rows = []
    stage = {n: collections.Counter() for n in TR.VEHICLES}
    while t < t_end:
        rx, ry, ryaw, rv, plan, gi, lap = robot_at(t)
        people = {n: TR.person_at(wps[n], t) for n in wps}
        wx, wy, wvx = worker(t)
        ppl_xy = [(p[0], p[1]) for p in people.values()] + [(wx, wy)]
        for n, d in drivers.items():
            v, w = d.step(t, dt_ctrl, (rx, ry, ryaw) if t >= phase else None, ppl_xy)
            x, y, a = d.pose
            d.pose = (x + v * math.cos(a) * dt_ctrl, y + v * math.sin(a) * dt_ctrl, TR.wrap(a + w * dt_ctrl))
        hist.append((t, (rx, ry, ryaw)))
        while hist and t - hist[0][0] > 2.0:
            hist.popleft()
        if t + 1e-9 >= next_scan:
            next_scan += 1.0 / hz
            ang = rel + ryaw
            dx, dy = np.cos(ang), np.sin(ang)
            best = np.full(beams, np.inf)
            for b in BOXES.values():
                best = np.minimum(best, ray_aabb(rx, ry, dx, dy, b, r_min))
            for n, d in drivers.items():
                if n in opts.get('invis', '').split(','):
                    continue
                x, y, a = d.pose
                parts = None
                if sink:
                    key = (n, round(sink * t, 2))
                    if key not in sunk_parts:
                        sunk_parts[key] = parts_at_plane(n, LIDAR_Z + key[1])
                    parts = sunk_parts[key]
                for wx2, wy2, a2, sx, sy in part_polys(n, x, y, a, parts):
                    best = np.minimum(best, ray_obb(rx, ry, dx, dy, wx2, wy2, a2, sx, sy, r_min))
            legs = []
            for n, p in people.items():
                legs += legs_of(p[0], p[1], p[3] * math.cos(p[2]), p[3] * math.sin(p[2]), t)
            legs += legs_of(wx, wy, wvx, 0.0, t)
            for cx, cy in legs:
                best = np.minimum(best, ray_circle(rx, ry, dx, dy, cx, cy, 0.07, r_min))
            r = best + rng.normal(0.0, noise, beams)
            r[best >= rmax] = np.inf
            # the laser pose the guard is given (TF lag: an older pose)
            lp = (rx, ry, ryaw)
            if lag > 0:
                for th, pose in hist:
                    if th <= t - lag:
                        lp = pose
            hide = core.scan(r, a_min, a_inc, r_min, rmax, lp, t, lp, plan)
            core.decide(lp[:2], lp[2], rv, 0.8, t, plan)
            n_fol = sum(1 for v in core.vehicles if v.moving or v.recently_moved(t))
            if dump:
                D['t'].append(t); D['ranges'].append(r.astype(np.float32)); D['laser'].append(lp); D['robot'].append(lp)
                D['odom'] = D.get('odom', []) + [rv]
                if plan is not None and (not D['plans'] or t - D['plans'][-1][0] >= 1.0):
                    D['plans'].append((t, np.asarray(plan, dtype=np.float32)))
                D['guard'].append((t, n_fol))
            near = core.vlook.near
            # per vehicle: truth, sight, and the pipeline stages
            pts_all = np.column_stack((rx + r * np.cos(ang), ry + r * np.sin(ang)))
            fin = np.isfinite(r) & (r < rmax)
            for n, d in drivers.items():
                x, y, a = d.pose
                vis = in_sight(n, x, y, a, rx, ry)
                mov = abs(d.v) >= 0.05
                dpart = dist_to_parts(pts_all[:, 0], pts_all[:, 1], n, x, y, a)
                on = fin & (dpart < 0.08)
                n_raw = int(on.sum())
                offm = ~core.map.blocked(pts_all[:, 0], pts_all[:, 1], 'scan')
                n_off = int((on & offm).sum())
                # the guard's track on this vehicle (its points mostly on it), this scan
                trk = None
                for v in core.vehicles:
                    if v.t == t and len(v.pts):
                        dd = dist_to_parts(v.pts[:, 0], v.pts[:, 1], n, x, y, a)
                        if np.mean(dd < 0.15) > 0.5:
                            trk = v
                            break
                true_d = float(np.min(dist_to_parts(np.array([rx]), np.array([ry]), n, x, y, a)))
                this_one = near < 9.0 and abs(near - true_d) <= 0.5
                rows.append(dict(t=t, name=n, vis=vis, mov=mov, fol=n_fol > 0, this=this_one,
                                 raw=n_raw, off=n_off, trk=trk is not None,
                                 hits=trk.hits if trk else 0, spd=trk.speed if trk else 0.0,
                                 chg=(trk is not None and trk.change_t is not None and t - trk.change_t <= PG.VEH_CHANGE_KEEP),
                                 moving=bool(trk and trk.moving), rec=bool(trk and trk.recently_moved(t)),
                                 vid=trk.id if trk else 0, vx=x, vy=y, v=d.v, rx=rx, ry=ry, lap=lap,
                                 trk_len=float(np.hypot(*np.ptp(trk.pts, axis=0))) if trk else 0.0))
        if dump and (not D['traffic'] or t - D['traffic'][-1][0] >= 0.1 - 1e-9):
            for n, d in drivers.items():
                x, y, a = d.pose
                D['traffic'].append((t, n, 'vehicle', x, y, a, d.v, d.phase))
            for n, p in people.items():
                D['traffic'].append((t, n, 'person', p[0], p[1], p[2], p[3], ''))
        t += dt_ctrl
    report(rows, hz)
    if dump:
        write_dump(dump, D, grid, a_min, a_inc, r_min, rmax, beams)
    return rows


def write_dump(label, D, grid, a_min, a_inc, r_min, rmax, beams):
    import json, csv as _csv
    runs = os.path.expanduser('~/runs')
    for sub in ('rec', 'traffic', 'guard'):
        os.makedirs(os.path.join(runs, sub), exist_ok=True)
    n = len(D['t'])
    meta = dict(version='test', label=label, frame='lidar_link', beams=beams, angle_min=a_min,
                angle_max=a_min + a_inc * (beams - 1), angle_increment=a_inc, range_min=r_min, range_max=rmax,
                laser_z=0.28)
    arrays = dict(t=np.asarray(D['t']), ranges=np.stack(D['ranges']), laser=np.asarray(D['laser']),
                  robot=np.asarray(D['robot']), exact=np.ones((n, 2), dtype=np.int8),
                  tf_age=np.zeros((n, 2), dtype=np.float32), recv=np.full(n, 0.02, dtype=np.float32),
                  wall=np.arange(n) * 0.17, odom_v=np.asarray(D['odom'], dtype=np.float32),
                  odom_age=np.zeros(n, dtype=np.float32), meta=np.array(json.dumps(meta)),
                  map_info=np.array(json.dumps(dict(width=W, height=H, resolution=RES, origin_x=OX, origin_y=OY,
                                                    origin_yaw=0.0))),
                  map_data=grid.ravel().astype(np.int8))
    if D['plans']:
        arrays['plan_t'] = np.asarray([p[0] for p in D['plans']])
        arrays['plan_n'] = np.asarray([len(p[1]) for p in D['plans']], dtype=np.int32)
        arrays['plan_xy'] = np.concatenate([p[1] for p in D['plans']])
    with open(os.path.join(runs, 'rec', label + '.npz'), 'wb') as f:
        np.savez_compressed(f, **arrays)
    with open(os.path.join(runs, 'traffic', label + '.csv'), 'w', newline='') as f:
        w = _csv.writer(f)
        w.writerow(['t', 'name', 'kind', 'x', 'y', 'yaw', 'v', 'state'])
        for t, nm, kind, x, y, a, v, st in D['traffic']:
            w.writerow([f'{t:.2f}', nm, kind, f'{x:.3f}', f'{y:.3f}', f'{a:.3f}', f'{v:.3f}', st])
    with open(os.path.join(runs, 'guard', label + '.csv'), 'w', newline='') as f:
        w = _csv.writer(f)
        w.writerow(['t', 'state', 'vehicles', 'cruise'])
        for t, nf in D['guard']:
            w.writerow([f'{t:.2f}', 'go', str(nf), '0.800'])
    print(f'dumped {n} scans as {label}')


def report(rows, hz):
    dt = 1.0 / hz
    for n in TR.VEHICLES:
        R = [r for r in rows if r['name'] == n]
        vis = [r for r in R if r['vis']]
        vm = [r for r in vis if r['mov']]
        if not vis:
            print(f'{n}: never in sight')
            continue
        f = lambda rs, k: 100.0 * sum(1 for r in rs if r[k]) / max(len(rs), 1)
        print(f'{n:8s}: in sight {len(vis) * dt:5.0f} s ({len(vm) * dt:4.0f} s moving); followed {f(vis, "fol"):3.0f} % '
              f'({f(vm, "fol"):3.0f} % while moving; this one {f(vm, "this"):3.0f} %)')
        if vm:
            st = [('lidar hits >= 4', lambda r: r['raw'] >= 4), ('off-map hits >= 4', lambda r: r['off'] >= 4),
                  ('a guard track on it', lambda r: r['trk']), ('track hits >= 5', lambda r: r['hits'] >= 5),
                  ('track speed > 0.3', lambda r: r['spd'] > 0.3), ('free-space change', lambda r: r['chg']),
                  ('track moving', lambda r: r['moving']), ('moving or recent', lambda r: r['moving'] or r['rec'])]
            print('          while in sight and moving: ' + '; '.join(
                f'{k} {100.0 * sum(1 for r in vm if fn(r)) / len(vm):3.0f} %' for k, fn in st))
            ids = {r['vid'] for r in vm if r['vid']}
            print(f'          distinct guard tracks on it while moving in sight: {len(ids)}')


if __name__ == '__main__':
    main(sys.argv[1:])
