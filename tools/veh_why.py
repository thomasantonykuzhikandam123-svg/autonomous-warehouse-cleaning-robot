#!/usr/bin/env python3
"""veh_why.py (v10.3) - where does the guard lose the vehicles?

    python3 ~/ros2_ws/src/tools/veh_why.py <label> [guard.py]

Replays a scan_rec.py recording (~/runs/rec/<label>.npz) through the guard's own code, off-line
(default: people_guard.py next to this file), and follows each vehicle of the traffic
(~/runs/traffic/<label>.csv, its true pose) through the guard's vehicle pipeline, scan by scan:

  1. the lidar has returns on it (its parts at the lidar's height, from traffic.py's model)
  2. ... off the map (the guard ignores points within MAP_CLEAR of the map's obstacles)
  3. ... in a blob that is a vehicle candidate (long enough, or wide and solid; not too long)
  4. ... matched to a vehicle track
  5. ... seen in VEH_MIN_HITS scans
  6. ... its centre moving faster than VEH_MOVING
  7. ... with free-space evidence (its points changed against the scan VEH_EVIDENCE_DT ago)
  8. ... moving: the guard acts on it (give way, fork ground, speed cap)

First (v10.2), is the model where the lidar can see it at all? For every stretch the vehicle is
within 5 m, the model is cut at the lidar plane for each depth below (or above) the floor, raycast
beam by beam, and the depth the real beams fit best is reported: "on the floor", "x-y m below the
floor", or "not in the lidar plane at all" (the beams go through where it stands). tour_v101b's
recording showed traffic.py v10's vehicles sinking about 1 cm per second of sim time.

The share of scans that get to each step, while the vehicle is in sight within 5 m and moving
(tour_why's measure), shows the step that loses it. Then the longest stretch it was not followed,
step by step. With ~/runs/guard/<label>.csv it first checks that the replay counts the same
vehicles as the guard did in the run.

The camera's people (/eye/persons) are not recorded, so standing people are not replayed: that
changes nothing in the vehicle layer.
"""
import collections
import csv
import importlib.util
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

VERSION = 'v10.3'
RUNS = os.path.expanduser('~/runs')
TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

SIGHT_R = 5.0           # m: tour_why's "in sight"
NEAR_R = 8.0            # m: vehicles further than this are not followed through the steps
ON_PART = 0.12          # m: a lidar point this close to a vehicle's parts is a return from it
LIDAR_Z = 0.28          # m: lidar height if the recording doesn't say
DEPTHS = np.round(np.arange(-0.60, 2.62, 0.02), 2)   # m below the floor tried for each vehicle (- above)
FIT_TOL = 0.10          # m: a beam that ends this close to the predicted surface fits it
STRETCH_MAX = 10.0      # s: stretches in view are reported in pieces this long at most


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


def load_guard(path):
    spec = importlib.util.spec_from_file_location('people_guard_replay', path)
    pg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pg)
    return pg


def lidar_parts(TR, name, z):
    """Boxes of the vehicle's model that cross the lidar's plane: [(cx, cy, sx, sy)] in its frame."""
    out = []
    root = ET.fromstring(TR.vehicle_sdf(name, TR.VEHICLES[name]))
    for vis in root.iter('visual'):
        size = vis.find('geometry/box/size')
        pose = vis.find('pose')
        if size is None:
            continue
        sx, sy, sz = (float(v) for v in size.text.split())
        p = [float(v) for v in pose.text.split()] if pose is not None else [0.0] * 6
        if p[2] - sz / 2 <= z <= p[2] + sz / 2:
            out.append((p[0], p[1], sx, sy))
    return out


def model_boxes(TR, name):
    """Every box of the vehicle's model: [(name, cx, cy, z0, z1, sx, sy)] in its frame."""
    out = []
    root = ET.fromstring(TR.vehicle_sdf(name, TR.VEHICLES[name]))
    for vis in root.iter('visual'):
        size = vis.find('geometry/box/size')
        if size is None:
            continue
        sx, sy, sz = (float(v) for v in size.text.split())
        pose = vis.find('pose')
        p = [float(v) for v in pose.text.split()] if pose is not None else [0.0] * 6
        out.append((vis.get('name'), p[0], p[1], p[2] - sz / 2, p[2] + sz / 2, sx, sy))
    return out


def ray_box(ox, oy, th, cx, cy, yaw, sx, sy):
    """Range along the beams th from (ox, oy) into a box (centre, yaw, size); inf where they miss."""
    dx, dy = np.cos(th), np.sin(th)
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = c * (ox - cx) + s * (oy - cy), -s * (ox - cx) + c * (oy - cy)
    ldx, ldy = c * dx + s * dy, -s * dx + c * dy
    with np.errstate(divide='ignore', invalid='ignore'):
        t1x, t2x = (-sx / 2 - lx) / ldx, (sx / 2 - lx) / ldx
        t1y, t2y = (-sy / 2 - ly) / ldy, (sy / 2 - ly) / ldy
        tmin = np.maximum(np.minimum(t1x, t2x), np.minimum(t1y, t2y))
        tmax = np.minimum(np.maximum(t1x, t2x), np.maximum(t1y, t2y))
    hit = (tmax >= tmin) & (tmax > 0)
    return np.where(hit, np.where(tmin > 0, tmin, tmax), np.inf)


def depth_fit(boxes, ranges, th, laser, pose, rmin, rmax, z):
    """For each depth in DEPTHS: (beams ending at the model cut there, beams going through it)."""
    lx, ly = laser[0], laser[1]
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    rb = np.array([ray_box(lx, ly, th, x + c * bx - s * by, y + s * bx + c * by, yaw, sx, sy)
                   for _, bx, by, _, _, sx, sy in boxes])
    rm = np.where(np.isfinite(ranges) & (ranges >= rmin) & (ranges < rmax), ranges, np.inf)
    agree = np.zeros(len(DEPTHS))
    through = np.zeros(len(DEPTHS))
    for i, h in enumerate(DEPTHS):
        cut = np.array([z0 <= z + h <= z1 for _, _, _, z0, z1, _, _ in boxes])
        if not cut.any():
            continue
        rp = rb[cut].min(axis=0)
        m = np.isfinite(rp) & (rp > rmin) & (rp < rmax)
        with np.errstate(invalid='ignore'):                 # inf - inf where both miss
            agree[i] = np.sum(m & (np.abs(rm - rp) < FIT_TOL))
            through[i] = np.sum(m & (rm > rp + FIT_TOL))
    return agree, through


def depth_stretches(fits, dt):
    """fits: [(t, agree, through)] -> [(t0, t1, status, lo, hi, share_at_best, share_on_floor, beams)]."""
    out, cur = [], []
    for f in fits:
        if cur and (f[0] - cur[-1][0] > 3 * dt or f[0] - cur[0][0] > STRETCH_MAX):
            out.append(cur)
            cur = []
        cur.append(f)
    if cur:
        out.append(cur)
    i0 = int(np.argmin(np.abs(DEPTHS)))
    res = []
    for st in out:
        A = np.sum([f[1] for f in st], axis=0)
        Th = np.sum([f[2] for f in st], axis=0)
        beams0 = A[i0] + Th[i0]
        share0 = A[i0] / beams0 if beams0 else float('nan')
        score = A - 2 * Th
        best = float(score.max())
        if beams0 < 20:
            status, lo, hi, share_b = 'few beams', float('nan'), float('nan'), float('nan')
        elif best <= 0:
            status, lo, hi, share_b = 'gone', float('nan'), float('nan'), float('nan')
        else:
            good = DEPTHS[score >= 0.9 * best]
            lo, hi = float(good.min()), float(good.max())
            j = int(np.argmax(score))
            share_b = A[j] / max(A[j] + Th[j], 1)
            if lo <= 0.0 <= hi or share0 >= 0.8:
                status = 'floor'
            else:
                status = 'below' if lo > 0 else 'above'
        res.append((st[0][0], st[-1][0], status, lo, hi, share_b, share0, int(beams0)))
    return res


def dist_to_parts(px, py, parts, x, y, yaw):
    """Distance from points to a vehicle's parts at (x, y, yaw); 0 inside."""
    best = np.full(np.shape(px), np.inf)
    c, s = math.cos(yaw), math.sin(yaw)
    for cx, cy, sx, sy in parts:
        wx, wy = x + c * cx - s * cy, y + s * cx + c * cy
        lx = c * (px - wx) + s * (py - wy)
        ly = -s * (px - wx) + c * (py - wy)
        best = np.minimum(best, np.hypot(np.maximum(np.abs(lx) - sx / 2, 0.0),
                                         np.maximum(np.abs(ly) - sy / 2, 0.0)))
    return best


def blockers():
    """tour_why's line-of-sight blockers: racks and areas from lanes.py, shrunk 5 cm."""
    try:
        import lanes
    except Exception:
        return []
    ox, oy = lanes.MAP_OFFSET
    return [(x0 - ox + 0.05, y0 - oy + 0.05, x1 - ox - 0.05, y1 - oy - 0.05)
            for n, (x0, y0, x1, y1) in lanes.OBSTACLES.items() if not n.endswith('_wall')]


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


class Truth:
    """One vehicle's true pose from the traffic log, at any time (linear between rows)."""

    def __init__(self, rows):
        rows = sorted((fnum(r.get('t')), r) for r in rows if fnum(r.get('t')) == fnum(r.get('t')))
        self.t = np.array([t for t, _ in rows])
        self.x = np.array([fnum(r.get('x')) for _, r in rows])
        self.y = np.array([fnum(r.get('y')) for _, r in rows])
        self.yaw = np.array([fnum(r.get('yaw'), 0.0) for _, r in rows])
        self.v = np.array([fnum(r.get('v'), 0.0) for _, r in rows])
        self.state = [r.get('state') or '' for _, r in rows]

    def at(self, t):
        if not len(self.t):
            return None
        i = int(np.searchsorted(self.t, t))
        j = min(max(i, 1), len(self.t) - 1)
        a, b = j - 1, j
        if min(abs(self.t[a] - t), abs(self.t[b] - t)) > 0.3:
            return None
        if self.t[b] > self.t[a]:
            f = min(max((t - self.t[a]) / (self.t[b] - self.t[a]), 0.0), 1.0)
        else:
            f = 0.0
        k = a if abs(self.t[a] - t) <= abs(self.t[b] - t) else b
        return (self.x[a] + f * (self.x[b] - self.x[a]), self.y[a] + f * (self.y[b] - self.y[a]),
                self.yaw[k], self.v[k], self.state[k])


def candidate_why(pg, P, I, g, n_rays):
    """Why the guard's vehicle_candidates() would or would not take blob g: (length, fill, why)."""
    b = P[g]
    length = float(np.hypot(*np.ptp(b, axis=0))) if len(b) else 0.0
    rays = np.sort(I[g])
    gaps = np.diff(rays)
    if len(gaps) and gaps.max() > n_rays / 2:
        rays = np.sort((rays + n_rays // 2) % n_rays)
    span = int(rays[-1] - rays[0]) + 1 if len(rays) else 1
    fill = len(set(rays.tolist())) / max(span, 1)
    if len(g) < pg.VEH_MIN_PTS:
        why = 'few points'
    elif length > pg.VEH_MAX_LEN:
        why = 'merged, too long'
    elif length >= pg.VEH_MIN_LEN or (length >= pg.VEH_SOLID_W and fill >= pg.VEH_SOLID_FILL):
        why = ''
    elif length < pg.VEH_SOLID_W:
        why = 'short'
    else:
        why = 'gappy'
    return length, fill, why


STEPS = [('raw', 'the lidar has returns on it (>= 4)'),
         ('off', 'returns off the map (>= 4)'),
         ('cand', 'in a blob that is a vehicle candidate'),
         ('trk', 'matched to a vehicle track'),
         ('hits_ok', 'track seen in >= VEH_MIN_HITS scans'),
         ('spd_ok', 'track centre faster than VEH_MOVING'),
         ('chg', 'free-space change in the last VEH_CHANGE_KEEP'),
         ('mov', 'track moving: the guard acts on it'),
         ('fol', 'the guard follows a vehicle (tour_why\'s measure)')]


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0].startswith('-'):
        print(__doc__)
        return 1
    label = argv[0]
    guard_path = argv[1] if len(argv) > 1 else os.path.join(TOOLS, 'people_guard.py')
    rec_path = os.path.join(RUNS, 'rec', f'{label}.npz')
    if not os.path.exists(rec_path):
        print(f'no {rec_path} - was scan_rec.py running during the {label} run?')
        return 1
    import traffic as TR
    pg = load_guard(guard_path)
    R = np.load(rec_path, allow_pickle=False)
    meta = json.loads(str(R['meta']))
    if 'map_info' not in R.files:
        print('the recording has no /map (was the map server up?) - cannot replay the guard')
        return 1
    mi = json.loads(str(R['map_info']))
    smap = pg.StaticMap(mi['width'], mi['height'], mi['resolution'], mi['origin_x'], mi['origin_y'],
                        R['map_data'])
    T, RG, LS, RB = R['t'], R['ranges'], R['laser'], R['robot']
    EX, OV, OA = R['exact'], R['odom_v'], R['odom_age']
    N, B = RG.shape
    amin, ainc = float(meta['angle_min']), float(meta['angle_increment'])
    rmin, rmax = float(meta['range_min']), float(meta['range_max'])
    plans = []
    if 'plan_t' in R.files:
        off = np.concatenate(([0], np.cumsum(R['plan_n'])))
        plans = [(float(R['plan_t'][k]), R['plan_xy'][off[k]:off[k + 1]]) for k in range(len(R['plan_t']))]
    plan_ts = np.array([p[0] for p in plans]) if plans else np.zeros(0)

    z = fnum(meta.get('laser_z'), LIDAR_Z)
    z_used = z if 0.1 <= z <= 1.5 else LIDAR_Z
    traffic = read_csv(os.path.join(RUNS, 'traffic', f'{label}.csv'))
    names = sorted({r.get('name') for r in traffic if r.get('kind') == 'vehicle' and r.get('name') in TR.VEHICLES})
    if not names:
        print(f'no vehicles in ~/runs/traffic/{label}.csv - was the traffic running under this label?')
        return 1
    truth = {n: Truth([r for r in traffic if r.get('name') == n]) for n in names}
    parts = {n: lidar_parts(TR, n, z_used) for n in names}
    boxes_all = {n: model_boxes(TR, n) for n in names}
    fits = {n: [] for n in names}
    zlog = {n: [fnum(r.get('z')) for r in traffic if r.get('name') == n and r.get('z', '') != ''] for n in names}
    glog = {}
    for r in read_csv(os.path.join(RUNS, 'guard', f'{label}.csv')):
        tt = fnum(r.get('t'))
        if tt == tt:
            glog[round(tt, 2)] = r
    boxes = blockers()

    # ---- replay
    core = pg.GuardCore(smap)
    rows = {n: [] for n in names}
    agree = [0, 0]
    gfol = {}
    prev = None
    v_est = 0.0
    for k in range(N):
        t = float(T[k])
        laser, robot = tuple(LS[k]), tuple(RB[k])
        ranges = RG[k]
        plan = None
        if len(plan_ts):
            i = int(np.searchsorted(plan_ts, t, side='right')) - 1
            if i >= 0 and t - plan_ts[i] <= pg.PLAN_AGE:
                plan = [tuple(p) for p in plans[i][1]]
        core.scan(ranges, amin, ainc, rmin, rmax, laser, t, robot, plan)
        if OA[k] < 0.5:
            v_est = float(OV[k])
        elif prev is not None and t > prev[0]:
            v_est = 0.5 * v_est + 0.5 * math.hypot(robot[0] - prev[1], robot[1] - prev[2]) / (t - prev[0])
        prev = (t, robot[0], robot[1])
        g = glog.get(round(t, 2))
        cruise = fnum(g.get('cruise'), 0.8) if g is not None else 0.8
        core.decide(robot[:2], robot[2], v_est, cruise if cruise == cruise else 0.8, t, plan)
        n_fol = sum(1 for v in core.vehicles if v.moving or v.recently_moved(t))
        if g is not None and g.get('vehicles', '') != '':
            agree[0] += int(fnum(g.get('vehicles'), -1) == n_fol)
            agree[1] += 1
            gfol[round(t, 2)] = fnum(g.get('vehicles'), 0) > 0
        # ---- each vehicle through the steps
        near = [n for n in names if truth[n].at(t) is not None and
                math.hypot(truth[n].at(t)[0] - robot[0], truth[n].at(t)[1] - robot[1]) <= NEAR_R]
        if not near:
            continue
        th = laser[2] + amin + ainc * np.arange(B)
        for n in near:                  # how deep is the model? (only within SIGHT_R: enough beams)
            tp = truth[n].at(t)
            if math.hypot(tp[0] - laser[0], tp[1] - laser[1]) <= SIGHT_R:
                a_, th_ = depth_fit(boxes_all[n], ranges.astype(float), th, laser, tp[:3], rmin, rmax, z_used)
                fits[n].append((t, a_, th_))
        pts, idx = pg.scan_points(ranges, amin, ainc, rmin, rmax, laser)
        offm = ~smap.blocked(pts[:, 0], pts[:, 1], 'scan') if len(pts) else np.zeros(0, dtype=bool)
        P, I = pts[offm], idx[offm]
        groups = pg.blobs(P) if len(P) else []
        cands = pg.vehicle_candidates(P, I, B) if len(P) else []
        cand_sets = [set(ci.tolist()) for _, ci in cands]
        vl = core.vlook
        for n in near:
            x, y, yaw, v, state = truth[n].at(t)
            sp = TR.VEHICLES[n]
            corner = [(x, y)] + TR.corners(x, y, yaw, sp['L'], sp['W'])
            sight = any(math.hypot(px - robot[0], py - robot[1]) <= SIGHT_R and
                        not any(crosses(robot[0], robot[1], px, py, bx) for bx in boxes) for px, py in corner)
            d_all = dist_to_parts(pts[:, 0], pts[:, 1], parts[n], x, y, yaw) if len(pts) else np.zeros(0)
            on = d_all < ON_PART
            n_raw, n_off = int(on.sum()), int((on & offm).sum())
            on_off = on[offm]
            blob_len = blob_fill = 0.0
            why = 'no points'
            cand = False
            if n_off:
                gi = max(range(len(groups)), key=lambda j: int(on_off[groups[j]].sum()))
                blob_len, blob_fill, why = candidate_why(pg, P, I, groups[gi], B)
                mine = set(I[on_off].tolist())
                cand = any(len(mine & cs) >= 0.5 * len(mine) for cs in cand_sets)
                if cand:
                    why = ''
                elif not why:
                    why = 'not a candidate'
            trk = None
            for veh in core.vehicles:
                if veh.t == t and len(veh.pts):
                    dd = dist_to_parts(veh.pts[:, 0], veh.pts[:, 1], parts[n], x, y, yaw)
                    if np.mean(dd < 0.15) >= 0.5:
                        trk = veh
                        break
            body = [(0.0, 0.0, sp['L'], sp['W'])]       # tour_why's 'this one': the body box
            true_d = float(np.min(dist_to_parts(np.array([robot[0]]), np.array([robot[1]]), body, x, y, yaw)))
            rows[n].append(dict(
                t=t, sight=sight, moving=abs(v) >= 0.05, x=x, y=y, v=v, state=state,
                rx=robot[0], ry=robot[1], rv=v_est, d=true_d,
                raw=n_raw >= 4, off=n_off >= 4, n_raw=n_raw, n_off=n_off, cand=cand, why=why,
                blob_len=blob_len, blob_fill=blob_fill, trk=trk is not None,
                tid=trk.id if trk else 0, hits=trk.hits if trk else 0,
                spd=trk.speed if trk else 0.0,
                hits_ok=bool(trk is not None and trk.hits >= pg.VEH_MIN_HITS),
                spd_ok=bool(trk is not None and trk.speed > pg.VEH_MOVING),
                chg=bool(trk is not None and trk.change_t is not None and t - trk.change_t <= pg.VEH_CHANGE_KEEP),
                mov=bool(trk is not None and trk.moving), rec=bool(trk is not None and trk.recently_moved(t)),
                fol=n_fol > 0, this=vl.near < 9.0 and abs(vl.near - true_d) <= 0.5,
                gfol=gfol.get(round(t, 2))))

    if os.environ.get('VEH_WHY_ROWS'):              # every vehicle's rows, for analysis (JSON lines)
        with open(os.environ['VEH_WHY_ROWS'], 'w') as f:
            for n in names:
                for r in rows[n]:
                    f.write(json.dumps(dict({k: (v.item() if isinstance(v, np.generic) else v)
                                             for k, v in r.items() if k != 'gfol'}, name=n)) + '\n')
    # ---- report
    dt = float(np.median(np.diff(T))) if N > 1 else 0.1
    span = math.degrees(ainc * (B - 1))
    print(f'=== veh_why: {label} - where the guard loses the vehicles (replayed through '
          f'{os.path.basename(guard_path)} {getattr(pg, "VERSION", "?")})')
    print(f'recording: {N} scans over {T[-1] - T[0]:.0f} s of sim time ({1.0 / dt:.1f} per s); lidar '
          f'{meta.get("frame", "?")}: {B} beams, {math.degrees(amin):.0f} to {math.degrees(amin) + span:.0f} deg, '
          f'{rmin:.2f}-{rmax:.1f} m, {z:.2f} m up' + ('' if z_used == z else f' (used {z_used:.2f})'))
    print(f'  TF at the scan\'s own stamp: laser {100.0 * EX[:, 0].mean():.0f} %, robot {100.0 * EX[:, 1].mean():.0f} % '
          f'(else the latest, as the guard does); scan reached the recorder {np.median(R["recv"]):.2f} s after its stamp '
          f'(median), {np.percentile(R["recv"], 95):.2f} s (95 %)')
    print(f'  map: {mi["width"]} x {mi["height"]} cells at {mi["resolution"]:.3f} m, origin ({mi["origin_x"]:.2f}, '
          f'{mi["origin_y"]:.2f}); vehicle parts at the lidar\'s height: ' +
          '; '.join(f'{n} {len(parts[n])}' for n in names))
    if agree[1]:
        print(f'  replay against the guard\'s own log: the same "vehicles" count in {100.0 * agree[0] / agree[1]:.0f} % '
              f'of {agree[1]} scans' + ('' if agree[0] >= 0.9 * agree[1] else
                                       '  <- LOW: the guard saw something the recording did not (dropped scans?)'))
    else:
        print('  (no guard log with vehicle columns to check the replay against)')
    verdicts = []
    for n in names:
        rs = rows[n]
        vis = [r for r in rs if r['sight']]
        vm = [r for r in vis if r['moving']]
        print()
        if not vis:
            print(f'--- {n}: never in sight within {SIGHT_R:.0f} m')
            continue
        gl = [r for r in vm if r['gfol'] is not None]
        print(f'--- {n}: in sight {len(vis) * dt:.0f} s, moving for {len(vm) * dt:.0f} s of it; the guard followed a '
              f'vehicle for {100.0 * sum(r["fol"] for r in vm) / max(len(vm), 1):.0f} % of that (replay)'
              + (f', {100.0 * sum(r["gfol"] for r in gl) / len(gl):.0f} % (guard log)' if gl else '')
              + f'; this one {100.0 * sum(r["this"] for r in vm) / max(len(vm), 1):.0f} %')
        stretches = depth_stretches(fits[n], dt)
        floor_bad = [s_ for s_ in stretches if s_[2] in ('below', 'above', 'gone')]
        print('  height from the beams (the model cut at the lidar plane at each depth, raycast; which fits):')
        for t0, t1, status, lo, hi, share_b, share0, beams in stretches:
            if status == 'floor':
                txt = f'on the floor ({100 * share0:.0f} % of {beams} beams fit)'
            elif status == 'gone':
                txt = f'NOT IN THE LIDAR PLANE AT ALL: {beams} beams went through where it is'
            elif status == 'few beams':
                txt = 'too few beams on it to tell'
            else:
                where = 'below' if status == 'below' else 'above'
                a_, b_ = (lo, hi) if status == 'below' else (-hi, -lo)
                txt = (f'{a_:.2f}-{b_:.2f} m {where.upper()} THE FLOOR ({100 * share_b:.0f} % fit there; '
                       f'on the floor {100 * share0:.0f} % of {beams} beams)')
            print(f'     t {t0:6.1f}-{t1:6.1f}: {txt}')
        if zlog[n]:
            print(f'  height in the traffic log (odometry): {min(zlog[n]):+.3f} to {max(zlog[n]):+.3f} m')
        else:
            print('  height in the traffic log: not logged (traffic.py before v10.2, 2D odometry)')
        if not vm:
            verdicts.append((n, (0.0, None), None, floor_bad, stretches))
            continue
        print('  while in sight and moving, the share of scans in which:')
        prev_share, worst = 100.0, (0.0, None)
        shares = {}
        for key, text in STEPS:
            share = 100.0 * sum(1 for r in vm if r[key]) / len(vm)
            shares[key] = share
            drop = prev_share - share
            if key != 'fol' and drop > worst[0]:
                worst = (drop, key)
            print(f'     {text:50s} {share:4.0f} %')
            if key != 'fol':
                prev_share = share
        whys = collections.Counter(r['why'] for r in vm if r['off'] and not r['cand'])
        if whys:
            tot = sum(whys.values())
            print('  with returns off the map but no candidate: ' + ', '.join(
                f'{w or "?"} {100.0 * c / tot:.0f} %' for w, c in whys.most_common()) +
                f' (blob length median {np.median([r["blob_len"] for r in vm if r["off"] and not r["cand"]]):.2f} m)')
        lost_map = [r for r in vm if r['raw'] and not r['off']]
        if lost_map:
            print(f'  returns on it but on the map (ignored as racks/walls) in {100.0 * len(lost_map) / len(vm):.0f} % '
                  f'of these scans, e.g. with it at ({lost_map[0]["x"]:.2f}, {lost_map[0]["y"]:.2f})')
        ids = [r['tid'] for r in vm if r['tid']]
        print(f'  tracks on it while it moved in sight: {len(set(ids))}; typical track speed '
              f'{np.median([r["spd"] for r in vm if r["trk"]]) if any(r["trk"] for r in vm) else 0.0:.2f} m/s '
              f'against its true {np.median([abs(r["v"]) for r in vm]):.2f} m/s')
        # the longest stretch in sight, moving, not followed: step by step
        best, cur, last_t = [], [], None
        for r in vis:
            if last_t is not None and r['t'] - last_t > 3 * dt:
                cur = []                              # out of sight in between: a new stretch
            last_t = r['t']
            if not r['fol']:
                cur.append(r)
                if len(cur) > len(best):
                    best = list(cur)
            else:
                cur = []
        if best:
            step = max(0.5, round((best[-1]['t'] - best[0]['t']) / 25.0, 1))
            print(f'  longest in sight without the guard following: {len(best) * dt:.1f} s from t {best[0]["t"]:.1f}; '
                  f'every {step:.1f} s:')
            print(f'  {"t":>7s} {"robot x":>7s} {"y":>6s} {"v":>4s} | {n[:8] + " x":>10s} {"y":>6s} {"v":>5s} '
                  f'{"d":>5s} | {"hits":>4s} {"off":>4s} {"blob":>5s} {"fill":>4s} {"cand":>14s} | '
                  f'{"track":>5s} {"hits":>4s} {"speed":>5s} {"chg":>3s} {"mov":>3s}')
            nxt = -1.0
            for r in best:
                if r['t'] < nxt:
                    continue
                nxt = r['t'] + step
                cand = 'yes' if r['cand'] else (r['why'] or '-')
                print(f'  {r["t"]:7.1f} {r["rx"]:7.2f} {r["ry"]:6.2f} {r["rv"]:4.2f} | {r["x"]:10.2f} {r["y"]:6.2f} '
                      f'{abs(r["v"]):5.2f} {r["d"]:5.2f} | {r["n_raw"]:4d} {r["n_off"]:4d} {r["blob_len"]:5.2f} '
                      f'{r["blob_fill"]:4.2f} {cand[:14]:>14s} | {r["tid"] or "":>5} {r["hits"]:4d} {r["spd"]:5.2f} '
                      f'{"y" if r["chg"] else "-":>3s} {"y" if r["mov"] else "-":>3s}')
        verdicts.append((n, worst, shares, floor_bad, stretches))
    print()
    for n, (drop, key), shares, floor_bad, stretches in verdicts:
        text = dict(STEPS).get(key, '')
        judged = [s_ for s_ in stretches if s_[2] != 'few beams']
        if floor_bad:
            first = floor_bad[0]
            print(f'VERDICT {n}: NOT WHERE THE LIDAR CAN SEE IT in {len(floor_bad)} of {len(judged)} stretches in view, '
                  f'from t {first[0]:.0f} ({"not in the lidar plane at all" if first[2] == "gone" else "the model sits " + first[2] + " the floor"}). '
                  f'That is the traffic, not the guard (traffic.py before v10.2 let the vehicles sink about '
                  f'1 cm/s); the steps above measure a model the lidar could not see.')
            continue
        if shares is None:
            print(f'VERDICT {n}: on the floor in view, but it never moved in sight: nothing to follow.')
            continue
        if shares['mov'] >= 80.0 or key is None:
            print(f'VERDICT {n}: the vehicle layer works on these scans: its track was moving in '
                  f'{shares["mov"]:.0f} % of the time it moved in sight.')
        else:
            print(f'VERDICT {n}: lost at "{text}": {drop:.0f} points of the time it moved in sight drop out '
                  f'there (its track was moving in only {shares["mov"]:.0f} % of that time).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
