#!/usr/bin/env python3
"""perception_audit.py (v10.11) - what the robot knew against where everyone really was.

    python3 perception_audit.py <people_guard.py> <runs folder> <label> [--json out.json]
    e.g.  python3 ~/ros2_ws/src/tools/lab/perception_audit.py ~/ros2_ws/src/tools/people_guard.py ~/runs shift_v108a

The recording's scans are run through the guard again (open loop, as replay mode does: lidar only, no
camera). At every scan each person and vehicle in the warehouse, from the simulator's truth (traffic.py's
log, and the world's worker, whose loop is fixed), is put in one of three classes from the robot's own
lidar: out of range (over 8 m), hidden (in range, but the beams towards it stop short: a rack, a
person, a vehicle in between), or in sight (a beam ends on it). Then: was the guard tracking it then
(a fresh track within 0.75 m; 1.5 m of a vehicle's centre), remembering it (a track it lost in the last
5 s), or did it not know? And per encounter (8 m and closer, gaps under 2 s joined): how far away it
was when the guard first tracked it, and whether it was first in sight already closer than 3 m (it came
out from behind something). The robot never gets the truth: this is the answer to 'is the page showing
things the robot cannot know?'.
"""
import csv
import importlib.util
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
RANGE = 8.0
P_R, P_MATCH, V_MATCH = 0.25, 0.75, 1.5
BINS = [(0.0, 2.0), (2.0, 4.0), (4.0, 6.0), (6.0, 8.0)]
VEH = {'tugger': (1.3, 0.7), 'forklift': (1.2, 1.0)}


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def worker(t):                                    # the world's worker: a fixed 6 s loop (tourrep.worker)
    wp = [(0.0, -1.6), (2.2, 0.6), (3.0, 0.6), (5.2, -1.6), (6.0, -1.6)]
    p = t % 6.0
    for (t0, x0), (t1, x1) in zip(wp, wp[1:]):
        if t0 <= p <= t1:
            return x0 + (p - t0) / (t1 - t0) * (x1 - x0), 4.0, abs(x1 - x0) / (t1 - t0)
    return -1.6, 4.0, 0.0


def truth_table(path):
    by = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                by.setdefault(r['name'], ([], [], [], [], [], r['kind']))
                T, X, Y, Yw, V, _ = by[r['name']]
                T.append(float(r['t'])), X.append(float(r['x'])), Y.append(float(r['y'])), Yw.append(float(r['yaw']))
                V.append(abs(float(r['v'] or 0.0)))
            except (ValueError, KeyError):
                pass
    return {n: (np.array(T), np.array(X), np.array(Y), np.array(Yw), np.array(V), k)
            for n, (T, X, Y, Yw, V, k) in by.items()}


def at(tab, t):
    T, X, Y, Yw, V, _ = tab
    i = int(np.searchsorted(T, t))
    if i <= 0 or i >= len(T) or T[i] - T[i - 1] > 1.0:
        return None
    f = (t - T[i - 1]) / (T[i] - T[i - 1])
    return X[i - 1] + f * (X[i] - X[i - 1]), Y[i - 1] + f * (Y[i] - Y[i - 1]), Yw[i], max(V[i - 1], V[i])


def sight(ranges, a_min, a_inc, rmax, lp, x, y, half):
    """'in sight' / 'hidden' / 'out of range' from the beams towards a body of half-width `half`."""
    d = math.hypot(x - lp[0], y - lp[1])
    if d > RANGE:
        return 'out', d
    n = len(ranges)
    b = math.atan2(y - lp[1], x - lp[0]) - lp[2]
    span = math.atan2(half, max(d, 0.3))
    i0 = int(math.floor((b - span - a_min) / a_inc))
    i1 = int(math.ceil((b + span - a_min) / a_inc))
    hit = False
    for i in range(i0, i1 + 1):
        r = float(ranges[i % n])
        if math.isfinite(r) and r < rmax * 0.995 and d - half - 0.35 <= r <= d + half + 0.25:
            hit = True
            break
    return ('in sight' if hit else 'hidden'), d


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    PG = load(os.path.abspath(os.path.expanduser(argv[0])), 'pg_audit')
    runs, label = os.path.expanduser(argv[1]), argv[2]
    out_json = argv[argv.index('--json') + 1] if '--json' in argv else None
    Z = np.load(os.path.join(runs, 'rec', f'{label}.npz'), allow_pickle=False)
    meta = json.loads(str(Z['meta']))
    mi = json.loads(str(Z['map_info']))
    a_min, a_inc = float(meta['angle_min']), float(meta['angle_increment'])
    r_min, r_max = float(meta['range_min']), float(meta['range_max'])
    T, RG, LS, RB, OV, OA = Z['t'], Z['ranges'], Z['laser'], Z['robot'], Z['odom_v'], Z['odom_age']
    off = np.concatenate(([0], np.cumsum(Z['plan_n'])))
    plan_ts = np.asarray(Z['plan_t'], dtype=float)
    truth = truth_table(os.path.join(runs, 'traffic', f'{label}.csv'))
    names = sorted(truth) + ['worker']
    core = PG.GuardCore(PG.StaticMap(mi['width'], mi['height'], mi['resolution'], mi['origin_x'],
                                     mi['origin_y'], Z['map_data']))
    core.guard.prog_time = 25.0
    prev, v_est = None, 0.0
    cls_n = {c: 0 for c in ('in sight', 'hidden', 'out')}
    seen_bin = {b: [0, 0] for b in BINS}             # moving people in sight: [tracked, all]
    still_bin = {b: [0, 0] for b in BINS}            # people standing (under 0.2 m/s) in sight
    per = {}                                         # name -> [in sight, tracked while in sight]
    vmov, vstill = [0, 0], [0, 0]                    # vehicles within 8 m, moving / standing: tracked, all
    hid = [0, 0, 0]                                  # people hidden in range: tracked, remembered, all
    veh = [0, 0]                                     # vehicles in sight: tracked, all
    knows, near_n, false_tr = [], [], 0
    enc = {}                                         # name -> open encounter
    encounters = []
    n_scans = 0
    for k in range(len(T)):
        t = float(T[k])
        laser, robot = tuple(LS[k]), tuple(RB[k])
        i = int(np.searchsorted(plan_ts, t, side='right')) - 1
        plan = ([tuple(p) for p in Z['plan_xy'][off[i]:off[i + 1]]]
                if i >= 0 and t - plan_ts[i] <= PG.PLAN_AGE else None)
        if OA[k] < 0.5:
            v_est = float(OV[k])
        elif prev is not None and t > prev[0]:
            v_est = 0.5 * v_est + 0.5 * math.hypot(robot[0] - prev[1], robot[1] - prev[2]) / (t - prev[0])
        prev = (t, robot[0], robot[1])
        core.scan(RG[k], a_min, a_inc, r_min, r_max, laser, t, robot, plan)
        core.decide(robot[:2], robot[2], v_est, 0.8, t, plan)
        fresh = [(tr.px, tr.py) for tr in core.people if not tr.lost and t - tr.t <= 0.3]
        memory = [(tr.px, tr.py) for tr in core.people if tr.lost or t - tr.t > 0.3]
        vts = [(v.cx, v.cy) for v in core.vehicles if t - getattr(v, 't', t) <= 0.5]
        used = set()
        known, near = 0, 0
        n_scans += 1
        for nm in names:
            if nm == 'worker':
                x, y, spd = worker(t)
                kind, half = 'person', P_R
            else:
                p = at(truth[nm], t)
                if p is None:
                    continue
                x, y, _, spd = p
                kind = truth[nm][5]
                half = P_R if kind == 'person' else max(VEH.get(nm, (1.2, 1.0))) / 2
            c, d = sight(RG[k], a_min, a_inc, r_max, laser, x, y, half)
            cls_n[c] += 1
            if kind == 'person':
                j = min(range(len(fresh)), key=lambda q: math.hypot(fresh[q][0] - x, fresh[q][1] - y), default=None)
                tracked = j is not None and math.hypot(fresh[j][0] - x, fresh[j][1] - y) <= P_MATCH
                if tracked:
                    used.add(j)
                remembered = any(math.hypot(mx - x, my - y) <= 1.5 for mx, my in memory)
            else:
                tracked = any(math.hypot(vx - x, vy - y) <= V_MATCH for vx, vy in vts) or \
                    any(math.hypot(fx - x, fy - y) <= V_MATCH for fx, fy in fresh)
                remembered = False
            known += tracked
            if c != 'out':
                near += 1
                if kind == 'person' and c == 'in sight':
                    pa = per.setdefault(nm, [0, 0])
                    pa[0] += 1
                    pa[1] += tracked
                    for b in BINS:
                        if b[0] <= d < b[1]:
                            sb = seen_bin[b] if spd > 0.2 else still_bin[b]
                            sb[0] += tracked
                            sb[1] += 1
                elif kind == 'person':
                    hid[0] += tracked
                    hid[1] += (not tracked) and remembered
                    hid[2] += 1
                else:
                    veh[0] += tracked
                    veh[1] += 1
                    vv = vmov if spd > 0.2 else vstill
                    vv[0] += tracked
                    vv[1] += 1
                e = enc.get(nm)
                if e is None or t - e['t_last'] > 2.0:
                    if e is not None:
                        encounters.append(e)
                    e = enc[nm] = {'who': nm, 'kind': kind, 't0': t, 't_last': t, 'first_sight_d': None,
                                   'first_track_d': None, 'first_track_t': None, 'min_d': d}
                e['t_last'], e['min_d'] = t, min(e['min_d'], d)
                if c == 'in sight' and e['first_sight_d'] is None:
                    e['first_sight_d'], e['first_sight_t'] = d, t
                if tracked and e['first_track_d'] is None:
                    e['first_track_d'], e['first_track_t'] = d, t
        vpos = [p[:2] for p in (at(truth[n], t) for n in truth if truth[n][5] != 'person') if p is not None]
        false_tr += sum(1 for q in range(len(fresh)) if q not in used and
                        not any(math.hypot(fresh[q][0] - vx, fresh[q][1] - vy) <= V_MATCH for vx, vy in vpos))
        knows.append(known)
        near_n.append(near)
    encounters += [e for e in enc.values()]
    total = len(names)
    n_all = sum(cls_n.values())
    res = {
        'label': label, 'guard': getattr(PG, 'VERSION', '?'), 'scans': n_scans,
        'sim_s': round(float(T[-1] - T[0]), 1), 'agents': names,
        'share_out_of_range': round(cls_n['out'] / n_all, 3),
        'share_hidden_of_in_range': round(cls_n['hidden'] / max(1, cls_n['hidden'] + cls_n['in sight']), 3),
        'people_in_sight_tracked_by_distance': {f'{a:.0f}-{b:.0f} m': (round(v[0] / v[1], 3) if v[1] else None, v[1])
                                                for (a, b), v in seen_bin.items()},
        'standing_people_in_sight_tracked_by_distance': {
            f'{a:.0f}-{b:.0f} m': (round(v[0] / v[1], 3) if v[1] else None, v[1]) for (a, b), v in still_bin.items()},
        'per_person_in_sight_tracked': {n: (round(v[1] / v[0], 3), v[0]) for n, v in sorted(per.items())},
        'vehicles_moving_tracked': (round(vmov[0] / vmov[1], 3) if vmov[1] else None, vmov[1]),
        'vehicles_standing_tracked': (round(vstill[0] / vstill[1], 3) if vstill[1] else None, vstill[1]),
        'people_hidden_in_range': {'tracked': hid[0], 'remembered_only': hid[1], 'all': hid[2]},
        'vehicles_in_range_tracked': round(veh[0] / veh[1], 3) if veh[1] else None, 'vehicle_scans_in_range': veh[1],
        'known_of_total_mean': round(float(np.mean(knows)), 2), 'total_agents': total,
        'within_8m_mean': round(float(np.mean(near_n)), 2),
        'known_hist': {str(n): int(sum(1 for q in knows if q == n)) for n in range(total + 1)},
        'false_person_tracks_per_scan': round(false_tr / max(1, n_scans), 3),
    }
    pe = [e for e in encounters if e['kind'] == 'person' and e['min_d'] < 6.0]
    ft = [e['first_track_d'] for e in pe if e['first_track_d'] is not None]
    surprise = [e for e in pe if e['first_sight_d'] is not None and e['first_sight_d'] < 3.0]
    lat = [e['first_track_t'] - e['first_sight_t'] for e in pe
           if e['first_track_t'] is not None and e['first_sight_d'] is not None]
    res['person_encounters'] = len(pe)
    res['first_tracked_at_m'] = {'median': round(float(np.median(ft)), 2) if ft else None,
                                 'min': round(float(min(ft)), 2) if ft else None}
    res['first_in_sight_closer_than_3m'] = len(surprise)
    res['track_after_sight_s'] = {'median': round(float(np.median(lat)), 2) if lat else None,
                                  'p90': round(float(np.percentile(lat, 90)), 2) if lat else None}
    res['never_tracked'] = [e['who'] for e in pe if e['first_track_d'] is None and e['first_sight_d'] is not None]
    res['hidden_whole_encounter'] = len([e for e in pe if e['first_sight_d'] is None])
    print(f"perception_audit: {label}, guard {res['guard']}, {n_scans} scans over {res['sim_s']} s of sim time; "
          f"truth for {total}: {', '.join(names)} (the robot never gets it)")
    print(f"  of every person and vehicle at every scan: {res['share_out_of_range']:.0%} more than 8 m away; "
          f"of those within 8 m, {res['share_hidden_of_in_range']:.0%} hidden from the lidar")
    print('  people walking (over 0.2 m/s) in sight, tracked by the guard: ' + ', '.join(
        f"{b} {v[0]:.0%} (n {v[1]})" if v[0] is not None else f'{b} -' for b, v in
        res['people_in_sight_tracked_by_distance'].items()))
    print('  people standing in sight, tracked from the lidar alone (live, the camera confirms them): ' + ', '.join(
        f"{b} {v[0]:.0%} (n {v[1]})" if v[0] is not None else f'{b} -' for b, v in
        res['standing_people_in_sight_tracked_by_distance'].items()))
    print('  each person, share of their in-sight scans tracked: ' + ', '.join(
        f"{n} {v[0]:.0%} of {v[1]}" for n, v in res['per_person_in_sight_tracked'].items()))
    print(f"  people hidden but within 8 m: tracked {hid[0]}, remembered only {hid[1]}, unknown "
          f"{hid[2] - hid[0] - hid[1]} of {hid[2]} scans")
    print(f"  vehicles within 8 m tracked: moving {res['vehicles_moving_tracked']}, standing "
          f"{res['vehicles_standing_tracked']} (tracked share, scans)")
    print(f"  the robot knew {res['known_of_total_mean']} of the {total} at a time on average "
          f"({res['within_8m_mean']} within 8 m); histogram {res['known_hist']}")
    print(f"  person encounters (closer than 6 m): {len(pe)}; first tracked at median "
          f"{res['first_tracked_at_m']['median']} m (closest {res['first_tracked_at_m']['min']} m); "
          f"first in sight closer than 3 m (came out from behind something): {len(surprise)}; "
          f"tracked {res['track_after_sight_s']['median']} s after first in sight (p90 {res['track_after_sight_s']['p90']} s)")
    print(f"  tracks of nobody (not a person, not a vehicle): {res['false_person_tracks_per_scan']} per scan; "
          f"in sight but never tracked: {res['never_tracked'] or 'none'}; encounters hidden all along: "
          f"{res['hidden_whole_encounter']}")
    if out_json:
        json.dump({**res, 'encounters': [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in e.items()}
                                         for e in encounters]}, open(out_json, 'w'), indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
