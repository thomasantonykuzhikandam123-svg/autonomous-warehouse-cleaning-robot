#!/usr/bin/env python3
"""wet_audit.py <tools dir> <runs dir> <label> [label ...] [--carry] [--sweep] - the wet-floor guardian (v10.13) on
recorded shifts, against today's scrubbers (the water always on).

    python3 ~/ros2_ws/src/tools/lab/wet_audit.py ~/ros2_ws/src/tools ~/runs shift_v108a
    python3 ~/ros2_ws/src/tools/lab/wet_audit.py ~/ros2_ws/src/tools ~/runs shift_v103a shift_v104a shift_v105a shift_v106a shift_v108a --carry

Each recorded scan goes through the real people guard (open loop, as the page's replay does); the
guardian decides the water from the guard's tracks and predictions and the lidar's view only. The robot
drives exactly as recorded either way (the water changes nothing about where it goes), so both are
measured on the same drive. Then, from the simulator's truth (the traffic log plus the world's worker):
person-seconds on wet floor, encounters (stepping onto it; back on it within 5 s is the same one), and
whether they were warned in the 6 s before.
--carry: what it learned about where people walk is kept from one shift to the next (in the order given).
--sweep: also a few other settings, side by side (same replay).
"""
import math
import os
import sys
import time


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    tools, runs = os.path.expanduser(argv[0]), os.path.expanduser(argv[1])
    labels = [a for a in argv[2:] if not a.startswith('--')]
    carry, sweep = '--carry' in argv, '--sweep' in argv
    sys.path.insert(0, tools)
    import robot_web as W            # noqa: E402  (no ROS needed for the replay parts)
    import wet_floor as WF           # noqa: E402
    configs = [('always on', dict(policy='always')), ('guardian', dict(policy='guardian'))]
    if sweep:
        configs += [('only learned habits', dict(policy='guardian', now_r=-9.0, pred_r=-9.0)),
                    ('habits 3 s', dict(policy='guardian', h_off=3.0)),
                    ('near 0.5 m', dict(policy='guardian', now_r=0.5)),
                    ('habits 3 s, near 0.5', dict(policy='guardian', h_off=3.0, now_r=0.5))]
    print(f'wet_audit: wet_floor {WF.VERSION}; the floor stays wet {WF.T_DRY:.0f} s after cleaning (assumed); '
          f'water off where people are expected over {WF.H_OFF:.0f} s of that, or someone is within {WF.NOW_R:.1f} m '
          f'of the brush or predicted within {WF.PRED_R:.1f} m' + ('; habits carried between shifts' if carry else ''),
          flush=True)
    habits = {}
    sums = {name: {'ps': 0.0, 'vs': 0.0, 'enc': 0, 'warned': 0, 'dry': 0.0, 'cleaned': 0.0, 'off': 0, 'clean': 0,
                   'warnings': 0, 'by_name': {}} for name, _ in configs}
    for label in labels:
        t_start = time.time()
        hub = W.Hub('replay')
        rp = W.Replay(hub, runs, label, guard=os.path.join(tools, 'people_guard.py'), loop=False)
        mi = rp.mi
        floors = []
        for name, kw in configs:
            f = WF.WetFloor(mi['width'], mi['height'], mi['resolution'], mi['origin_x'], mi['origin_y'], **kw)
            if carry and name in habits:
                f.P, f.O = habits[name][0].copy(), habits[name][1].copy()
            floors.append((name, f))
        pg = rp.pg
        core = rp.new_core()
        amin, ainc = float(rp.meta['angle_min']), float(rp.meta['angle_increment'])
        rmin, rmax = float(rp.meta['range_min']), float(rp.meta['range_max'])
        prev, v_est = None, 0.0
        n_clean, n_off = 0, {name: 0 for name, _ in configs}
        for k in range(len(rp.T)):
            t = float(rp.T[k])
            robot, laser = tuple(float(v) for v in rp.RB[k]), tuple(float(v) for v in rp.LS[k])
            if rp.OA[k] < 0.5:
                v_est = float(rp.OV[k])
            elif prev is not None and t > prev[0]:
                v_est = 0.5 * v_est + 0.5 * math.hypot(robot[0] - prev[1], robot[1] - prev[2]) / (t - prev[0])
            prev = (t, robot[0], robot[1])
            plan = rp.plan_at(t)
            core.scan(rp.RG[k], amin, ainc, rmin, rmax, laser, t, robot, plan)
            state, lk, _ = core.decide(robot[:2], robot[2], v_est, 0.8, t, plan)
            people = WF.people_from_shapes(W.guard_shapes(pg, core, lk, state, robot, t))
            view = W.view_polygon(rp.RG[k], amin, ainc, rmin, rmax, *laser)
            mission = rp.mission_at(t, None, 0.0, 0, 0.0)
            if mission.get('zone') in W.ZONES:          # older recordings did not log the command itself
                mission['awaiting_command'] = False
            cleaning = WF.cleaning_now(mission, W.ZONES, robot[0], robot[1], v_est)
            n_clean += cleaning
            j = rp.at(rp.truth_ts, t)
            agents = list(rp.truth[j]) + [W.world_worker(t)] if j >= 0 and t - rp.truth_ts[j] < 1.0 else None
            for name, f in floors:
                f.step(t, robot[0], robot[1], cleaning, people, view)
                n_off[name] += cleaning and f.pump == 'off'
                f.take_events()
                if agents is not None:
                    f.measure(t, agents)
        print(f'\n{label}: {len(rp.T)} scans, cleaning {n_clean / 10:.0f} s ({time.time() - t_start:.0f} s to replay)')
        print(f"  {'':22s} {'people on wet':>13s} {'encounters':>10s} {'warned':>7s} {'vehicles on wet':>15s} "
              f"{'water off':>9s} {'cleaned dry':>15s}")
        for name, f in floors:
            st, a = f.truth_stats, f.area(t)
            ps = st['seconds_by_kind'].get('person', 0.0)
            vs = st['seconds_by_kind'].get('vehicle', 0.0)
            enc = st['by_kind'].get('person', 0)
            S = sums[name]
            S['ps'] += ps
            S['vs'] += vs
            S['enc'] += enc
            S['warned'] += st['warned']
            S['dry'] += a['dry_m2']
            S['cleaned'] += a['cleaned_m2']
            S['off'] += n_off[name]
            S['clean'] += n_clean
            S['warnings'] += f.n_warned
            for n, c in st['seconds_by_name'].items():
                S['by_name'][n] = S['by_name'].get(n, 0) + c
            print(f"  {name:22s} {ps:11.0f} s {enc:10d} {st['warned']:7d} {vs:13.0f} s "
                  f"{100 * n_off[name] / max(n_clean, 1):8.0f} % {a['dry_m2']:6.1f} of {a['cleaned_m2']:5.1f} m2")
            habits[name] = (f.P, f.O)
    if len(labels) > 1:
        print(f'\nall {len(labels)} shifts' + (' (habits carried)' if carry else ''))
        b = sums['always on']
        for name, _ in configs:
            S = sums[name]
            print(f"  {name:22s} people on wet floor {S['ps']:5.0f} s ({100 * (1 - S['ps'] / max(b['ps'], 1e-9)):.0f} % "
                  f"less than always on), encounters {S['enc']}, warned in time {S['warned']} of {S['enc']}, vehicles "
                  f"{S['vs']:.0f} s, water off {100 * S['off'] / max(S['clean'], 1):.0f} % of the cleaning, cleaned dry "
                  f"{S['dry']:.1f} of {S['cleaned']:.1f} m2 ({100 * S['dry'] / max(S['cleaned'], 1e-9):.0f} %)")
        g = sums['guardian']
        print('  seconds on wet floor by agent (always on -> guardian): ' + ', '.join(
            f"{n} {b['by_name'].get(n, 0):.0f}->{g['by_name'].get(n, 0):.0f}"
            for n in sorted(b['by_name'], key=lambda n: -b['by_name'][n])))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
