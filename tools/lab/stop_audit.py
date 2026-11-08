#!/usr/bin/env python3
"""stop_audit.py <tools dir> <runs dir> <label> [label ...] [--own] [--list] - every time the robot stopped
for a person, the future it avoided (rewind.py, the same code as the page's Rewind).

    python3 ~/ros2_ws/src/tools/lab/stop_audit.py ~/ros2_ws/src/tools ~/runs shift_v108a --list
    python3 ~/ros2_ws/src/tools/lab/stop_audit.py ~/ros2_ws/src/tools ~/runs shift_v103a shift_v104a shift_v105a shift_v106a shift_v108a --own

A stop = the people guard switching to WAIT for a person (the recorded guard/<label>.csv: what the robot
really did). For each: "if I had not stopped", the robot along the plan it had, by the guard's own model of
going on, against where every person really went (traffic/<label>.csv and the world's worker: scripted
actors that never react to the robot, so this is exactly where they would have gone). Body to body, 0.00 =
touching: would have hit / within 20 cm / tight (to 0.50 m) / clear (over-cautious, in hindsight).
--own: also the verdict from the robot's own tracks only (the guard replayed over the recording, no truth:
what a real robot could do), and how often it agrees. How they would have met: crossing its path (only the
guard handles those: it hides crossers from Nav2), or walking along / standing in it (Nav2 sees those and
might have braked as well, so for them 'would have hit' is an upper bound).
"""
import math
import os
import sys

import numpy as np


class Observed:
    """What the robot itself saw: the guard replayed over the recording (open loop, as the page's replay)."""

    def __init__(self, rp, W, RW):
        core = rp.new_core()
        amin, ainc = float(rp.meta['angle_min']), float(rp.meta['angle_increment'])
        rmin, rmax = float(rp.meta['range_min']), float(rp.meta['range_max'])
        prev, v_est = None, 0.0
        self.tracks = RW.Tracks()
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
            self.tracks.add(t, RW.people_from_shapes(W.guard_shapes(rp.pg, core, lk, state, robot, t)))


def how_met(st, truth):
    """crossing / along / standing: the person's real motion against the robot's heading at the moment."""
    if st.get('who') is None or st.get('at') is None:
        return None
    t = st['t'] + st['at']
    p0 = [a for a in truth.at(t - 0.2) if a[0] == st['who']]
    p1 = [a for a in truth.at(t + 0.2) if a[0] == st['who']]
    if not p0 or not p1:
        return None
    wx, wy = p1[0][1] - p0[0][1], p1[0][2] - p0[0][2]
    path = st.get('path') or []
    i = min(int(round(st['at'] / 0.1)), len(path) - 1)
    ryaw = path[i][3] if path else st['yaw']
    wn = math.hypot(wx, wy)
    if wn < 0.06:
        return 'standing'
    return 'along' if abs(wx * math.cos(ryaw) + wy * math.sin(ryaw)) / wn > 0.7 else 'crossing'


def audit(tools, runs, label, W, PG, RW, own=False):
    hub = W.Hub('replay')
    rp = W.Replay(hub, runs, label, guard=os.path.join(tools, 'people_guard.py'), loop=False)
    rw = RW.Rewind(PG)
    rw.from_record(os.path.join(os.path.expanduser(runs), 'guard', f'{label}.csv'), rp.plan_at, rp.truth_ts, rp.truth,
                   W.world_worker, rp.T, rp.RB)
    truth = RW.Tracks()
    for t, fr in zip(rp.truth_ts, rp.truth):
        ppl = [(a['name'], a['x'], a['y']) for a in fr if a.get('kind') == 'person']
        w = W.world_worker(float(t))
        truth.add(float(t), ppl + [(w['name'], w['x'], w['y'])])
    seen = Observed(rp, W, RW) if own else None
    out = []
    for i in [s['id'] for s in rw.visible()]:
        st = rw.detail(i)
        st['label'] = label
        st['how'] = how_met(st, truth)
        if seen is not None:
            f = RW.avoided(PG, st['x'], st['y'], st['yaw'], st['v'], rp.plan_at(st['t']), st['t'], seen.tracks, st['soft'])
            st['own_gap'], st['own_cls'] = f['gap'], RW.classify(f['gap'])
        out.append(st)
    return out


def summary(res, title, RW):
    n = len(res)
    print(f'\n{title}: {n} stops for a person')
    if not n:
        return
    for key, _, name in RW.CLASSES:
        k = [r for r in res if r['cls'] == key]
        print(f'  {name:15s} {len(k):4d} ({100 * len(k) / n:3.0f} %), waited {sum(r["waited"] for r in k):6.0f} s in all')
    hit = [r for r in res if r['cls'] == 'hit']
    if hit:
        hw, who = {}, {}
        for r in hit:
            hw[r['how']] = hw.get(r['how'], 0) + 1
            who[r['who']] = who.get(r['who'], 0) + 1
        print('  the hits it avoided, by how they would have met: ' + ', '.join(
            f'{k} {hw.get(k, 0)}' for k in ('crossing', 'along', 'standing'))
            + '; by person: ' + ', '.join(f'{k} {v}' for k, v in sorted(who.items(), key=lambda kv: -kv[1])))
        sp = np.array([r['v_at'] for r in hit])
        print(f'  at the moment of the hit it would have been doing {np.median(sp):.2f} m/s (median; '
              f'{np.mean(sp >= 0.5) * 100:.0f} % at 0.5 m/s or more), {np.median([r["at"] for r in hit]):.1f} s after it stopped')
    pred = np.array([r.get('pred', 9.0) for r in res])
    ok = pred < 8.9
    if ok.any():
        # the true closest centre distance along the same avoided path
        true_c = []
        for r in res:
            best = 9.0
            for (dt, x, y, yaw, v) in r['path']:
                for name, pts in r['people'].items():
                    for (pt, px, py) in pts:
                        if abs(pt - dt) < 0.051:
                            best = min(best, math.hypot(px - x, py - y))
            true_c.append(best)
        err = pred[ok] - np.array(true_c)[ok]
        print(f"  the guard's prediction (closest centre distance going on) against what really would have happened: "
              f"median {np.median(err):+.2f} m, within 0.25 m in {100 * np.mean(np.abs(err) <= 0.25):.0f} %, predicted "
              f"closer than it was in {100 * np.mean(err < 0):.0f} %")
    own = [r for r in res if r.get('own_cls')]
    if own:
        same = sum(1 for r in own if r['own_cls'] == r['cls'])
        hits = [r for r in own if r['cls'] == 'hit']
        clear = [r for r in own if r['cls'] == 'clear']
        print(f"  from its own tracks only (no simulator truth): the same verdict in {same} of {len(own)} "
              f"({100 * same / len(own):.0f} %); of the {len(hits)} hits it calls {sum(1 for r in hits if r['own_cls'] == 'hit')} "
              f"hits and {sum(1 for r in hits if r['own_cls'] in ('tight', 'clear'))} tight or clear (out of its sight); "
              f"of the {len(clear)} clear ones it calls {sum(1 for r in clear if r['own_cls'] == 'clear')} clear")
    real = np.array([r['real_gap'] for r in res])
    touch = [r for r in res if r['real_gap'] <= 0.0]
    tow = [r for r in touch if (r.get('real_v') or 0.0) >= 0.05]
    print(f'  what really happened in the 4 s after it stopped: closest {real.min():.2f} m; touching after {len(touch)} '
          f'of the stops: in {len(touch) - len(tow)} they walked into it (it was not moving towards them), in {len(tow)} '
          f'it was moving towards them' + (f" (at most {max(r['real_v'] for r in tow):.2f} m/s)" if tow else ''))


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    tools, runs = os.path.expanduser(argv[0]), os.path.expanduser(argv[1])
    labels = [a for a in argv[2:] if not a.startswith('--')]
    sys.path.insert(0, tools)
    import robot_web as W            # noqa: E402
    import people_guard as PG        # noqa: E402
    import rewind as RW              # noqa: E402
    print(f'stop_audit: rewind {RW.VERSION}; going on = the guard\'s own model along the plan for {RW.HORIZON:.0f} s')
    allres = []
    for label in labels:
        res = audit(tools, runs, label, W, PG, RW, own='--own' in argv)
        allres += res
        if '--list' in argv:
            print(f'\n{label}:   t    why        waited  v     guard said  would have  who          how       real gap')
            for r in res:
                print(f"  {r['t']:7.1f}  {r['why']:9s} {r['waited']:5.1f} s {r['v']:4.2f}  {min(r.get('pred', 9), 9):5.2f} m    "
                      f"{r['gap']:5.2f} m {RW.label(r['cls']):15s} {str(r['who']):12s} {str(r['how']):9s} {r['real_gap']:.2f} m"
                      + (f"   own: {RW.label(r['own_cls'])}" if r.get('own_cls') else ''))
        summary(res, label, RW)
    if len(labels) > 1:
        summary(allres, f'all {len(labels)} shifts', RW)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
