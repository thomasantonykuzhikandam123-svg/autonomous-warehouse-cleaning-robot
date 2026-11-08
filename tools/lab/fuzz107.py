"""Randomized stress test of coverage_nav2 v10.7's manoeuvres (ALIGN, UNSTICK) and start-up, in a
fake aisle 2 between racks 2 and 3 (static map and /scan_nav), with test_v107.py's RPP-like
controller and the robot obeying the node's /cmd_vel while no goal runs.

    python3 fuzz107.py <coverage_nav2.py> <tests dir> [runs]

Each run: the robot starts somewhere in the front corridor facing anywhere; 'clean aisle_2' is sent
(after a while with the guard flickering, before the command); then at random: guard holds 2-25 s
(nothing moves meanwhile: the Collision Monitor), people 0.5-3 m away, someone standing in a lane
(legs or personal space), the guard silent 1-12 s, goals aborted, the operator pausing 2-20 s and
resuming. Promises:
  P1 the robot's 0.30 m circle never overlaps a rack (RPP's check, the Brake zone, turns in place)
  P2 the node never drives /cmd_vel while a Nav2 goal is active
  P3 ... nor while paused, in a safety hold, before the first command, or while the guard holds
  P4 every run ends (COVERAGE COMPLETE), no exception
  P5 at most UNSTICK_MAX unsticks per waypoint
and it reports the lanes swept, the time, and how often ALIGN / UNSTICK were used.
"""
import json
import math
import os
import random
import sys
import tempfile
import traceback

sys.path.insert(0, sys.argv[2])
sys.argv = [sys.argv[0], sys.argv[1]] + sys.argv[2:]
import test_v107 as V  # noqa: E402

T, W, GUARD, P = V.T, V.W, V.GUARD, V.P
T.install_fakes()
os.environ['HOME'] = tempfile.mkdtemp()
m = T.load(sys.argv[1])
RACKS = [(1.0, 2.0, 1.5, 13.5), (5.0, 6.0, 1.5, 13.5)]


def dist_racks(x, y):
    best = 9.0
    for x0, x1, y0, y1 in RACKS:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        best = min(best, math.hypot(dx, dy))
    return best


V.dist_rack = dist_racks                     # the controller model and the Brake zone see both racks


def walls():
    out = []
    for x0, x1, y0, y1 in RACKS:
        out += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return out


def fake_map():
    res, ox, oy = 0.05, -2.0, -2.0
    w, h = int(10.0 / res), int(18.0 / res)
    data = [0] * (w * h)
    for gy in range(h):
        cy = oy + (gy + 0.5) * res
        for gx in range(w):
            cx = ox + (gx + 0.5) * res
            for x0, x1, y0, y1 in RACKS:
                if x0 <= cx <= x1 and y0 <= cy <= y1:
                    data[gy * w + gx] = 100
    return P(info=P(resolution=res, width=w, height=h, origin=P(position=P(x=ox, y=oy))), data=data)


MAP = fake_map()


def seg_hits(p, q, pad=0.42):
    """Does the straight drive p -> q pass within pad of a rack?"""
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    n = max(2, int(L / 0.05))
    return any(dist_racks(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n) < pad
               for k in range(n + 1))


_nav_route_people = T.nav_route


def nav_route(p, goal):
    """The fake planner: straight (round someone standing in the way, as before), else round the
    nearer rack end (y 0.75 or 14.25)."""
    if not seg_hits(p, goal):
        r = _nav_route_people(p, goal)
        if all(not seg_hits(a, b) for a, b in zip([p] + r[:-1], r)):
            return r
        return [goal]
    best = None
    for yc in (0.75, 14.25):
        r = [(p[0], yc), (goal[0], yc), goal]
        L = (abs(p[1] - yc) + abs(goal[0] - p[0]) + abs(goal[1] - yc))
        if best is None or L < best[0]:
            best = (L, r)
    return best[1]


T.nav_route = nav_route


def run(seed, runs_log):
    rnd = random.Random(seed)
    calm = seed < 0
    W.__init__()
    V.TW.update(t=-1e9, vx=0.0, wz=0.0)
    GUARD.update(on=True, state='go', person_m=None, vehicle_m=None, say='clear', v_limit=None, limit=None)
    W.walls = walls()
    W.twists = []
    W.min_gap = 9.0
    if calm:
        W.x, W.y, W.yaw = 0.0, 0.0, 0.0
    else:
        while True:
            W.x, W.y = rnd.uniform(-0.5, 6.5), rnd.uniform(-1.0, 0.6)
            if dist_racks(W.x, W.y) > 0.6:
                break
        W.yaw = rnd.uniform(-math.pi, math.pi)
    n = m.CoverageNav2()                             # auto_start false, as stack.sh starts it
    n.LANES_FOLLOW = True                            # its default (the fake parameters say False)
    n._map = MAP
    n.cleaned, n.zone_pct = [], {}
    bad = {'P2': 0, 'P3': 0}
    # before the command: the guard flickers (shift_v106a), nothing may move
    t_cmd = W.t + (0.0 if calm else rnd.uniform(2.0, 15.0))
    while W.t < t_cmd:
        T.step()
        if not calm and rnd.random() < 0.05:
            GUARD['on'] = not GUARD['on']
    GUARD['on'] = True
    pre_sent, pre_tw = len(W.sent), len(W.twists)
    n.on_command(P(data=json.dumps({'action': 'clean', 'zones': ['aisle_2']})))
    t_end = W.t + 2500
    hold_until = silent_until = blk_until = pause_until = pm_until = 0.0
    paused = False
    while W.t < t_end and not n._done:
        k_tw = len(W.twists)
        goal_active = W.goal is not None and not W.goal.done
        T.step()
        t = W.t
        # P2 / P3: what the node drove on /cmd_vel during this step
        for (_, vx, wz, during_goal) in W.twists[k_tw:]:
            if vx == 0.0 and wz == 0.0:
                continue
            if during_goal:
                bad['P2'] += 1
            if n._paused or n._safety_hold or n._awaiting or (GUARD['on'] and GUARD['state'] == 'wait'):
                bad['P3'] += 1
        if calm:
            continue
        if GUARD['state'] == 'go' and rnd.random() < 0.002:
            GUARD.update(state='wait', say='waiting for a person to pass')
            hold_until = t + rnd.uniform(2, 25)
        if GUARD['state'] == 'wait' and t > hold_until:
            GUARD.update(state='go', say='clear')
        if GUARD['person_m'] is None and rnd.random() < 0.003:
            GUARD['person_m'] = rnd.choice([0.5, 0.8, 1.2, 1.8, 2.5, 3.0])
            pm_until = t + rnd.uniform(2, 30)
        if GUARD['person_m'] is not None and t > pm_until:
            GUARD['person_m'] = None
        if W.blocker is None and rnd.random() < 0.002 and W.goal is not None and W.goal.path is not None:
            gx, gy = W.goal.x, W.goal.y
            f = rnd.uniform(0.2, 0.9)
            W.blocker = (W.x + (gx - W.x) * f + rnd.uniform(-0.2, 0.2), W.y + (gy - W.y) * f)
            legs = rnd.random() < 0.4
            W.blocker_r, W.block_r = (0.12, 0.55) if legs else (0.9, 1.6)
            blk_until = t + rnd.choice([3, 8, 12, 20, 40])
        if W.blocker is not None and t > blk_until:
            W.blocker = None
            W.blocker_r, W.block_r = 0.9, 1.6
        if GUARD['on'] and rnd.random() < 0.0005:
            GUARD['on'] = False
            silent_until = t + rnd.uniform(1.0, 12.0)
        if not GUARD['on'] and t > silent_until:
            GUARD['on'] = True
        if rnd.random() < 0.001:
            W.abort_next, W.abort_after = 1, rnd.uniform(0.3, 4.0)
        if not paused and rnd.random() < 0.0008:
            n.on_command(P(data=json.dumps({'action': 'stop'})))
            paused, pause_until = True, t + rnd.uniform(2, 20)
        if paused and t > pause_until:
            n.on_command(P(data=json.dumps({'action': 'resume'})))
            paused = False
    n._update_swept(force=True)
    un = {}
    for _, l, msg in W.log:
        if msg.startswith('UNSTICK wp'):
            k = msg.split(':')[0]
            un[k] = un.get(k, 0) + 1
    kinds = {}
    for _, l, msg in W.log:
        for k in ('ALIGN: wp', 'UNSTICK wp', 'BYPASS', 'DEFER', 'SKIP', 'STALL', 'RESCUE', 'CLEARANCE: wp',
                  'SAFETY: no word', 'TIMEOUT', 'not backing off', 'not turning', 'gave up after', 'took over'):
            if k in msg:
                kinds[k] = kinds.get(k, 0) + 1
    r = dict(seed=seed, done=n._done, t=round(W.t - t_cmd, 0), swept=round(n.swept_pct.get('aisle_2', 0.0), 1),
             min_gap=round(W.min_gap, 3), pre=(pre_sent, pre_tw), P2=bad['P2'], P3=bad['P3'],
             P5=max(un.values()) if un else 0, kinds=kinds, skipped=len(n.skipped))
    runs_log.append(r)
    return r


def main():
    runs = int(sys.argv[3]) if len(sys.argv) > 3 else 30
    res = []
    for seed in range(-1, runs):
        try:
            r = run(seed, res)
        except Exception:
            traceback.print_exc()
            res.append(dict(seed=seed, done='EXC'))
            continue
        print(r, flush=True)
    ok = [r for r in res if r.get('done') is True]
    print(f'{len(res)} runs, {len(res) - len(ok)} not finished or exception')
    if not ok:
        return 1
    print('P1 closest to a rack (circle edge), m: ' + str(sorted((r['min_gap'], r['seed']) for r in ok)[:3]))
    print('before the command: goals sent / twists in any run: '
          f"{max(r['pre'][0] for r in ok)} / {max(r['pre'][1] for r in ok)}")
    print(f"P2 twists during a goal: {sum(r['P2'] for r in ok)}; P3 twists while paused/held/silent: "
          f"{sum(r['P3'] for r in ok)}; P5 most unsticks on one waypoint: {max(r['P5'] for r in ok)}")
    sw = sorted(r['swept'] for r in ok)
    tt = sorted(r['t'] for r in ok[1:]) or [ok[0]['t']]
    print(f'aisle 2 swept: median {sw[len(sw) // 2]}, lowest {sw[0]}, runs below 90: '
          f'{sum(1 for x in sw if x < 90)}; time: calm {ok[0]["t"]:.0f} s, median {tt[len(tt) // 2]:.0f}, '
          f'longest {tt[-1]:.0f}')
    tot = {}
    for r in ok:
        for k, v in r['kinds'].items():
            tot[k] = tot.get(k, 0) + v
    print('events in all runs: ' + ', '.join(f'{k} {v}' for k, v in sorted(tot.items())))
    fails = (len(res) - len(ok) + sum(r['P2'] + r['P3'] for r in ok)
             + sum(1 for r in ok if r['min_gap'] < -0.005 or r['P5'] > 2 or r['pre'] != (0, 0)))
    print('ALL PROMISES KEPT' if not fails else f'{fails} PROBLEM(S)')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
