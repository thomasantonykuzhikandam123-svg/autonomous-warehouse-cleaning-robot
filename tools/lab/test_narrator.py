"""narrator.py fed from a fake-ROS coverage_nav2 v10.4 run (test_coverage_traffic's world):
a guard hold, a stretch put off and revisited, the end of the job.
v10.5: a second run with lanes driven along: someone standing in the lane (gone round), the
patch pass, the lanes swept at the end; then traffic going past the idle robot: says nothing.

    python3 test_narrator.py <coverage_nav2.py> <narrator.py>
"""
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_coverage_traffic as T  # noqa: E402

spec = importlib.util.spec_from_file_location('narrator_t', sys.argv[2])
N = importlib.util.module_from_spec(spec)
spec.loader.exec_module(N)
LINES = []
nar = N.Narrator()
orig = T.Pub.publish


def publish(self, m):
    orig(self, m)
    if self.topic == '/mission_state':
        for ln in nar.feed(json.loads(m.data), T.W.t):
            LINES.append((round(T.W.t, 1), ln))


T.Pub.publish = publish
T.install_fakes()
os.environ['HOME'] = __import__('tempfile').mkdtemp()
m = T.load(sys.argv[1])
lane = [(0.0, 0.0, 'wp1'), (0.0, 10.0, 'wp2'), (3.0, 10.0, 'wp3'), (3.0, 0.0, 'wp4'), (0.0, 0.0, 'HOME')]
zones = ['aisle_1', 'aisle_1', 'aisle_2', 'aisle_2', 'home']
n = T.fresh(m, lane, zones)
T.GUARD['on'] = True
T.run_until(lambda: T.W.y > 2.0, 60)
T.GUARD.update(state='wait', vehicle_m=1.4, say='giving way to a vehicle: going on would pass 0.3 m from it')
T.W.hold = True
T.run_until(lambda: False, 12)
T.GUARD.update(state='go', vehicle_m=None, say='clear')
T.W.hold = False
T.run_until(lambda: T.W.y > 5.0, 60)
T.GUARD.update(state='go', person_m=1.0, say='slowed to 0.3 m/s: passing a person standing nearby')
T.W.hold = True
T.run_until(lambda: n.deferred, 90)
T.GUARD.update(person_m=None, say='clear')
T.W.hold = False
T.run_until(lambda: n._done, 400)
T.run_until(lambda: False, 2)
for t, ln in LINES:
    print(f'[{t:7.1f}] robot: {ln}')
text = ' | '.join(l for _, l in LINES)
FAILS = []
for want in ('New job: aisle 1, then aisle 2', 'Giving way to a vehicle', 'Off again after 12 s',
             "I'll leave that stretch and come back", 'Going back for the floor', 'Now cleaning aisle 2',
             'Heading back to the dock', 'Job done. Cleaned: aisle 1, aisle 2', 'stopped for people or vehicles 1 time,'):
    ok = want in text
    print(f'{"ok  " if ok else "FAIL"} says: {want}')
    if not ok:
        FAILS.append(want)

# ---- v10.5: lanes driven along, someone standing in one, the patch pass; then idle traffic
LINES.clear()
nar = N.Narrator()
L2 = [(0.0, 0.0, 'wp1'), (0.0, 8.0, 'wp2'), (0.5, 8.0, 'wp3'), (0.5, 0.0, 'wp4'), (0.0, 0.0, 'HOME')]
n = T.fresh(m, L2, ['aisle_1'] * 4 + ['home'], lanes=True, sweeps={L2[1]: (0.0, 0.0), L2[3]: (0.5, 8.0)})
T.GUARD['on'] = True
T.W.blocker = (0.0, 4.0)
T.run_until(lambda: n.bypasses, 60)
T.run_until(lambda: T.W.goal is not None and abs(T.W.goal.x - 0.5) < 1e-6, 60)
T.W.blocker = None
T.run_until(lambda: n._done, 300)
t_done = T.W.t
for k in range(60):                       # traffic going past the robot at the dock
    T.GUARD.update(v_limit=0.4, limit='a vehicle is near' if k % 2 else 'easing off near people',
                   state='wait' if k % 10 == 3 else 'go', say='giving way to a vehicle' if k % 10 == 3 else 'clear')
    T.run_until(lambda: False, 1.0)
for t, ln in LINES:
    print(f'[{t:7.1f}] robot: {ln}')
text = ' | '.join(l for _, l in LINES)
for want in ("Someone is standing in my lane, so I'm going round them", 'Going back over 1 stretch of lane',
             'Lanes swept: aisle 1'):
    ok = want in text
    print(f'{"ok  " if ok else "FAIL"} v10.5 says: {want}')
    if not ok:
        FAILS.append(want)
after = [l for t, l in LINES if t > t_done + 0.5]
# ---- v10.6: the guard falling silent, a vehicle at a lane end, something in the lane; freezes
LINES.clear()
nar = N.Narrator()
n = T.fresh(m, L2, ['aisle_1'] * 4 + ['home'], lanes=True, sweeps={L2[1]: (0.0, 0.0), L2[3]: (0.5, 8.0)})
T.GUARD['on'] = True
T.run_until(lambda: T.W.y > 2.0, 60)
T.GUARD['on'] = False
T.run_until(lambda: n._safety_hold, 5)
T.run_until(lambda: False, 3)
T.GUARD['on'] = True
T.run_until(lambda: T.W.y > 4.5, 60)
T.GUARD.update(state='wait', say='waiting: a vehicle is standing in my way and its forks may be out')
T.W.hold = True
T.run_until(lambda: n.vehicle_skips, 20)
T.GUARD.update(state='go', say='clear')
T.W.hold = False
T.run_until(lambda: T.W.goal is not None and abs(T.W.goal.x - 0.5) < 1e-6 and T.W.goal.path is not None, 60)
T.W.blocker, T.W.blocker_r, T.W.block_r = (0.5, T.W.y - 1.5), 0.12, 0.55
T.run_until(lambda: n._ahead_hold, 20)
T.run_until(lambda: False, 2)
T.W.blocker = None
T.run_until(lambda: n._done, 300)
text6 = ' | '.join(l for _, l in LINES)
for want in ("My safety layer has stopped reporting, so I'm holding still", 'My safety layer is back',
             'A vehicle is working where I need to sweep', "Something is standing in my lane, so I've stopped short"):
    ok = want in text6
    print(f'{"ok  " if ok else "FAIL"} v10.6 says: {want}')
    if not ok:
        FAILS.append(want)
LINES.clear()
n = T.fresh(m, L2, ['aisle_1'] * 4 + ['home'], lanes=True, sweeps={L2[1]: (0.0, 0.0), L2[3]: (0.5, 8.0)})
T.GUARD['on'] = True
T.W.blocker = (0.0, 4.0)
T.run_until(lambda: T.W.log and any('PATCH round 2' in x[2] for x in T.W.log) or n._done, 500)
T.W.blocker = None
T.run_until(lambda: n._done, 300)
text7 = ' | '.join(l for _, l in LINES)
want = "going over it once more before going home"
ok = want in text7 and text7.count('Going back over') == 1
print(f'{"ok  " if ok else "FAIL"} v10.6 says: the second patch round, once ({text7.count(want)})')
if not ok:
    FAILS.append('second round')
LINES.clear()
n = T.fresh(m, L2, ['aisle_1'] * 4 + ['home'], lanes=True, sweeps={L2[1]: (0.0, 0.0), L2[3]: (0.5, 8.0)})
T.GUARD['on'] = True
T.run_until(lambda: T.W.y > 3.0, 60)
T.W.nav_state = 'inactive'
T.end_goal(T.W.goal, 6)
T.run_until(lambda: False, 30)
T.W.nav_state = 'up'
T.run_until(lambda: n._done, 300)
text8 = ' | '.join(l for _, l in LINES)
for want in ("My navigation (Nav2) has stopped taking goals, so I'm holding still", 'My navigation is back'):
    ok = want in text8 and text8.count(want) == 1
    print(f'{"ok  " if ok else "FAIL"} v10.6 says (once): {want}')
    if not ok:
        FAILS.append(want)
fw = N.FreezeWatch(20.0)
r = [fw.seen(100.0), fw.check(110.0), fw.check(125.0), fw.check(130.0), fw.seen(1445.0), fw.check(1450.0)]
ok = (r[0] is None and r[1] is None and r[2] and 'no word from the robot for 20 s' in r[2] and r[3] is None
      and r[4] and '22.4 min' in r[4] and r[5] is None)
print(f'{"ok  " if ok else "FAIL"} v10.6 freeze watch: says it once after 20 s of real time, and when the robot is back ({r[4]})')
if not ok:
    FAILS.append('freeze watch')
ok = not after
print(f'{"ok  " if ok else "FAIL"} v10.5: nothing said about traffic while idle after the job ({len(after)} lines)')
if not ok:
    FAILS.append('idle silence')
print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED')
sys.exit(1 if FAILS else 0)
