"""coverage_nav2 v10.10 (14 Oct, after the demo rehearsal): the front, the back and the dock through
the middle of an aisle, never round the racks along a wall.

    python3 test_v1010.py <path to coverage_nav2.py>

Uses test_coverage_traffic.py's fake ROS. The rehearsal: 'clean the back area' at the front area's
east end (10.37, 1.29) sent the robot up the corridor along the east wall (x 9.06-10.9), where it
wedged at (10.41, 4.17) for good.
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_coverage_traffic as T  # noqa: E402

W = T.W
FAILS = []


def check(name, ok, detail=''):
    print(('  ok   ' if ok else '  FAIL ') + name + ('' if ok else f'   {detail}'), flush=True)
    if not ok:
        FAILS.append(name)


def east_corridor(pts):
    """map points in the corridor beside the east wall, between the racks' ends"""
    return [(x, y) for x, y in pts if x > 9.1 and 1.6 < y < 13.4]


def main(path):
    T.install_fakes()
    os.environ['HOME'] = tempfile.mkdtemp()
    m = T.load(path)
    W.__init__()
    n = m.CoverageNav2()
    aisle3 = [((l + r) / 2.0 - n.spawn_x) for l, r in n.aisles][2]
    W.x, W.y = 10.37, 1.29                                     # the rehearsal: the front's east end
    g, z = n.build_mission(['back'])
    pts = [(x, y) for (x, y, _) in g]
    check("the back from the front's east end: up the middle of aisle 3 first",
          len(pts) > 3 and abs(pts[1][0] - aisle3) < 0.05 and abs(pts[1][1]) < 0.05
          and abs(pts[2][0] - aisle3) < 0.05 and abs(pts[2][1] - 14.8) < 0.05, pts[:4])
    check('... and nothing in the corridor along the east wall', not east_corridor(pts), east_corridor(pts))
    W.x, W.y = -3.2, -0.9                                      # the front's west end
    g, z = n.build_mission(['back'])
    pts = [(x, y) for (x, y, _) in g]
    check('the back from the west end: up aisle 1', abs(pts[1][0] - (-0.49)) < 0.1 and abs(pts[2][1] - 14.8) < 0.05,
          pts[:3])
    W.x, W.y = 10.4, 15.0                                      # the back's east end
    g, z = n.build_mission(['front'])
    pts = [(x, y) for (x, y, _) in g]
    check("the front from the back's east end: along the back strip to aisle 3, down it",
          abs(pts[1][0] - aisle3) < 0.05 and abs(pts[1][1] - 14.8) < 0.05 and abs(pts[2][1]) < 0.05
          and not east_corridor(pts), pts[:4])
    W.x, W.y = 0.0, 0.0                                        # the dock
    g, z = n.build_mission(['front', 'aisle_1', 'back'])
    pts = [(x, y) for (x, y, _) in g]
    zs = list(z)
    first_back = zs.index('back')
    check("front, aisle 1, back from the dock: the back straight on from aisle 1's top (no extra transit)",
          pts[first_back][1] > 13.5 and not east_corridor(pts), pts[first_back - 1:first_back + 2])
    W.x, W.y = 3.0, 8.0                                        # inside aisle 2: 'clean the back' goes up it
    g, z = n.build_mission(['back'])
    pts = [(x, y) for (x, y, _) in g]
    check('the back from inside aisle 2: straight up it (unchanged)', abs(pts[1][1] - 14.8) < 0.05
          and abs(pts[1][0] - 3.0) < 0.05, pts[:3])
    # going home from the back's east end: two via goals first
    W.__init__()
    n = m.CoverageNav2()
    W.x, W.y = 10.4, 15.0
    n.on_command(T.P(data=json.dumps({'action': 'home'})))
    T.run_until(lambda: len(W.sent) > 0, 5)
    check("home from the back's east end: first along the back strip to aisle 3",
          bool(W.sent) and abs(W.sent[0][1] - aisle3) < 0.05 and abs(W.sent[0][2] - 14.8) < 0.05, W.sent[:2])
    check('... the goals: via1, via2, HOME', [str(x[2]) for x in n.goals[:3]] == ['via1', 'via2', 'HOME']
          and n.goal_zones[:3] == ['home', 'home', 'home'], n.goals[:3])
    W.__init__()
    n = m.CoverageNav2()
    W.x, W.y = 3.0, 8.0                                        # inside aisle 2: home as before
    n.on_command(T.P(data=json.dumps({'action': 'home'})))
    T.run_until(lambda: len(W.sent) > 0, 5)
    check('home from inside aisle 2: straight to the dock, as before', bool(W.sent)
          and abs(W.sent[0][1]) < 0.05 and abs(W.sent[0][2]) < 0.05 and len(n.goals) == 1, W.sent[:1])
    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: ' + '; '.join(FAILS))
    return 0 if not FAILS else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
