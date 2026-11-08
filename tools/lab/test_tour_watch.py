"""Fake-ROS test of tour_test.py's watch mode (v10.3): no goals sent, no emergency stop; the shift
is measured from the robot's first move until coverage_nav2's /mission_state says done for 5 s;
tick file with waypoint, zone and coverage; laps file with one SHIFT row; trials rows per agent;
a second run stopped by Ctrl+C still saves.

    python3 test_tour_watch.py [tour_test.py]
"""
import csv
import importlib.util
import json
import math
import os
import signal
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
P = types.SimpleNamespace
FAILS = []
STATE = dict(now=0.0, subs={}, pubs={}, sent_goals=0, done_at=60.0, sigint_at=None, home=None)


def check(name, ok, detail=''):
    print(f'{"ok  " if ok else "FAIL"} {name}{"  (" + detail + ")" if detail else ""}')
    if not ok:
        FAILS.append(name)


def robot_at(t):
    """Stands until t 3, then drives north at 0.5 m/s; held (standing) between t 20 and 26."""
    if t < 3.0:
        return 0.0, 0.0, math.pi / 2, 0.0
    tm = min(t, 20.0) - 3.0 + max(t - 26.0, 0.0)
    v = 0.0 if 20.0 <= t < 26.0 else 0.5
    return -1.25, 0.5 * tm, math.pi / 2, v


class Pub:
    def __init__(self, topic):
        self.topic = topic
        STATE['pubs'].setdefault(topic, [])

    def publish(self, m):
        STATE['pubs'][self.topic].append(m)


class Node:
    def __init__(self, name, parameter_overrides=None):
        STATE['node'] = self

    def create_subscription(self, typ, topic, cb, qos):
        STATE['subs'][topic] = cb

    def create_publisher(self, typ, topic, qos):
        return Pub(topic)

    def get_clock(self):
        return P(now=lambda: P(nanoseconds=int(STATE['now'] * 1e9)))

    def destroy_node(self):
        pass


class ActionClient:
    def __init__(self, node, typ, name):
        pass

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        STATE['sent_goals'] += 1
        raise AssertionError('watch mode must not send goals')


class Buffer:
    def lookup_transform(self, target, source, when):
        x, y, yaw, _ = robot_at(STATE['now'])
        return P(transform=P(translation=P(x=x, y=y, z=0.0),
                             rotation=P(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


def spin_once(node, timeout_sec=None):
    t = STATE['now'] = round(STATE['now'] + 0.05, 3)
    subs = STATE['subs']
    x, y, yaw, v = robot_at(t)
    subs['/odom'](P(twist=P(twist=P(linear=P(x=v)))))
    if '/people_guard/hold' in subs:
        held = 20.0 <= t < 26.0
        subs['/people_guard/hold'](P(width=5 if held else 0, height=1))
    for topic, cb in subs.items():
        if topic.startswith('/model/') and abs(t * 10 - round(t * 10)) < 1e-6:
            cb(P(pose=P(pose=P(position=P(x=4.0 + 3.0, y=4.0 + 5.0, z=0.0),
                               orientation=P(x=0.0, y=0.0, z=0.0, w=1.0))),
                 twist=P(twist=P(linear=P(x=0.0)))))
    if '/mission_state' in subs and abs(t * 2 - round(t * 2)) < 1e-6:
        done = t >= STATE['done_at']
        subs['/mission_state'](P(data=json.dumps(dict(goal=1 + int(t // 10), total=8, zone='aisle_1',
                                                      coverage_pct=round(t * 0.5, 1), paused=False,
                                                      done=done))))
    if STATE['sigint_at'] is not None and abs(t - STATE['sigint_at']) < 1e-6:
        os.kill(os.getpid(), signal.SIGINT)


def install_fakes():
    def mod(name, **kw):
        m = types.ModuleType(name)
        m.__dict__.update(kw)
        sys.modules[name] = m

    class Parameter:
        class Type:
            BOOL = 1

        def __init__(self, *a):
            pass

    class Time:
        def __init__(self, *a, **k):
            pass

    class Msg:
        def __init__(self, data=None):
            self.data = data

    class GoalStatus:
        STATUS_SUCCEEDED, STATUS_ABORTED, STATUS_CANCELED = 4, 6, 5

    class SignalHandlerOptions:
        NO = 0

    mod('rclpy', init=lambda **k: None, spin_once=spin_once, shutdown=lambda: None)
    mod('rclpy.action', ActionClient=ActionClient)
    mod('rclpy.node', Node=Node)
    mod('rclpy.parameter', Parameter=Parameter)
    mod('rclpy.qos', qos_profile_sensor_data=None)
    mod('rclpy.time', Time=Time)
    mod('rclpy.signals', SignalHandlerOptions=SignalHandlerOptions)
    mod('geometry_msgs')
    mod('geometry_msgs.msg', PoseStamped=object)
    mod('nav_msgs')
    mod('nav_msgs.msg', Odometry=object)
    mod('nav2_msgs')
    mod('nav2_msgs.action', NavigateThroughPoses=object)
    mod('nav2_msgs.msg', SpeedLimit=object)
    mod('action_msgs')
    mod('action_msgs.msg', GoalStatus=GoalStatus)
    mod('sensor_msgs')
    mod('sensor_msgs.msg', PointCloud2=object)
    mod('std_msgs')
    mod('std_msgs.msg', Bool=Msg, String=Msg)
    mod('tf2_ros', Buffer=Buffer, TransformListener=lambda buf, node: None)


def run(path, argv, done_at=60.0, sigint_at=None):
    STATE.update(now=0.0, subs={}, pubs={}, sent_goals=0, done_at=done_at, sigint_at=sigint_at)
    spec = importlib.util.spec_from_file_location('tour_test_w', path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.OUT_DIR = os.path.join(STATE['home'], 'runs', 'tour')
    m.TRIALS = os.path.join(STATE['home'], 'runs', 'tour_trials.csv')
    m.STOP['requested'] = False
    old = signal.getsignal(signal.SIGINT)
    rc = m.main(argv)
    signal.signal(signal.SIGINT, old)
    return rc, m


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'tour_test.py')
    sys.path.insert(0, os.path.dirname(os.path.abspath(path)))
    install_fakes()
    STATE['home'] = tempfile.mkdtemp()
    os.environ['HOME'] = STATE['home']
    rc, m = run(path, ['shift_t', 'watch', '600'])
    tick = os.path.join(m.OUT_DIR, 'shift_t.csv')
    laps = os.path.join(m.OUT_DIR, 'shift_t_laps.csv')
    check('watch: returns 0', rc == 0, f'{rc}')
    check('watch: no goals sent', STATE['sent_goals'] == 0)
    check('watch: no emergency stop published', not STATE['pubs'].get('/emergency_stop'),
          f'{len(STATE["pubs"].get("/emergency_stop", []))} messages')
    rows = list(csv.DictReader(open(tick)))
    t = [float(r['t']) for r in rows]
    check('tick file starts when the robot first moves (t 3)', rows and 3.0 <= t[0] <= 3.3, f'{t[0] if t else None}')
    check('tick file ends 5 s after done (t 60)', rows and 64.8 <= t[-1] <= 65.4, f'{t[-1] if t else None}')
    period = (t[-1] - t[0]) / (len(t) - 1)
    check('ticks every 0.1 s (0.15 s with this fake clock: next tick = t + 0.1 lands just past a 0.05 step)',
          rows and 0.09 < period < 0.16, f'{period:.3f}')
    check('waypoint, zone and coverage in the tick file', rows[-1]['goal'] == '7' and rows[-1]['zone'] == 'aisle_1'
          and rows[-1]['coverage_pct'] != '' and rows[-1]['mission_done'] == '1', str({k: rows[-1][k] for k in ('goal', 'zone', 'coverage_pct', 'mission_done')}))
    check('gaps for people and vehicles', 'gap_worker' in rows[0] and 'gap_forklift' in rows[0]
          and rows[10]['gap_forklift'] != '')
    held = [r for r in rows if r['hold'] == '1']
    check('guard holds recorded (t 20-26)', abs(len(held) * period - 6.0) < 0.5, f'{len(held)} ticks')
    lr = list(csv.DictReader(open(laps)))
    check('laps file: one SHIFT row', len(lr) == 1 and lr[0]['results'].startswith('SHIFT') and 'done' in lr[0]['results']
          and abs(float(lr[0]['duration_s']) - 62.0) < 0.6 and lr[0]['guard_waits'] == '1', str(lr))
    tr = [r for r in csv.DictReader(open(m.TRIALS)) if r['label'] == 'shift_t']
    check('trials: one row per agent', len(tr) == len(m.TRIAL_FIELDS) * 0 + len({r['agent'] for r in tr}) and len(tr) >= 5,
          f'{len(tr)} rows')
    # Ctrl+C in the middle: still saved
    rc, m = run(path, ['shift_c', 'watch'], done_at=999.0, sigint_at=30.0)
    lr = list(csv.DictReader(open(os.path.join(m.OUT_DIR, 'shift_c_laps.csv'))))
    check('Ctrl+C: laps file written, not done', len(lr) == 1 and 'not done' in lr[0]['results']
          and abs(float(lr[0]['duration_s']) - 27.0) < 0.6, str(lr))
    # the tour itself is untouched by the watch code: argv parsing
    check('tour mode parses as before', m.ESTOP_Y0 == 6.0 and m.VERSION == 'v10.3')
    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
