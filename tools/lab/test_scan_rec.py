"""Fake-ROS test of scan_rec.py: map, 300 scans with TF (some only as 'latest'), /odom, /plan;
stop when the laps file appears; a second run stopped by Ctrl+C mid-way. Checks the npz."""
import importlib.util
import json
import math
import os
import signal
import sys
import tempfile
import time
import types

import numpy as np

P = types.SimpleNamespace
FAILS = []


def check(name, ok, detail=''):
    print(f'{"ok  " if ok else "FAIL"} {name}{"  (" + detail + ")" if detail else ""}')
    if not ok:
        FAILS.append(name)


STATE = dict(k=0, subs={}, now=100.0, n_scans=300, laps_at=280, sigint_at=None, home=None, label='x',
             jump_at=None, jump=0.0)


class FakeTime:
    """time for scan_rec with a clock that can jump (a suspended VM): both clocks jump forward."""
    def time(self):
        return time.time() + STATE['jump']

    def monotonic(self):
        return time.monotonic() + STATE['jump']

    def strftime(self, *a):
        return time.strftime(*a)

    def sleep(self, s):
        time.sleep(s)


class Node:
    def __init__(self, name, parameter_overrides=None):
        STATE['node'] = self

    def create_subscription(self, typ, topic, cb, qos):
        STATE['subs'][topic] = cb

    def get_clock(self):
        return P(now=lambda: P(nanoseconds=int(STATE['now'] * 1e9)))

    def destroy_node(self):
        pass


class Time:
    def __init__(self, *a, **k):
        self.latest = True

    @staticmethod
    def from_msg(m):
        t = Time()
        t.latest = False
        t.t = m.sec + m.nanosec * 1e-9
        return t


class Buffer:
    def lookup_transform(self, target, source, when):
        k = STATE['k']
        if not when.latest and k % 10 == 3:          # every 10th scan: TF at the stamp not there yet
            raise RuntimeError('extrapolation into the future')
        t = STATE['now'] if when.latest else when.t
        x, y = 0.01 * k, 2.0
        if target == 'base_link':
            return P(header=P(stamp=P(sec=int(t), nanosec=0)),
                     transform=P(translation=P(x=0.1, y=0.0, z=0.28), rotation=P(x=0.0, y=0.0, z=0.0, w=1.0)))
        z = 0.28 if source == 'lidar_link' else 0.0
        return P(header=P(stamp=P(sec=int(t), nanosec=int((t - int(t)) * 1e9))),
                 transform=P(translation=P(x=x, y=y, z=z), rotation=P(x=0.0, y=0.0, z=math.sin(0.25), w=math.cos(0.25))))


def spin_once(node, timeout_sec=None):
    subs = STATE['subs']
    k = STATE['k']
    if k == 0 and '/map' in subs:
        subs['/map'](P(info=P(width=4, height=3, resolution=0.5, origin=P(position=P(x=-1.0, y=-2.0),
                                                                             orientation=P(x=0, y=0, z=0, w=1))),
                       data=[0, 100, 0, 0, -1, 0, 0, 0, 100, 0, 0, 0]))
    if k >= STATE['n_scans']:
        time.sleep(0.01)
        return
    STATE['now'] = 100.0 + 0.1 * k
    t = STATE['now'] - 0.02
    sec, ns = int(t), int(round((t - int(t)) * 1e9))
    subs['/odom'](P(twist=P(twist=P(linear=P(x=-0.5 if k % 2 else 0.5)))))
    if k % 25 == 0:
        subs['/plan'](P(poses=[P(pose=P(position=P(x=0.05 * j, y=1.0))) for j in range(40 + k // 25)]))
    r = np.full(360, np.inf, dtype=np.float32)
    r[::7] = 1.0 + 0.001 * k
    subs['/scan'](P(header=P(stamp=P(sec=sec, nanosec=ns), frame_id='lidar_link'), ranges=list(r),
                    angle_min=-math.pi, angle_max=math.pi - 2 * math.pi / 360, angle_increment=2 * math.pi / 360,
                    range_min=0.12, range_max=12.0, scan_time=0.1))
    STATE['k'] = k + 1
    if STATE['jump_at'] is not None and k + 1 == STATE['jump_at']:
        STATE['jump'] += 7800.0                      # 2 h 10 min asleep between two scans
        time.sleep(0.6)                              # and the first scan after waking comes a bit later
    if STATE['laps_at'] is not None and k + 1 == STATE['laps_at']:
        d = os.path.join(STATE['home'], 'runs', 'tour')
        os.makedirs(d, exist_ok=True)
        open(os.path.join(d, STATE['label'] + '_laps.csv'), 'w').write('label,lap\n')
        time.sleep(1.2)
    if STATE['sigint_at'] is not None and k + 1 == STATE['sigint_at']:
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

    class QoSProfile:
        def __init__(self, **k):
            pass

    class Enum:
        TRANSIENT_LOCAL = 1
        RELIABLE = 1
        BEST_EFFORT = 2

    class SignalHandlerOptions:
        NO = 0

    mod('rclpy', init=lambda **k: None, spin_once=spin_once, shutdown=lambda: None, ok=lambda: True)
    mod('rclpy.node', Node=Node)
    mod('rclpy.parameter', Parameter=Parameter)
    mod('rclpy.qos', QoSProfile=QoSProfile, DurabilityPolicy=Enum, ReliabilityPolicy=Enum,
        qos_profile_sensor_data=None)
    mod('rclpy.time', Time=Time)
    mod('rclpy.signals', SignalHandlerOptions=SignalHandlerOptions)
    mod('nav_msgs')
    mod('nav_msgs.msg', OccupancyGrid=object, Odometry=object, Path=object)
    mod('sensor_msgs')
    mod('sensor_msgs.msg', LaserScan=object)
    mod('tf2_ros', Buffer=Buffer, TransformListener=lambda buf, node: None)


def run(label, laps_at, sigint_at, n_scans=300, jump_at=None):
    STATE.update(k=0, subs={}, label=label, laps_at=laps_at, sigint_at=sigint_at, n_scans=n_scans,
                 jump_at=jump_at, jump=0.0)
    spec = importlib.util.spec_from_file_location('scan_rec_t', sys.argv[1])
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.time = FakeTime()
    m.SAVE_EVERY = 5.0
    m.CHECK_EVERY = 0.0
    m.IDLE_STOP = 0.5
    m.RUNS = os.path.join(STATE['home'], 'runs')
    old = (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))
    rc = m.main([label])
    signal.signal(signal.SIGINT, old[0])
    signal.signal(signal.SIGTERM, old[1])
    return rc, os.path.join(STATE['home'], 'runs', 'rec', label + '.npz')


def main():
    install_fakes()
    STATE['home'] = tempfile.mkdtemp()
    rc, path = run('t_laps', laps_at=280, sigint_at=None)
    check('stops by itself when the laps file is written', rc == 0 and os.path.exists(path))
    R = np.load(path, allow_pickle=False)
    meta = json.loads(str(R['meta']))
    n = len(R['t'])
    check('recorded about 280 scans before the laps file', 275 <= n <= 290, f'{n}')
    check('ranges shape (n, 360) float32', R['ranges'].shape == (n, 360) and R['ranges'].dtype == np.float32)
    check('ranges kept exactly (inf stays inf)', np.isinf(R['ranges'][0, 1]) and abs(R['ranges'][5, 0] - 1.005) < 1e-6)
    check('laser pose and yaw from TF', abs(R['laser'][10, 1] - 2.0) < 1e-9 and abs(R['laser'][10, 2] - 0.5) < 1e-9)
    ex = R['exact']
    check('TF at the stamp missing for every 10th scan, flagged', abs(ex[:, 0].mean() - 0.9) < 0.02, f'{ex[:, 0].mean():.3f}')
    check('meta: lidar config and laser height', meta['beams'] == 360 and abs(meta['laser_z'] - 0.28) < 1e-9 and
          meta['frame'] == 'lidar_link' and abs(meta['range_max'] - 12.0) < 1e-9)
    check('laser offset on the robot', meta['laser_in_base'] is not None and abs(meta['laser_in_base'][0] - 0.1) < 1e-9)
    mi = json.loads(str(R['map_info']))
    check('map recorded', mi['width'] == 4 and mi['height'] == 3 and list(R['map_data'][:3]) == [0, 100, 0]
          and R['map_data'].dtype == np.int8 and R['map_data'][4] == -1)
    check('plans recorded with their lengths', len(R['plan_t']) >= 11 and int(R['plan_n'][1]) == 41 and
          R['plan_xy'].shape == (int(R['plan_n'].sum()), 2))
    check('odom speed recorded (abs)', abs(R['odom_v'][3] - 0.5) < 1e-6 and float(R['odom_age'][3]) < 0.2)
    check('receive lag = now - stamp', abs(float(np.median(R['recv'])) - 0.02) < 1e-3)
    # Ctrl+C in the middle: still saved, nothing half-written
    rc, path = run('t_int', laps_at=None, sigint_at=150)
    R = np.load(path, allow_pickle=False)
    n = len(R['t'])
    check('Ctrl+C: saved what it had', 148 <= n <= 151 and R['ranges'].shape[0] == n and len(R['odom_v']) == n, f'{n}')
    check('no temp file left behind', not os.path.exists(path + '.tmp'))
    # the stack goes away mid-run (no laps file, no Ctrl+C): stops after IDLE_STOP without scans
    t0 = time.time()
    rc, path = run('t_idle', laps_at=None, sigint_at=None, n_scans=100)
    R = np.load(path, allow_pickle=False)
    check('no scans for a while: stops and saves', len(R['t']) == 100 and time.time() - t0 < 10.0,
          f'{len(R["t"])} scans, {time.time() - t0:.1f} s')
    # v10.3: the computer sleeps for 2 h 10 min in the middle: the recording carries on
    rc, path = run('t_sleep', laps_at=280, sigint_at=None, jump_at=120)
    R = np.load(path, allow_pickle=False)
    n = len(R['t'])
    check('a 2 h sleep mid-run does not stop the recording', 275 <= n <= 290, f'{n} scans')
    print('ALL OK' if not FAILS else f'{len(FAILS)} FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
