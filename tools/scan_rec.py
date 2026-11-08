#!/usr/bin/env python3
"""scan_rec.py (v10.1) - record what the guard sees, so it can be replayed off-line.

    python3 ~/ros2_ws/src/tools/scan_rec.py <label> [max_s]

Records every /scan with the laser and robot poses the guard would use for it (TF at the scan's
stamp, else the latest, exactly as people_guard.py does), the robot's speed from /odom, Nav2's
/plan and the /map, into ~/runs/rec/<label>.npz.

It stops by itself when tour_test.py writes ~/runs/tour/<label>_laps.csv (the end of the tour),
after max_s seconds of simulation time (default 1200), a minute after the last scan (the stack
was stopped; v10.3: a minute it was running for, so a sleeping Mac or VM does not count), or on
Ctrl+C. It saves every 60 s of simulation time on the way, so a crash loses
at most a minute. It changes nothing in the stack: it only listens.

    python3 ~/ros2_ws/src/tools/veh_why.py <label>

replays the recording through the guard's own code and says where in the vehicle pipeline each
vehicle is lost.
"""
import json
import math
import os
import signal
import sys
import time

import numpy as np

VERSION = 'v10.3'
RUNS = os.path.expanduser('~/runs')
TOOLS = os.path.dirname(os.path.abspath(__file__))
SAVE_EVERY = 60.0        # s of simulation time between checkpoints
CHECK_EVERY = 1.0        # s (wall) between looks at the stop conditions
IDLE_STOP = 60.0         # s (wall) without a scan, once recording: the stack is gone, save and stop
SUSPEND_CAP = 2.0        # v10.3: one pass of the loop adds at most this to the time without a scan, so a
                         # suspended VM or a clock jump does not end the recording (tour_v101t: the Mac
                         # slept 2 h 10 min and scan_rec stopped the moment it woke, losing t 407-490)


class Stop(Exception):
    pass


def _request_stop(signum, frame):
    raise Stop()


def guard_version():
    try:
        for line in open(os.path.join(TOOLS, 'people_guard.py')):
            if line.startswith('VERSION'):
                return line.split('=', 1)[1].strip().strip("'\"")
    except OSError:
        pass
    return '?'


def save(path, rec, final):
    """Write the recording atomically (a temp file, then rename). Checkpoints are uncompressed
    (fast); the final file is compressed."""
    keys = ('t', 'ranges', 'laser', 'robot', 'exact', 'tf_age', 'recv', 'wall', 'odom_v', 'odom_age')
    n = min(len(rec[k]) for k in keys)          # a stop in the middle of a scan: drop that scan
    if n == 0:
        return 0
    B = rec['beams']
    arrays = dict(
        t=np.asarray(rec['t'][:n], dtype=np.float64),
        ranges=np.stack(rec['ranges'][:n]).astype(np.float32).reshape(n, B),
        laser=np.asarray(rec['laser'][:n], dtype=np.float64).reshape(n, 3),
        robot=np.asarray(rec['robot'][:n], dtype=np.float64).reshape(n, 3),
        exact=np.asarray(rec['exact'][:n], dtype=np.int8).reshape(n, 2),
        tf_age=np.asarray(rec['tf_age'][:n], dtype=np.float32).reshape(n, 2),
        recv=np.asarray(rec['recv'][:n], dtype=np.float32),
        wall=np.asarray(rec['wall'][:n], dtype=np.float64),
        odom_v=np.asarray(rec['odom_v'][:n], dtype=np.float32),
        odom_age=np.asarray(rec['odom_age'][:n], dtype=np.float32),
        meta=np.array(json.dumps(rec['meta'])),
    )
    if rec['map'] is not None:
        arrays['map_info'] = np.array(json.dumps(rec['map']['info']))
        arrays['map_data'] = rec['map']['data']
    if rec['plans']:
        arrays['plan_t'] = np.asarray([p[0] for p in rec['plans']], dtype=np.float64)
        arrays['plan_n'] = np.asarray([len(p[1]) for p in rec['plans']], dtype=np.int32)
        arrays['plan_xy'] = np.concatenate([p[1] for p in rec['plans']]).astype(np.float32).reshape(-1, 2)
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        (np.savez_compressed if final else np.savez)(f, **arrays)
    os.replace(tmp, path)
    return n


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv or argv[0].startswith('-'):
        print(__doc__)
        return 1
    label = argv[0]
    max_s = float(argv[1]) if len(argv) > 1 else 1200.0

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))

    out_dir = os.path.join(RUNS, 'rec')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f'{label}.npz')
    laps_path = os.path.join(RUNS, 'tour', f'{label}_laps.csv')
    started_wall = time.time()

    class Rec(Node):
        def __init__(self):
            super().__init__('scan_rec', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.buf = Buffer()
            self.tfl = TransformListener(self.buf, self)
            self.rec = dict(t=[], ranges=[], laser=[], robot=[], exact=[], tf_age=[], recv=[], wall=[],
                            odom_v=[], odom_age=[], plans=[], map=None, beams=None,
                            meta=dict(version=VERSION, label=label, guard=guard_version(),
                                      started=time.strftime('%Y-%m-%d %H:%M:%S')))
            self.odom_v, self.odom_t = 0.0, None
            self.skipped = 0              # scans with a different beam count, or no TF at all
            self.said = set()
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(LaserScan, '/scan', self.on_scan,
                                     QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Path, '/plan', self.on_plan, 5)
            self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
            print(f'scan_rec {VERSION}: recording /scan, TF, /odom, /plan and /map to {out_path} '
                  f'(guard on disk: {self.rec["meta"]["guard"]}); stops when tour_test writes '
                  f'{laps_path}, after {max_s:.0f} s of sim time, or on Ctrl+C', flush=True)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def once(self, key, text):
            if key not in self.said:
                self.said.add(key)
                print(text, flush=True)

        def on_map(self, msg):
            i = msg.info
            self.rec['map'] = dict(info=dict(width=i.width, height=i.height, resolution=i.resolution,
                                             origin_x=i.origin.position.x, origin_y=i.origin.position.y,
                                             origin_yaw=yaw_of(i.origin.orientation)),
                                   data=np.asarray(msg.data, dtype=np.int8))
            self.once('map', f'scan_rec: map {i.width} x {i.height} cells at {i.resolution:.3f} m, origin '
                             f'({i.origin.position.x:.2f}, {i.origin.position.y:.2f})')

        def on_odom(self, msg):
            self.odom_v = abs(msg.twist.twist.linear.x)
            self.odom_t = self.now()

        def on_plan(self, msg):
            pts = np.asarray([(p.pose.position.x, p.pose.position.y) for p in msg.poses], dtype=np.float32)
            if len(pts):
                self.rec['plans'].append((self.now(), pts.reshape(-1, 2)))

        def pose_of(self, frame, stamp):
            """As people_guard.py: TF at the stamp, else the latest. -> (x, y, yaw, z, exact, age)"""
            for exact, when in ((1, stamp), (0, Time())):
                try:
                    tf = self.buf.lookup_transform('map', frame, when)
                except Exception:
                    continue
                tr = tf.transform.translation
                ts = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                return tr.x, tr.y, yaw_of(tf.transform.rotation), tr.z, exact, ts
            return None

        def on_scan(self, scan):
            t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
            stamp = Time.from_msg(scan.header.stamp)
            laser = self.pose_of(scan.header.frame_id, stamp)
            robot = self.pose_of('base_link', stamp)
            if laser is None or robot is None:
                self.skipped += 1
                self.once('tf', f'scan_rec: no TF map -> {scan.header.frame_id} / base_link yet')
                return
            r = np.asarray(scan.ranges, dtype=np.float32)
            rec = self.rec
            if rec['beams'] is None:
                rec['beams'] = len(r)
                off = None
                try:
                    tf = self.buf.lookup_transform('base_link', scan.header.frame_id, Time())
                    off = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
                           yaw_of(tf.transform.rotation)]
                except Exception:
                    pass
                rec['meta'].update(frame=scan.header.frame_id, beams=len(r), angle_min=scan.angle_min,
                                   angle_max=scan.angle_max, angle_increment=scan.angle_increment,
                                   range_min=scan.range_min, range_max=scan.range_max,
                                   scan_time=scan.scan_time, laser_z=laser[3], laser_in_base=off)
                span = math.degrees(scan.angle_increment * (len(r) - 1))
                print(f'scan_rec: lidar frame {scan.header.frame_id}, {len(r)} beams from '
                      f'{math.degrees(scan.angle_min):.1f} to {math.degrees(scan.angle_max):.1f} deg '
                      f'(every {math.degrees(scan.angle_increment):.3f} deg, {span:.1f} deg covered), range '
                      f'{scan.range_min:.2f}-{scan.range_max:.2f} m, {laser[3]:.2f} m above the map'
                      + ('' if off is None else
                         f'; on the robot at ({off[0]:.2f}, {off[1]:.2f}, {off[2]:.2f}) m, yaw '
                         f'{math.degrees(off[3]):.0f} deg'), flush=True)
            if len(r) != rec['beams']:
                self.skipped += 1
                return
            now = self.now()
            rec['t'].append(t)
            rec['ranges'].append(r)
            rec['laser'].append(laser[:3])
            rec['robot'].append(robot[:3])
            rec['exact'].append((laser[4], robot[4]))
            rec['tf_age'].append((t - laser[5], t - robot[5]))
            rec['recv'].append(now - t)
            rec['wall'].append(time.time())
            rec['odom_v'].append(self.odom_v)
            rec['odom_age'].append(99.0 if self.odom_t is None else now - self.odom_t)

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    node = Rec()
    why = 'stopped'
    last_save_t = None
    last_check = time.monotonic()
    last_note = None
    idle, seen = 0.0, 0           # s running without a new scan (each pass counts at most SUSPEND_CAP)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            now_m = time.monotonic()
            if now_m - last_check < CHECK_EVERY:
                continue
            step, last_check = now_m - last_check, now_m
            ts = node.rec['t']
            if step > 10.0:
                print(f'scan_rec: this computer was paused for {step:.0f} s (sleep or suspend?); still '
                      f'recording', flush=True)
            if len(ts) != seen:
                seen, idle = len(ts), 0.0
            elif ts:
                idle += min(step, SUSPEND_CAP)
            if ts and idle > IDLE_STOP:
                why = f'no scan for {IDLE_STOP:.0f} s (stack stopped?)'
                break
            if ts:
                if last_save_t is None:
                    last_save_t = ts[0]
                if ts[-1] - last_save_t >= SAVE_EVERY:
                    save(out_path, node.rec, final=False)
                    last_save_t = ts[-1]
                if last_note is None or ts[-1] - last_note >= 60.0:
                    last_note = ts[-1]
                    ex = np.asarray(node.rec['exact'][-600:])
                    print(f'[{ts[-1]:8.1f}] scan_rec: {len(ts)} scans; TF at the scan stamp for the laser in '
                          f'{100.0 * ex[:, 0].mean():.0f} % of the last {len(ex)}', flush=True)
                if ts[-1] - ts[0] >= max_s:
                    why = f'{max_s:.0f} s of sim time recorded'
                    break
            try:
                if os.path.getmtime(laps_path) > started_wall:
                    why = 'tour_test.py finished (laps file written)'
                    break
            except OSError:
                pass
    except Stop:
        why = 'Ctrl+C'
    except Exception as e:              # rclpy shut down under us
        why = f'{type(e).__name__}'
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        n = 0
        try:
            n = save(out_path, node.rec, final=True)
        except Exception as e:
            print(f'scan_rec: SAVE FAILED: {e}', flush=True)
        ts = node.rec['t']
        if n:
            ex = np.asarray(node.rec['exact'])
            size = os.path.getsize(out_path) / 1e6
            print(f'scan_rec: {why}. Saved {n} scans, {ts[-1] - ts[0]:.0f} s of sim time, to {out_path} '
                  f'({size:.1f} MB). TF at the scan stamp: laser {100.0 * ex[:, 0].mean():.0f} %, robot '
                  f'{100.0 * ex[:, 1].mean():.0f} %. Map {"recorded" if node.rec["map"] is not None else "MISSING"}, '
                  f'{len(node.rec["plans"])} plans' + (f', {node.skipped} scans skipped' if node.skipped else '')
                  + '.', flush=True)
        else:
            print(f'scan_rec: {why}. Nothing recorded (no /scan with TF).', flush=True)
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
