#!/usr/bin/env python3
"""worker_clearance.py (v3) - robot-to-worker distance during Nav2 trials.

Start it BEFORE sending goals, leave it running, Ctrl+C when finished:
    python3 ~/ros2_ws/src/tools/worker_clearance.py <label>

One row per Nav2 goal is printed and appended to ~/runs/people_trials.csv.

Robot pose  : TF map -> base_link.
Worker pose : predicted from the scripted actor trajectory (world frame,
              6.0 s loop starting at sim time 0), shifted into the map
              frame (map = world - (4, 4)).
seen_pct    : self-check of that prediction against /scan. The % of ticks
              (worker 0.5-7.5 m away) where a lidar return lies within
              0.35 m of where the script says she is. If this is low,
              the prediction is off and the distances can't be trusted.
contact     : centre distance < 0.56 m (chassis corner 0.36 + person 0.20).
approaching : robot moving toward her (> 0.05 m/s) at that moment. Separates
              the robot driving into her from her walking into the robot.

v2: also sees NavigateThroughPoses goals (people_trials.py route=lanes).
    Same columns and CSV as v1; the measurements are unchanged.
v3: while she is within 2.5 m, every 0.1 s tick also goes to
    ~/runs/traces/<label>.csv (robot and worker positions, distance, robot
    speed), so each contact can be plotted and explained. No change to the
    summary rows. Quiet exit when stack.sh stop ends it.
"""
import csv
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from tf2_ros import Buffer, TransformListener

# worker_test script, world frame: (time s, x, y). Keep in sync with the SDF.
WAYPOINTS = [(0.0, 2.4, 8.0), (2.2, 4.6, 8.0), (3.0, 4.6, 8.0),
             (5.2, 2.4, 8.0), (6.0, 2.4, 8.0)]
PERIOD = 6.0
MAP_OFFSET = (4.0, 4.0)
CONTACT_M = 0.36 + 0.20
SEEN_M = 0.35
CSV_PATH = os.path.expanduser('~/runs/people_trials.csv')
TRACE_DIR = os.path.expanduser('~/runs/traces')
TRACE_M = 2.5
TRACE_FIELDS = ['label', 'goal', 't', 'robot_x', 'robot_y', 'robot_yaw', 'robot_speed',
                'worker_x', 'worker_y', 'dist', 'approaching']
RESULT = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def worker_map_xy(t):
    """Scripted worker position at sim time t, in the map frame."""
    t %= PERIOD
    for (t0, x0, y0), (t1, x1, y1) in zip(WAYPOINTS, WAYPOINTS[1:]):
        if t0 <= t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return (x0 + f * (x1 - x0) - MAP_OFFSET[0],
                    y0 + f * (y1 - y0) - MAP_OFFSET[1])
    return WAYPOINTS[0][1] - MAP_OFFSET[0], WAYPOINTS[0][2] - MAP_OFFSET[1]


def seen_in_scan(scan, rx, ry, yaw, wx, wy):
    """True if any valid lidar return lies within SEEN_M of (wx, wy)."""
    for i, r in enumerate(scan.ranges):
        if not (scan.range_min < r < scan.range_max):
            continue
        a = yaw + scan.angle_min + i * scan.angle_increment
        if math.hypot(rx + r * math.cos(a) - wx, ry + r * math.sin(a) - wy) < SEEN_M:
            return True
    return False


class Probe(Node):
    def __init__(self, label):
        super().__init__('worker_clearance', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.label = label
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.scan = None
        self.goal = None
        self.done = set()
        self.recoveries = {}
        self.n = 0
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        for action, typ in (('navigate_to_pose', NavigateToPose),
                            ('navigate_through_poses', NavigateThroughPoses)):
            self.create_subscription(GoalStatusArray, f'/{action}/_action/status',
                                     self.on_status, 10)
            self.create_subscription(typ.Impl.FeedbackMessage,
                                     f'/{action}/_action/feedback', self.on_feedback, 10)
        self.create_timer(0.1, self.tick)
        print(f'[{label}] waiting for goals...', flush=True)

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_scan(self, msg):
        self.scan = msg

    def on_feedback(self, msg):
        gid = bytes(msg.goal_id.uuid).hex()[:8]
        self.recoveries[gid] = msg.feedback.number_of_recoveries

    def on_status(self, msg):
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()[:8]
            if s.status == 2 and self.goal is None and gid not in self.done:
                self.start(gid)
            elif s.status in RESULT and gid == self.goal:
                self.finish(RESULT[s.status])

    def start(self, gid):
        self.goal = gid
        self.t0 = self.sim_now()
        self.prev = None
        self.min_d = float('inf')
        self.approach_at_min = False
        self.contact = 0
        self.contact_approach = 0
        self.near_s = 0.0
        self.in_range = 0
        self.seen = 0
        self.trace = []
        print(f'  goal {gid} started', flush=True)

    def tick(self):
        if self.goal is None:
            return
        try:
            tf = self.buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        t = self.sim_now()
        tr, q = tf.transform.translation, tf.transform.rotation
        rx, ry = tr.x, tr.y
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        wx, wy = worker_map_xy(t)
        d = math.hypot(wx - rx, wy - ry)

        approaching = False
        speed = 0.0
        if self.prev is not None and t > self.prev[0]:
            dt = t - self.prev[0]
            vx, vy = (rx - self.prev[1]) / dt, (ry - self.prev[2]) / dt
            speed = math.hypot(vx, vy)
            approaching = (speed > 0.05
                           and vx * (wx - rx) + vy * (wy - ry) > 0)
            if d < 1.0:
                self.near_s += dt
        self.prev = (t, rx, ry)
        if d < TRACE_M:
            self.trace.append([self.label, self.goal, round(t, 2), round(rx, 3), round(ry, 3),
                               round(yaw, 3), round(speed, 3), round(wx, 3), round(wy, 3),
                               round(d, 3), int(approaching)])

        if d < self.min_d:
            self.min_d, self.approach_at_min = d, approaching
        if d < CONTACT_M:
            self.contact += 1
            self.contact_approach += int(approaching)

        if self.scan is not None and 0.5 < d < 7.5:
            self.in_range += 1
            if seen_in_scan(self.scan, rx, ry, yaw, wx, wy):
                self.seen += 1

    def finish(self, result):
        gid = self.goal
        self.done.add(gid)
        self.goal = None
        self.n += 1
        row = {
            'label': self.label,
            'pass': self.n,
            'goal': gid,
            'result': result,
            'duration_s': round(self.sim_now() - self.t0, 1),
            'recoveries': self.recoveries.get(gid, 0),
            'min_dist_m': round(self.min_d, 2),
            'contact': 'YES' if self.contact else 'no',
            'approaching_at_min': 'yes' if self.approach_at_min else 'no',
            'contact_ticks': self.contact,
            'contact_ticks_approaching': self.contact_approach,
            'within_1m_s': round(self.near_s, 1),
            'seen_pct': round(100 * self.seen / self.in_range) if self.in_range else -1,
        }
        print(' | '.join(f'{k}={v}' for k, v in row.items()), flush=True)
        new_file = not os.path.exists(CSV_PATH)
        os.makedirs(os.path.dirname(CSV_PATH), exist_ok=True)
        with open(CSV_PATH, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(row))
            if new_file:
                writer.writeheader()
            writer.writerow(row)
        if self.trace:
            os.makedirs(TRACE_DIR, exist_ok=True)
            path = os.path.join(TRACE_DIR, f'{self.label}.csv')
            new_trace = not os.path.exists(path)
            with open(path, 'a', newline='') as f:
                w = csv.writer(f)
                if new_trace:
                    w.writerow(TRACE_FIELDS)
                w.writerows(self.trace)


def main():
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        ExternalShutdownException = KeyboardInterrupt
    rclpy.init()
    node = Probe(sys.argv[1] if len(sys.argv) > 1 else 'unlabelled')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass     # Ctrl+C, or stack.sh stop


if __name__ == '__main__':
    main()
