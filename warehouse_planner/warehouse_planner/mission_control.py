#!/usr/bin/env python3
"""
Mission control — automatic complete coverage, zone-ordered.

Detects ALL reachable free space from the map (with safety inflation),
organizes it into Y-bands (front corridor, aisle body, back strip), and
sweeps each band boustrophedon-style. Skips slivers too thin for the robot.
Unreachable goals auto-skip. Returns home.

Ref: boustrophedon cellular decomposition (Choset 2000).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        self.resolution = 0.2
        self.spawn_x = 4.0
        self.spawn_y = 4.0
        self.width_cells = int(15.0 / self.resolution)
        self.height_cells = int(20.0 / self.resolution)
        self.inflate_cells = 3            # ~0.6m safety from walls
        self.column_spacing_m = 0.45      # pass spacing
        self.min_segment_cells = 3        # ignore free runs shorter than this
        self.home = (0.0, 0.0)

        self.free = self.build_free_map()
        self.goals = self.build_mission()
        self.goals.append((self.home[0], self.home[1], 'HOME'))

        self.gi = 0
        self.rx = self.ry = 0.0
        self.have_odom = False
        self.tol = 0.5
        self.sent = False
        self.last_send = 0.0
        self.resend_timeout = 16.0
        self.stuck_since = None
        self.stuck_skip_after = 22.0

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.wp_pub = self.create_publisher(PoseStamped, '/next_waypoint', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.create_timer(0.5, self.loop)
        self.create_timer(1.0, self.status)

        self.get_logger().info('========================================')
        self.get_logger().info(f'  AUTO COVERAGE — {len(self.goals)} waypoints')
        self.get_logger().info('========================================')

    # ---- map ----
    def add_wall(self, grid, xc, yc, lx, ly):
        gx0 = int((xc - lx / 2) / self.resolution)
        gy0 = int((yc - ly / 2) / self.resolution)
        gx1 = int((xc + lx / 2) / self.resolution)
        gy1 = int((yc + ly / 2) / self.resolution)
        for gy in range(max(0, gy0), min(self.height_cells, gy1 + 1)):
            for gx in range(max(0, gx0), min(self.width_cells, gx1 + 1)):
                grid[gy][gx] = 1

    def build_free_map(self):
        w = [[0] * self.width_cells for _ in range(self.height_cells)]
        self.add_wall(w, 7.5, 20.0, 15.0, 0.2)
        self.add_wall(w, 7.5, 0.0, 15.0, 0.2)
        self.add_wall(w, 0.0, 10.0, 0.2, 20.0)
        self.add_wall(w, 15.0, 10.0, 0.2, 20.0)
        self.add_wall(w, 2.5, 1.5, 3, 2)
        self.add_wall(w, 5.0, 1.5, 1, 2)
        self.add_wall(w, 12.0, 1.5, 4, 2)
        self.add_wall(w, 1.5, 11.5, 1, 12)
        self.add_wall(w, 5.5, 11.5, 1, 12)
        self.add_wall(w, 9.5, 11.5, 1, 12)
        self.add_wall(w, 13.5, 11.5, 1, 12)
        self.add_wall(w, 0.6, 9.0, 0.8, 1.2)
        self.add_wall(w, 14.4, 9.0, 0.8, 1.2)
        r = self.inflate_cells
        inf = [[0] * self.width_cells for _ in range(self.height_cells)]
        for gy in range(self.height_cells):
            for gx in range(self.width_cells):
                if w[gy][gx] == 1:
                    for dy in range(-r, r + 1):
                        for dx in range(-r, r + 1):
                            if dx * dx + dy * dy <= r * r:
                                ny, nx = gy + dy, gx + dx
                                if 0 <= ny < self.height_cells and 0 <= nx < self.width_cells:
                                    inf[ny][nx] = 1
        return [[inf[gy][gx] == 0 for gx in range(self.width_cells)]
                for gy in range(self.height_cells)]

    def cell_to_odom(self, gx, gy):
        return (gx * self.resolution + self.resolution / 2 - self.spawn_x,
                gy * self.resolution + self.resolution / 2 - self.spawn_y)

    def col_segments(self, gx, gy_lo, gy_hi):
        """Free vertical runs in column gx within [gy_lo, gy_hi]."""
        segs = []
        gy = gy_lo
        while gy <= gy_hi:
            if self.free[gy][gx]:
                lo = gy
                while gy <= gy_hi and self.free[gy][gx]:
                    gy += 1
                hi = gy - 1
                if hi - lo >= self.min_segment_cells:
                    segs.append((lo, hi))
            else:
                gy += 1
        return segs

    def sweep_band(self, gy_lo, gy_hi, label, flip_start=False):
        """Boustrophedon sweep of all columns in a Y-band."""
        goals = []
        step = max(1, int(self.column_spacing_m / self.resolution))
        flip = flip_start
        for gx in range(1, self.width_cells - 1, step):
            segs = self.col_segments(gx, gy_lo, gy_hi)
            for (lo, hi) in segs:
                bot = self.cell_to_odom(gx, lo)
                top = self.cell_to_odom(gx, hi)
                if not flip:
                    goals.append((bot[0], bot[1], f'{label}-c{gx}-bot'))
                    goals.append((top[0], top[1], f'{label}-c{gx}-top'))
                else:
                    goals.append((top[0], top[1], f'{label}-c{gx}-top'))
                    goals.append((bot[0], bot[1], f'{label}-c{gx}-bot'))
                flip = not flip
        return goals

    def build_mission(self):
        """Zone order: aisle body -> back strip -> front, by Y-band (world)."""
        # convert world Y bands to grid rows
        def wy_to_gy(wy):
            return int(wy / self.resolution)
        goals = []
        # Zone 1: aisle body (world Y 5.5..17.5) — the main aisles
        goals += self.sweep_band(wy_to_gy(5.0), wy_to_gy(17.8), 'aisle', False)
        # Zone 2: back strip (world Y 17.5..20) — behind racks
        goals += self.sweep_band(wy_to_gy(17.5), wy_to_gy(19.8), 'back', True)
        # Zone 3: front area (world Y 2.5..5.5) — corridor + open floor
        goals += self.sweep_band(wy_to_gy(2.3), wy_to_gy(5.5), 'front', False)
        return goals

    # ---- sequencing ----
    def odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        self.have_odom = True

    def send_goal(self):
        x, y, label = self.goals[self.gi]
        m = PoseStamped()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.orientation.w = 1.0
        self.wp_pub.publish(m)
        self.last_send = time.time()
        self.get_logger().info(f'{self.gi+1}/{len(self.goals)}: {label} ({x:.2f},{y:.2f})')

    def loop(self):
        if not self.have_odom or self.gi >= len(self.goals):
            return
        if not self.sent:
            self.send_goal()
            self.sent = True
            self.stuck_since = time.time()
            return
        x, y, label = self.goals[self.gi]
        d = math.hypot(self.rx - x, self.ry - y)
        if d < self.tol:
            self.gi += 1
            self.sent = False
            self.stuck_since = None
            if self.gi >= len(self.goals):
                self.get_logger().info('🎉 COVERAGE COMPLETE — home')
        else:
            if self.stuck_since and (time.time() - self.stuck_since) > self.stuck_skip_after:
                self.get_logger().warn(f'SKIP {label} (d={d:.2f})')
                self.gi += 1
                self.sent = False
                self.stuck_since = None
            elif time.time() - self.last_send > self.resend_timeout:
                self.send_goal()

    def status(self):
        m = String()
        m.data = 'COMPLETE' if self.gi >= len(self.goals) else \
                 f'{self.gi+1}/{len(self.goals)} {self.goals[self.gi][2]}'
        self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MissionControl())
    rclpy.shutdown()


if __name__ == '__main__':
    main()