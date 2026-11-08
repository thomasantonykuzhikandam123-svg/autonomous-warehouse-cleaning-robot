#!/usr/bin/env python3
"""
Coverage tracker — grid-based, thesis-grade coverage metric.

Overlays a grid matching the free-space map. Marks a cell cleaned when the
robot's cleaning brush (radius) passes over it. Coverage % = cleaned free
cells / total free cells * 100 — directly comparable to the MATLAB result.

Publishes:
  /coverage_markers  (MarkerArray) — green paint for RViz
  /coverage_percent  (Float32)     — live coverage %
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
import math


class CoverageTracker(Node):
    def __init__(self):
        super().__init__('coverage_tracker')

        # ---- grid config (must match warehouse) ----
        self.resolution = 0.2
        self.spawn_x = 4.0
        self.spawn_y = 4.0
        self.width_cells = int(15.0 / self.resolution)
        self.height_cells = int(20.0 / self.resolution)

        self.cleaning_radius = 0.45   # brush radius (wider -> reaches walls)
        self.drop_distance = 0.15     # update coverage every this many metres

        # build free-space map (same walls/racks as planner)
        self.free = self.build_free_map()
        self.total_free = sum(sum(1 for c in row if c) for row in self.free)

        # cleaned grid
        self.cleaned = [[False] * self.width_cells for _ in range(self.height_cells)]

        self.last_x = None
        self.last_y = None
        self.next_marker_id = 0

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage_markers', 10)
        self.pct_pub = self.create_publisher(Float32, '/coverage_percent', 10)
        self.create_timer(1.0, self.publish_pct)

        self.get_logger().info(
            f'Coverage tracker ready — {self.total_free} free cells to clean')

    # ---- map (same geometry as planner/brain) ----
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
        # free = NOT wall
        free = [[w[gy][gx] == 0 for gx in range(self.width_cells)]
                for gy in range(self.height_cells)]
        return free

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_x is None:
            self.last_x, self.last_y = x, y
            self.mark_cleaned(x, y)
            return
        if math.hypot(x - self.last_x, y - self.last_y) >= self.drop_distance:
            self.last_x, self.last_y = x, y
            self.mark_cleaned(x, y)

    def mark_cleaned(self, x, y):
        # mark all FREE cells within brush radius as cleaned
        xw = x + self.spawn_x
        yw = y + self.spawn_y
        r_cells = int(self.cleaning_radius / self.resolution)
        cgx = int(xw / self.resolution)
        cgy = int(yw / self.resolution)
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy <= r_cells * r_cells:
                    gx, gy = cgx + dx, cgy + dy
                    if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
                        if self.free[gy][gx] and not self.cleaned[gy][gx]:
                            self.cleaned[gy][gx] = True
        # one paint blob at robot position
        self.paint(x, y)

    def paint(self, x, y):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'coverage'
        m.id = self.next_marker_id
        self.next_marker_id += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.02
        m.pose.orientation.w = 1.0
        m.scale.x = self.cleaning_radius * 2
        m.scale.y = self.cleaning_radius * 2
        m.scale.z = 0.03
        m.color.r = 0.2
        m.color.g = 0.9
        m.color.b = 0.4
        m.color.a = 0.5
        m.lifetime.sec = 0
        ma = MarkerArray()
        ma.markers.append(m)
        self.marker_pub.publish(ma)

    def publish_pct(self):
        cleaned_count = sum(sum(1 for c in row if c) for row in self.cleaned)
        pct = 100.0 * cleaned_count / self.total_free if self.total_free else 0.0
        msg = Float32()
        msg.data = pct
        self.pct_pub.publish(msg)
        self.get_logger().info(
            f'Coverage: {pct:.1f}%  ({cleaned_count}/{self.total_free} free cells)',
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()