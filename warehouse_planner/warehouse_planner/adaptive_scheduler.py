#!/usr/bin/env python3
"""
Adaptive coverage scheduler.

Aisle selection (your specification):
  1. Prefer aisle with LEAST traffic (fewest vehicles / lowest heat)
  2. On tie, prefer aisle with SHORTEST path from current position
  3. Always a definite choice — never confused

score(aisle) = traffic_weight * traffic + distance_weight * path_distance
Robot picks the LOWEST score.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray, String
import math
import time


class CleaningRegion:
    def __init__(self, name, waypoints, sample_points, entry_point):
        self.name = name
        self.waypoints = waypoints
        self.sample_points = sample_points
        self.entry_point = entry_point   # for distance calc
        self.cleaned = False


class AdaptiveScheduler(Node):

    def __init__(self):
        super().__init__('adaptive_scheduler')

        self.heatmap_resolution = 0.4
        self.spawn_x = 4.0
        self.spawn_y = 4.0
        self.heatmap_width = int(15.0 / self.heatmap_resolution)
        self.heatmap_height = int(20.0 / self.heatmap_resolution)
        self.heatmap_values = []

        # Scoring weights — traffic dominates, distance is tiebreaker
        self.traffic_weight = 10.0
        self.distance_weight = 1.0

        aisle_1 = CleaningRegion(
            'aisle_1',
            waypoints=[(-0.5, 0.0), (-0.5, 10.0), (-0.5, 0.0)],
            sample_points=[(-0.5, 3.0), (-0.5, 6.0), (-0.5, 9.0)],
            entry_point=(-0.5, 0.0)
        )
        aisle_2 = CleaningRegion(
            'aisle_2',
            waypoints=[(3.5, 0.0), (3.5, 10.0), (3.5, 0.0)],
            sample_points=[(3.5, 3.0), (3.5, 6.0), (3.5, 9.0)],
            entry_point=(3.5, 0.0)
        )
        aisle_3 = CleaningRegion(
            'aisle_3',
            waypoints=[(7.5, 0.0), (7.5, 10.0), (7.5, 0.0)],
            sample_points=[(7.5, 3.0), (7.5, 6.0), (7.5, 9.0)],
            entry_point=(7.5, 0.0)
        )
        home = CleaningRegion(
            'home',
            waypoints=[(0.0, 0.0)],
            sample_points=[(0.0, 0.0)],
            entry_point=(0.0, 0.0)
        )

        self.regions = [aisle_1, aisle_2, aisle_3]
        self.home_region = home

        self.current_region = None
        self.current_wp_idx = 0
        self.mission_started = False
        self.mission_complete = False
        self.deferred = {r.name: 0 for r in self.regions}
        self.last_pub_time = 0.0
        self.last_pub_wp = None

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.waypoint_tolerance = 0.5
        self.max_defer = 5
        self.republish_period = 10.0

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.heatmap_sub = self.create_subscription(
            Float32MultiArray, '/heatmap_values', self.heatmap_callback, 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/next_waypoint', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)

        self.create_timer(5.0, self.start_mission_once)
        self.create_timer(1.0, self.scheduler_tick)

        self.get_logger().info('Adaptive scheduler ready (traffic + shortest-path selection)')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def heatmap_callback(self, msg):
        self.heatmap_values = list(msg.data)

    def get_traffic_at(self, x, y):
        if not self.heatmap_values:
            return 0.0
        xw = x + self.spawn_x
        yw = y + self.spawn_y
        gx = int(xw / self.heatmap_resolution)
        gy = int(yw / self.heatmap_resolution)
        if gx < 0 or gy < 0 or gx >= self.heatmap_width or gy >= self.heatmap_height:
            return 0.0
        idx = gy * self.heatmap_width + gx
        return self.heatmap_values[idx] if idx < len(self.heatmap_values) else 0.0

    def region_traffic(self, region):
        if not region.sample_points:
            return 0.0
        return sum(self.get_traffic_at(x, y) for x, y in region.sample_points) / len(region.sample_points)

    def region_distance(self, region):
        """Distance from robot's current position to the region's entry point."""
        ex, ey = region.entry_point
        return math.hypot(self.robot_x - ex, self.robot_y - ey)

    def region_score(self, region):
        """Lower score = better choice. Traffic dominates, distance breaks ties."""
        if region.cleaned:
            return float('inf')
        # Force selection if deferred too many times (avoid starvation)
        if self.deferred[region.name] >= self.max_defer:
            return -1000.0
        traffic = self.region_traffic(region)
        distance = self.region_distance(region)
        return self.traffic_weight * traffic + self.distance_weight * distance

    def select_next_region(self):
        candidates = [r for r in self.regions if not r.cleaned]
        if not candidates:
            return None
        candidates.sort(key=lambda r: self.region_score(r))
        chosen = candidates[0]
        for c in candidates:
            if c is not chosen:
                self.deferred[c.name] += 1
        return chosen

    def start_mission_once(self):
        if self.mission_started:
            return
        self.mission_started = True
        nxt = self.select_next_region()
        if nxt:
            self.current_region = nxt
            self.current_wp_idx = 0
            self.publish_status(
                f'Starting: {nxt.name} '
                f'(traffic={self.region_traffic(nxt):.2f}, '
                f'dist={self.region_distance(nxt):.1f})')
            self.publish_wp(force=True)

    def publish_wp(self, force=False):
        if self.current_region is None:
            return
        if self.current_wp_idx >= len(self.current_region.waypoints):
            return
        x, y = self.current_region.waypoints[self.current_wp_idx]
        wp = (x, y)
        if not force and self.last_pub_wp == wp:
            if time.time() - self.last_pub_time < self.republish_period:
                return
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.waypoint_pub.publish(msg)
        self.last_pub_wp = wp
        self.last_pub_time = time.time()

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def scheduler_tick(self):
        if not self.mission_started or self.mission_complete:
            return
        if self.current_region is None:
            return
        wp = self.current_region.waypoints[self.current_wp_idx]
        d = math.hypot(self.robot_x - wp[0], self.robot_y - wp[1])
        if d < self.waypoint_tolerance:
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.current_region.waypoints):
                self.current_region.cleaned = True
                self.publish_status(f'✓ {self.current_region.name} complete')
                self.deferred[self.current_region.name] = 0
                nxt = self.select_next_region()
                if nxt:
                    self.current_region = nxt
                    self.current_wp_idx = 0
                    self.publish_status(
                        f'Next: {nxt.name} '
                        f'(traffic={self.region_traffic(nxt):.2f}, '
                        f'dist={self.region_distance(nxt):.1f})')
                    self.publish_wp(force=True)
                else:
                    self.current_region = self.home_region
                    self.current_wp_idx = 0
                    self.publish_status('All cleaned — returning home')
                    self.publish_wp(force=True)
                    self.mission_complete = True
            else:
                self.publish_wp(force=True)
        else:
            self.publish_wp(force=False)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveScheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()