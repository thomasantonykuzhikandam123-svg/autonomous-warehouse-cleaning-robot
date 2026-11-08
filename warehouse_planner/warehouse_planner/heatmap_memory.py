#!/usr/bin/env python3
"""
Spatiotemporal heat-map memory for dynamic obstacle observation.

Maintains a persistent 2D grid where each cell stores a probability that
dynamic obstacles have been seen there recently. Updated from lidar
detections. Decays exponentially over time so recent observations
weight more than old ones.

This is the robot's "experience" — it learns where vehicles tend to
appear and uses this knowledge in the adaptive scheduler.

Reference: log-odds occupancy mapping (Thrun, Burgard, Fox 2005)
extended to dynamic agents with temporal decay.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
import math
import time


class HeatmapMemory(Node):

    def __init__(self):
        super().__init__('heatmap_memory')

        # Grid configuration (matches warehouse map)
        self.resolution = 0.4   # 0.4m cells = 37x50 grid = manageable size
        self.width_m = 15.0
        self.height_m = 20.0
        self.spawn_x = 4.0      # world spawn offset
        self.spawn_y = 4.0
        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)

        # Heat map data: float per cell, 0.0 (cold) to 1.0 (very recently observed)
        self.heatmap = [[0.0] * self.width_cells for _ in range(self.height_cells)]

        # Parameters
        self.observation_increment = 0.4   # how much an observation raises probability
        self.decay_half_life_s = 30.0      # probability halves every 30s
        self.influence_radius = 2          # cells around observation also raised

        self.last_decay_time = time.time()

        # ROS
        self.obs_sub = self.create_subscription(
            PoseArray, '/detected_obstacles', self.obstacle_callback, 10)
        self.heatmap_pub = self.create_publisher(OccupancyGrid, '/heatmap', 10)
        self.values_pub = self.create_publisher(Float32MultiArray, '/heatmap_values', 10)

        # Decay and publish at 2 Hz
        self.create_timer(0.5, self.decay_and_publish)

        self.get_logger().info(
            f'Heat-map memory ready. Grid {self.width_cells}x{self.height_cells} '
            f'(0.4m cells). Half-life {self.decay_half_life_s}s')

    def world_to_grid(self, x_odom, y_odom):
        """Convert odom-frame coordinate to heatmap grid cell."""
        x_world = x_odom + self.spawn_x
        y_world = y_odom + self.spawn_y
        gx = int(x_world / self.resolution)
        gy = int(y_world / self.resolution)
        return gx, gy

    def obstacle_callback(self, msg):
        """When obstacles are detected, raise heatmap probability at their cells."""
        for pose in msg.poses:
            gx, gy = self.world_to_grid(pose.position.x, pose.position.y)
            r = self.influence_radius
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy > r * r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width_cells and 0 <= ny < self.height_cells:
                        # Falloff: closer to center = more increment
                        falloff = 1.0 - math.sqrt(dx * dx + dy * dy) / (r + 1)
                        self.heatmap[ny][nx] = min(
                            1.0,
                            self.heatmap[ny][nx] + self.observation_increment * falloff
                        )

    def decay_and_publish(self):
        """Decay heatmap values over time, then publish."""
        now = time.time()
        dt = now - self.last_decay_time
        self.last_decay_time = now

        # Exponential decay: value *= 2^(-dt / half_life)
        if dt > 0:
            decay_factor = math.pow(2.0, -dt / self.decay_half_life_s)
            for gy in range(self.height_cells):
                for gx in range(self.width_cells):
                    self.heatmap[gy][gx] *= decay_factor

        self.publish_heatmap()

    def publish_heatmap(self):
        """Publish as OccupancyGrid (0-100) for RViz, and as raw values for planner."""
        # OccupancyGrid version for RViz visualization
        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = self.width_cells
        msg.info.height = self.height_cells
        msg.info.origin.position.x = -self.spawn_x
        msg.info.origin.position.y = -self.spawn_y
        msg.info.origin.orientation.w = 1.0
        data = []
        for gy in range(self.height_cells):
            for gx in range(self.width_cells):
                val = int(self.heatmap[gy][gx] * 100)
                data.append(max(0, min(100, val)))
        msg.data = data
        self.heatmap_pub.publish(msg)

        # Raw float array for scheduler
        values_msg = Float32MultiArray()
        for gy in range(self.height_cells):
            for gx in range(self.width_cells):
                values_msg.data.append(float(self.heatmap[gy][gx]))
        self.values_pub.publish(values_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeatmapMemory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()