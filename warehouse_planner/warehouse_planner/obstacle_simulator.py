#!/usr/bin/env python3
"""
Dynamic obstacle simulator with yielding behavior.

The obstacle moves slowly across the corridor near aisle 2 entrance.
When the robot is within 1.2m, the obstacle pauses (yields).
This simulates real warehouse forklift/person behavior.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import time


class ObstacleSimulator(Node):

    def __init__(self):
        super().__init__('obstacle_simulator')

        # Obstacle parameters (odom frame)
        # Corridor middle in odom: Y=0 (since world corridor Y=4, spawn Y=4)
        # X=4 in odom = world X=8 = aisle 2 entrance area
        self.obstacle_x = 4.0
        self.y_min = -1.0
        self.y_max = 1.0
        self.period_seconds = 20.0
        self.obstacle_size = 0.4

        # Yielding parameters
        self.yield_distance = 1.2     # pause if robot within this
        self.resume_distance = 1.6    # resume only when robot beyond this

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.is_paused = False
        self.paused_y = 0.0
        self.paused_time_accumulated = 0.0
        self.start_time = time.time()
        self.last_update_time = time.time()

        # ROS
        self.obstacle_pub = self.create_publisher(PoseStamped, '/dynamic_obstacles', 10)
        self.marker_pub = self.create_publisher(Marker, '/obstacle_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.05, self.update_obstacle)

        self.get_logger().info(
            f'Smart obstacle: X={self.obstacle_x} (corridor near aisle 2), '
            f'Y oscillates {self.y_min} to {self.y_max}, yields at {self.yield_distance}m')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def compute_obstacle_y(self, effective_time):
        """Triangle wave based on effective (non-paused) time."""
        phase = (effective_time % self.period_seconds) / self.period_seconds
        if phase < 0.5:
            y_norm = phase * 2.0
        else:
            y_norm = (1.0 - phase) * 2.0
        return self.y_min + y_norm * (self.y_max - self.y_min)

    def update_obstacle(self):
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now

        # Determine obstacle position
        if self.is_paused:
            y_obs = self.paused_y
        else:
            effective_time = now - self.start_time - self.paused_time_accumulated
            y_obs = self.compute_obstacle_y(effective_time)

        # Check distance to robot
        dist_to_robot = math.hypot(self.obstacle_x - self.robot_x,
                                   y_obs - self.robot_y)

        # State transition
        if self.is_paused:
            if dist_to_robot > self.resume_distance:
                self.is_paused = False
                self.get_logger().info('Robot clear — resuming')
            else:
                self.paused_time_accumulated += dt
        else:
            if dist_to_robot < self.yield_distance:
                self.is_paused = True
                self.paused_y = y_obs
                self.get_logger().info(f'Robot near ({dist_to_robot:.2f}m) — pausing')

        # Publish obstacle position
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'odom'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.obstacle_x
        pose_msg.pose.position.y = y_obs
        pose_msg.pose.orientation.w = 1.0
        self.obstacle_pub.publish(pose_msg)

        # Publish RViz marker
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dynamic_obstacle'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.obstacle_x
        marker.pose.position.y = y_obs
        marker.pose.position.z = self.obstacle_size / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.obstacle_size
        marker.scale.y = self.obstacle_size
        marker.scale.z = self.obstacle_size
        # Yellow when paused, red when moving (visual feedback)
        if self.is_paused:
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.0
        else:
            marker.color.r = 0.9
            marker.color.g = 0.2
            marker.color.b = 0.2
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()