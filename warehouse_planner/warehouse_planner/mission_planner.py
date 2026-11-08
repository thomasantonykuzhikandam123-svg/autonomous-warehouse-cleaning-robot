#!/usr/bin/env python3
"""
Mission Planner — autonomous cleaning sequence.

Orchestrates the robot to clean all 3 aisles + corridor systematically.
- Maintains list of cleaning waypoints (aisle entry, top, bottom)
- Tracks which aisles are completed
- Checks if next target aisle is blocked by vehicles; skips if blocked >5s
- Returns to skipped aisles later when clear
- When all done, returns to home

Coverage logic (Galceran & Carreras 2013): aisle-by-aisle traversal.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time


class MissionPlanner(Node):

    def __init__(self):
        super().__init__('mission_planner')

        # Cleaning route waypoints (odom frame)
        # Each entry: (name, x, y, "aisle_id")
        # aisle entry → top → entry again forms an aisle cleaning sweep
        self.waypoints = [
            ('aisle_1_entry', -2.5, 1.5, 'aisle_1'),
            ('aisle_1_top',   -2.5, 12.0, 'aisle_1'),
            ('aisle_1_exit',  -2.5, 1.5, 'aisle_1'),

            ('aisle_2_entry',  1.5, 1.5, 'aisle_2'),
            ('aisle_2_top',    1.5, 12.0, 'aisle_2'),
            ('aisle_2_exit',   1.5, 1.5, 'aisle_2'),

            ('aisle_3_entry',  5.5, 1.5, 'aisle_3'),
            ('aisle_3_top',    5.5, 12.0, 'aisle_3'),
            ('aisle_3_exit',   5.5, 1.5, 'aisle_3'),

            ('home',           0.0, 0.0, 'home'),
        ]

        # State
        self.current_wp_index = 0
        self.completed_aisles = set()
        self.skipped_aisles = []   # list of aisle_ids to revisit
        self.mission_started = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.vehicles = []   # [(x, y), ...]
        self.last_goal_published_time = 0.0
        self.last_goal = None
        self.aisle_block_check_time = None   # when did we detect blockage

        # Parameters
        self.goal_tolerance = 0.5
        self.aisle_blocked_radius = 2.0   # vehicle within 2m of aisle blocks it
        self.aisle_block_timeout_s = 8.0  # if blocked this long, skip
        self.republish_period_s = 5.0     # republish goal periodically

        # ROS
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.veh_sub = self.create_subscription(
            PoseArray, '/dynamic_obstacles_array', self.vehicles_callback, 10)

        # Start mission after short delay (let other nodes start)
        self.create_timer(2.0, self.start_mission_once)
        self.create_timer(0.5, self.mission_tick)

        self.get_logger().info(
            f'Mission planner ready: {len(self.waypoints)} waypoints, '
            f'3 aisles to clean')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def vehicles_callback(self, msg):
        self.vehicles = [(p.position.x, p.position.y) for p in msg.poses]

    def start_mission_once(self):
        if self.mission_started:
            return
        self.mission_started = True
        self.publish_current_goal()
        self.get_logger().info('Mission started: cleaning aisles 1 → 2 → 3 → home')

    def publish_current_goal(self):
        if self.current_wp_index >= len(self.waypoints):
            return
        name, x, y, aisle = self.waypoints[self.current_wp_index]
        msg = PoseStamped()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.last_goal_published_time = time.time()
        self.last_goal = (x, y, name, aisle)

        status = String()
        status.data = f'Heading to: {name}'
        self.status_pub.publish(status)
        self.get_logger().info(f'NEW GOAL: {name} ({x:.1f}, {y:.1f})')

    def is_aisle_blocked(self, aisle_id):
        """Check if any vehicle is near the aisle entrance/path."""
        if aisle_id == 'home' or aisle_id is None:
            return False

        # Get the aisle's X coordinate
        aisle_x = None
        for wp in self.waypoints:
            if wp[3] == aisle_id and 'entry' in wp[0]:
                aisle_x = wp[1]
                break
        if aisle_x is None:
            return False

        # Check if any vehicle is within blocking range of this aisle
        for vx, vy in self.vehicles:
            # If vehicle is near this aisle's X column AND in aisle Y range
            if abs(vx - aisle_x) < self.aisle_blocked_radius:
                if vy > 0.5:   # vehicle inside or at entrance of aisle
                    return True
        return False

    def is_at_current_goal(self):
        if self.current_wp_index >= len(self.waypoints):
            return False
        name, x, y, _ = self.waypoints[self.current_wp_index]
        return math.hypot(self.robot_x - x, self.robot_y - y) < self.goal_tolerance

    def advance_waypoint(self):
        """Move to next waypoint, considering skipped aisles."""
        # If we just finished an aisle's exit, mark it complete
        if self.last_goal is not None:
            name, _, _, aisle = self.last_goal[2], None, None, self.last_goal[3]
            if 'exit' in name:
                self.completed_aisles.add(aisle)
                self.get_logger().info(f'✓ Cleaned: {aisle}')

        self.current_wp_index += 1

        # Skip waypoints belonging to already-completed aisles (revisit case)
        while self.current_wp_index < len(self.waypoints):
            wp_aisle = self.waypoints[self.current_wp_index][3]
            if wp_aisle in self.completed_aisles and wp_aisle != 'home':
                self.current_wp_index += 1
            else:
                break

        # If mission complete, check if any skipped aisles remain
        if self.current_wp_index >= len(self.waypoints):
            if self.skipped_aisles:
                self.get_logger().info(f'Returning to skipped aisles: {self.skipped_aisles}')
                # Find first waypoint for skipped aisle
                skipped = self.skipped_aisles.pop(0)
                for i, wp in enumerate(self.waypoints):
                    if wp[3] == skipped and 'entry' in wp[0]:
                        self.current_wp_index = i
                        return
            else:
                status = String()
                status.data = 'MISSION COMPLETE — all aisles cleaned'
                self.status_pub.publish(status)
                self.get_logger().info('🎉 MISSION COMPLETE — all aisles cleaned')

    def mission_tick(self):
        if not self.mission_started:
            return
        if self.current_wp_index >= len(self.waypoints):
            return

        name, x, y, aisle = self.waypoints[self.current_wp_index]

        # If at goal, advance
        if self.is_at_current_goal():
            self.advance_waypoint()
            if self.current_wp_index < len(self.waypoints):
                self.publish_current_goal()
            return

        # If targeting an aisle's entry, check if blocked
        if 'entry' in name and aisle not in self.completed_aisles:
            if self.is_aisle_blocked(aisle):
                if self.aisle_block_check_time is None:
                    self.aisle_block_check_time = time.time()
                    self.get_logger().info(f'{aisle} blocked by vehicle, waiting...')
                elif time.time() - self.aisle_block_check_time > self.aisle_block_timeout_s:
                    # Skip to next aisle
                    self.get_logger().info(f'{aisle} blocked too long — skipping')
                    if aisle not in self.skipped_aisles:
                        self.skipped_aisles.append(aisle)
                    # Jump to next aisle's entry
                    self.skip_current_aisle()
                    self.aisle_block_check_time = None
                    self.publish_current_goal()
                    return
            else:
                self.aisle_block_check_time = None

        # Republish goal periodically (in case A* missed it)
        if time.time() - self.last_goal_published_time > self.republish_period_s:
            self.publish_current_goal()

    def skip_current_aisle(self):
        """Advance index past all waypoints of current aisle."""
        current_aisle = self.waypoints[self.current_wp_index][3]
        while (self.current_wp_index < len(self.waypoints)
               and self.waypoints[self.current_wp_index][3] == current_aisle):
            self.current_wp_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()