#!/usr/bin/env python3
"""
Executive Controller — self-contained, wall-aware cleaning robot.

Builds its own static warehouse map internally — no dependency on
external map publisher. Uses ray-casting in 3 directions (left, center,
right) for forward obstacle detection. Combines this with lidar-detected
dynamic obstacles for a comprehensive safety check.

Behavior:
  IDLE      — no waypoint
  ALIGNING  — heading off by >40°, rotating in place
  DRIVING   — clear ahead, full speed
  SLOWING   — obstacle/wall within 1.8m forward
  PAUSED    — obstacle/wall within 0.9m forward
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
import math
import time


class ExecutiveController(Node):

    def __init__(self):
        super().__init__('executive_controller')

        # --------- Static map (built internally) ---------
        self.resolution = 0.2
        self.width_m = 15.0
        self.height_m = 20.0
        self.spawn_x = 4.0
        self.spawn_y = 4.0
        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)
        self.static_grid_clean = None
        self.static_grid_inflated = self.build_static_map()

        # --------- Kinematic ---------
        self.max_v = 0.28
        self.min_v = 0.05
        self.max_omega = 1.0
        self.heading_kp = 1.8

        # --------- Forward checking ---------
        self.forward_check_distance = 1.8
        self.forward_check_step = 0.1
        self.lateral_offsets = [-0.20, 0.0, 0.20]   # left/center/right rays

        # --------- Stop / slow distances ---------
        self.slow_distance = 1.5
        self.stop_distance = 0.7

        # --------- Side safety ---------
        self.side_check_radius = 0.45
        self.side_safety_speed_cap = 0.15

        # --------- Heading alignment ---------
        self.realign_threshold = math.radians(35)

        # --------- Robot footprint ---------
        self.robot_radius = 0.30

        # --------- Goal ---------
        self.waypoint_tolerance = 0.40

        # --------- State ---------
        self.state = 'IDLE'
        self.current_waypoint = None
        self.pause_started_at = None

        # --------- Robot odom ---------
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.have_odom = False

        # --------- Dynamic obstacles ---------
        self.detected_obstacles = []

        # --------- ROS ---------
        self.waypoint_sub = self.create_subscription(
            PoseStamped, '/next_waypoint', self.waypoint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.obs_sub = self.create_subscription(
            PoseArray, '/detected_obstacles', self.obstacle_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/controller_state', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.5, self.publish_state)
        self.create_timer(1.0, self.publish_map)

        self.get_logger().info('Executive Controller ready (self-contained map)')

    # =====================================================
    # STATIC MAP BUILDING
    # =====================================================
    def add_wall_to(self, grid, xc, yc, lx, ly):
        gx_min = int((xc - lx / 2) / self.resolution)
        gy_min = int((yc - ly / 2) / self.resolution)
        gx_max = int((xc + lx / 2) / self.resolution)
        gy_max = int((yc + ly / 2) / self.resolution)
        for gy in range(max(0, gy_min), min(self.height_cells, gy_max + 1)):
            for gx in range(max(0, gx_min), min(self.width_cells, gx_max + 1)):
                grid[gy][gx] = 1

    def build_static_map(self):
        clean = [[0] * self.width_cells for _ in range(self.height_cells)]

        # Outer walls
        self.add_wall_to(clean, 7.5, 20.0, 15.0, 0.2)
        self.add_wall_to(clean, 7.5, 0.0, 15.0, 0.2)
        self.add_wall_to(clean, 0.0, 10.0, 0.2, 20.0)
        self.add_wall_to(clean, 15.0, 10.0, 0.2, 20.0)

        # Ground zones
        self.add_wall_to(clean, 2.5, 1.5, 3, 2)
        self.add_wall_to(clean, 5.0, 1.5, 1, 2)
        self.add_wall_to(clean, 12.0, 1.5, 4, 2)

        # Storage racks
        self.add_wall_to(clean, 1.5, 11.5, 1, 12)
        self.add_wall_to(clean, 5.5, 11.5, 1, 12)
        self.add_wall_to(clean, 9.5, 11.5, 1, 12)
        self.add_wall_to(clean, 13.5, 11.5, 1, 12)

        # Side pallets
        self.add_wall_to(clean, 0.6, 9.0, 0.8, 1.2)
        self.add_wall_to(clean, 14.4, 9.0, 0.8, 1.2)

        self.static_grid_clean = [row[:] for row in clean]
        # Inflate by 2 cells = 0.4m for safety clearance
        return self.inflate(clean, radius_cells=2)

    def inflate(self, grid, radius_cells=2):
        h = len(grid); w = len(grid[0])
        out = [[0] * w for _ in range(h)]
        for gy in range(h):
            for gx in range(w):
                if grid[gy][gx] == 1:
                    for dy in range(-radius_cells, radius_cells + 1):
                        for dx in range(-radius_cells, radius_cells + 1):
                            if dx * dx + dy * dy <= radius_cells * radius_cells:
                                ny, nx = gy + dy, gx + dx
                                if 0 <= ny < h and 0 <= nx < w:
                                    out[ny][nx] = 1
        return out

    # =====================================================
    # MAP UTILITIES (use INFLATED grid)
    # =====================================================
    def is_wall_at(self, x_odom, y_odom):
        x_world = x_odom + self.spawn_x
        y_world = y_odom + self.spawn_y
        gx = int(x_world / self.resolution)
        gy = int(y_world / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.width_cells or gy >= self.height_cells:
            return True
        return self.static_grid_inflated[gy][gx] == 1

    def forward_obstacle_distance(self):
        """Multi-ray forward check. Returns minimum distance across rays."""
        cos_t = math.cos(self.robot_theta)
        sin_t = math.sin(self.robot_theta)
        min_d = float('inf')
        for lat in self.lateral_offsets:
            # Each ray starts at robot + lateral offset, goes forward
            origin_x = self.robot_x + lat * (-sin_t)
            origin_y = self.robot_y + lat * cos_t
            d = self.forward_check_step
            while d < self.forward_check_distance:
                tx = origin_x + d * cos_t
                ty = origin_y + d * sin_t
                if self.is_wall_at(tx, ty):
                    if d < min_d:
                        min_d = d
                    break
                # Check dynamic obstacles too
                for ox, oy in self.detected_obstacles:
                    if math.hypot(tx - ox, ty - oy) < 0.4:
                        if d < min_d:
                            min_d = d
                        break
                else:
                    d += self.forward_check_step
                    continue
                break
        return min_d

    def nearest_wall_omnidirectional(self):
        """Minimum distance to a wall in any direction within side_check_radius."""
        min_d = float('inf')
        for ang_deg in range(0, 360, 20):
            ang = math.radians(ang_deg)
            d = 0.1
            while d < self.side_check_radius:
                tx = self.robot_x + d * math.cos(ang)
                ty = self.robot_y + d * math.sin(ang)
                if self.is_wall_at(tx, ty):
                    if d < min_d:
                        min_d = d
                    break
                d += 0.1
        return min_d

    # =====================================================
    # CALLBACKS
    # =====================================================
    def waypoint_callback(self, msg):
        new_wp = (msg.pose.position.x, msg.pose.position.y)
        if self.current_waypoint is None or \
                math.hypot(new_wp[0] - self.current_waypoint[0],
                           new_wp[1] - self.current_waypoint[1]) > 0.1:
            self.current_waypoint = new_wp
            self.pause_started_at = None
            self.get_logger().info(
                f'New waypoint: ({new_wp[0]:.2f}, {new_wp[1]:.2f})')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)
        self.have_odom = True

    def obstacle_callback(self, msg):
        self.detected_obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def publish_map(self):
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
                data.append(100 if self.static_grid_clean[gy][gx] == 1 else 0)
        msg.data = data
        self.map_pub.publish(msg)

    # =====================================================
    # MAIN CONTROL LOOP
    # =====================================================
    def control_loop(self):
        if not self.have_odom or self.current_waypoint is None:
            self.cmd_pub.publish(Twist())
            self.state = 'IDLE'
            return

        # SAFETY FIRST: if robot is RIGHT next to a wall, emergency stop
        nearest_wall = self.nearest_wall_omnidirectional()
        if nearest_wall < 0.15:
            self.cmd_pub.publish(Twist())
            self.state = 'EMERGENCY_STOP'
            self.get_logger().error(
                f'EMERGENCY: wall {nearest_wall:.2f}m away!',
                throttle_duration_sec=2.0)
            return

        wx, wy = self.current_waypoint
        dx = wx - self.robot_x
        dy = wy - self.robot_y
        dist_to_wp = math.hypot(dx, dy)

        # Arrived
        if dist_to_wp < self.waypoint_tolerance:
            self.cmd_pub.publish(Twist())
            self.state = 'IDLE'
            return

        # Compute heading error
        target_heading = math.atan2(dy, dx)
        heading_err = self.normalize_angle(target_heading - self.robot_theta)

        # ALIGNING — if heading off, rotate first (any time, not just on new wp)
        if abs(heading_err) > self.realign_threshold:
            self.state = 'ALIGNING'
            cmd = Twist()
            cmd.angular.z = math.copysign(
                min(self.max_omega, abs(heading_err) * 1.5),
                heading_err)
            self.cmd_pub.publish(cmd)
            return

        # Check forward
        fwd_dist = self.forward_obstacle_distance()

        # PAUSED — something very close
        if fwd_dist < self.stop_distance:
            if self.state != 'PAUSED':
                self.state = 'PAUSED'
                self.pause_started_at = time.time()
                self.get_logger().info(
                    f'PAUSED — obstacle at {fwd_dist:.2f}m',
                    throttle_duration_sec=2.0)
            self.cmd_pub.publish(Twist())
            return

        # SLOWING
        if fwd_dist < self.slow_distance:
            self.state = 'SLOWING'
            self.pause_started_at = None
            fraction = ((fwd_dist - self.stop_distance) /
                        (self.slow_distance - self.stop_distance))
            v = self.min_v + fraction * (self.max_v - self.min_v)
        else:
            self.state = 'DRIVING'
            self.pause_started_at = None
            v = self.max_v

        # Side-safety speed cap
        if nearest_wall < 0.5:
            v = min(v, self.side_safety_speed_cap)

        # Heading correction (proportional)
        omega = self.heading_kp * heading_err
        omega = max(-self.max_omega, min(self.max_omega, omega))

        # Slow down on turns
        if abs(heading_err) > 0.2:
            v *= max(0.3, 1.0 - abs(heading_err) / (math.pi / 2))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ExecutiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()