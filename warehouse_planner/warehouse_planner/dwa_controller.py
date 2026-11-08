#!/usr/bin/env python3
"""
Hybrid local planner:
- Pure Pursuit (Coulter 1992) for path following
- Velocity Obstacle (Van den Berg 2008) for predictive avoidance
- APF with annealing (Khatib 1986 + Geng 2023) for deadlock escape

Reads obstacles from /detected_obstacles (from lidar) — real perception.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import Float32MultiArray
import math
import random


class PathFollower(Node):

    def __init__(self):
        super().__init__('dwa_controller')

        # Kinematic limits
        self.max_v = 0.4
        self.min_v = 0.05
        self.max_omega = 1.5
        self.robot_radius = 0.22

        # Pure Pursuit
        self.lookahead_min = 0.5
        self.lookahead_max = 1.2
        self.lookahead_gain = 1.0
        self.curvature_threshold = 0.6
        self.proximity_threshold = 0.5
        self.goal_tolerance = 0.4

        # Rotate-to-heading prelude
        self.heading_align_threshold = math.radians(35)
        self.heading_align_omega = 0.8

        # Dynamic obstacles
        self.obstacle_radius = 0.35
        self.safety_margin = 0.30
        self.detection_radius = 3.5
        self.velocity_obstacle_horizon = 2.0
        self.vo_dt = 0.1

        # APF annealing
        self.annealing_temperature = 0.0
        self.annealing_cool_rate = 0.05
        self.deadlock_counter = 0

        # State
        self.current_path = []
        self.path_index = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.current_v = 0.0
        self.have_odom = False
        self.arrived = True
        self.aligning_heading = False

        # Detected obstacles (from lidar)
        self.detected_obstacles = []   # list of (x, y)
        self.detected_velocities = []  # list of (vx, vy)

        # Map
        self.map_data = None
        self.map_resolution = 0.2
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        # ROS
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.obs_sub = self.create_subscription(
            PoseArray, '/detected_obstacles', self.obstacle_callback, 10)
        self.vel_sub = self.create_subscription(
            Float32MultiArray, '/detected_obstacle_velocities', self.velocity_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Lidar-perception based controller ready')

    def path_callback(self, msg):
        self.current_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_index = 0
        if self.current_path:
            self.arrived = False
            self.aligning_heading = True

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)
        self.current_v = msg.twist.twist.linear.x
        self.have_odom = True

    def map_callback(self, msg):
        self.map_data = list(msg.data)
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height

    def obstacle_callback(self, msg):
        self.detected_obstacles = [(p.position.x, p.position.y) for p in msg.poses]

    def velocity_callback(self, msg):
        # Flat array [vx1, vy1, vx2, vy2, ...]
        data = msg.data
        self.detected_velocities = [(data[i], data[i + 1]) for i in range(0, len(data), 2)]

    def nearby_obstacles(self):
        """Return obstacles within detection_radius, with their velocities."""
        result = []
        for i, (x, y) in enumerate(self.detected_obstacles):
            d = math.hypot(x - self.robot_x, y - self.robot_y)
            if d < self.detection_radius:
                if i < len(self.detected_velocities):
                    vx, vy = self.detected_velocities[i]
                else:
                    vx, vy = 0.0, 0.0
                result.append((x, y, vx, vy, d))
        return result

    def is_occupied(self, x, y):
        if self.map_data is None:
            return False
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        if gx < 0 or gy < 0 or gx >= self.map_width or gy >= self.map_height:
            return True
        return self.map_data[gy * self.map_width + gx] > 50

    def footprint_collides(self, x, y):
        r = self.robot_radius
        for dx, dy in [(0, 0), (r, 0), (-r, 0), (0, r), (0, -r),
                        (r * 0.7, r * 0.7), (-r * 0.7, r * 0.7),
                        (r * 0.7, -r * 0.7), (-r * 0.7, -r * 0.7)]:
            if self.is_occupied(x + dx, y + dy):
                return True
        return False

    def nearest_wall(self, x, y, max_d=0.8):
        n = int(max_d / self.map_resolution)
        for ring in range(1, n + 1):
            d = ring * self.map_resolution
            for ang_deg in range(0, 360, 45):
                ang = math.radians(ang_deg)
                if self.is_occupied(x + d * math.cos(ang), y + d * math.sin(ang)):
                    return d
        return max_d

    def trajectory_hits_wall(self, v, omega, horizon):
        if v < 0.01:
            return False
        x, y, theta = self.robot_x, self.robot_y, self.robot_theta
        dt = 0.1
        steps = max(1, int(horizon / dt))
        for _ in range(steps):
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt
            if self.footprint_collides(x, y):
                return True
        return False

    def trajectory_clearance_vo(self, v, omega, obstacles, horizon):
        """Velocity Obstacle check — simulate both robot AND obstacles."""
        if not obstacles:
            return float('inf')
        x, y, theta = self.robot_x, self.robot_y, self.robot_theta
        dt = self.vo_dt
        steps = max(1, int(horizon / dt))
        threshold = self.robot_radius + self.obstacle_radius + self.safety_margin
        min_clear = float('inf')
        for step in range(steps):
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt
            t = (step + 1) * dt
            for ox, oy, ovx, ovy, _ in obstacles:
                fx = ox + ovx * t
                fy = oy + ovy * t
                d = math.hypot(x - fx, y - fy)
                if d < threshold:
                    return 0.0
                if d < min_clear:
                    min_clear = d
        return min_clear

    def dwa_with_vo(self, v_base, omega_base, lookahead, obstacles):
        v_samples = [self.max_v, self.max_v * 0.7, self.max_v * 0.4,
                     self.max_v * 0.2, self.min_v]
        omega_offsets = [-1.4, -1.0, -0.7, -0.4, -0.2, 0.0,
                          0.2, 0.4, 0.7, 1.0, 1.4]
        omega_samples = [max(-self.max_omega, min(self.max_omega, omega_base + o))
                          for o in omega_offsets]
        candidates = []

        for v in v_samples:
            v = max(self.min_v if v > 0 else 0.0, v)
            for omega in omega_samples:
                if self.trajectory_hits_wall(v, omega, 1.2):
                    continue
                clearance = self.trajectory_clearance_vo(v, omega, obstacles,
                                                         self.velocity_obstacle_horizon)
                if clearance < 0.001:
                    continue
                candidates.append((v, omega, clearance))

        if not candidates:
            return self.apf_escape(v_base, omega_base, obstacles)

        best_score = -float('inf')
        best_v = self.min_v
        best_omega = 0.0
        for v, omega, clearance in candidates:
            x = self.robot_x + v * math.cos(self.robot_theta) * 1.5
            y = self.robot_y + v * math.sin(self.robot_theta) * 1.5
            goal_dist = math.hypot(lookahead[0] - x, lookahead[1] - y)
            goal_score = 1.0 / (1.0 + goal_dist)
            clearance_score = min(clearance, 3.0) / 3.0
            v_dev = abs(v - v_base) / max(self.max_v, 0.1)
            omega_dev = abs(omega - omega_base) / self.max_omega
            smoothness = max(0.0, 1.0 - (v_dev + omega_dev) / 2.0)
            speed = v / self.max_v
            score = 2.0 * goal_score + 2.0 * clearance_score + 0.3 * smoothness + 0.5 * speed
            if score > best_score:
                best_score = score
                best_v = v
                best_omega = omega

        self.annealing_temperature = max(0.0, self.annealing_temperature - self.annealing_cool_rate)
        self.deadlock_counter = 0
        return best_v, best_omega

    def apf_escape(self, v_base, omega_base, obstacles):
        self.deadlock_counter += 1
        self.annealing_temperature = min(1.0, self.annealing_temperature + 0.15)
        repulse_x = 0.0
        repulse_y = 0.0
        for ox, oy, _, _, d in obstacles:
            if d > 0.01 and d < 2.5:
                force = (1.0 / d - 1.0 / 2.5) ** 2
                rx = (self.robot_x - ox) / d
                ry = (self.robot_y - oy) / d
                repulse_x += force * rx
                repulse_y += force * ry
        for ang_deg in range(0, 360, 30):
            ang = math.radians(ang_deg)
            for d_test in [0.3, 0.6, 0.9]:
                tx = self.robot_x + d_test * math.cos(ang)
                ty = self.robot_y + d_test * math.sin(ang)
                if self.is_occupied(tx, ty):
                    force = (1.0 / d_test - 1.0 / 1.2) ** 2
                    repulse_x -= force * math.cos(ang)
                    repulse_y -= force * math.sin(ang)
                    break
        repulse_angle = math.atan2(repulse_y, repulse_x)
        perturbation = random.uniform(-math.pi / 2, math.pi / 2) * self.annealing_temperature
        desired = self.normalize_angle(repulse_angle - self.robot_theta + perturbation)
        omega = max(-self.max_omega, min(self.max_omega, desired * 2.0))
        v = self.min_v * (1.0 + self.annealing_temperature)
        if self.trajectory_hits_wall(v, omega, 0.5):
            v = 0.0
        self.get_logger().warn(
            f'APF escape (T={self.annealing_temperature:.2f})',
            throttle_duration_sec=1.0)
        return v, omega

    def control_loop(self):
        if not self.have_odom or not self.current_path or self.arrived:
            return
        if self.map_data is None:
            return

        goal_x, goal_y = self.current_path[-1]
        if math.hypot(goal_x - self.robot_x, goal_y - self.robot_y) < self.goal_tolerance:
            self.stop_robot()
            self.arrived = True
            self.aligning_heading = False
            return

        if self.aligning_heading:
            target = self.find_initial_target()
            if target is None:
                self.aligning_heading = False
                return
            dx = target[0] - self.robot_x
            dy = target[1] - self.robot_y
            err = self.normalize_angle(math.atan2(dy, dx) - self.robot_theta)
            if abs(err) < self.heading_align_threshold:
                self.aligning_heading = False
            else:
                cmd = Twist()
                cmd.angular.z = math.copysign(self.heading_align_omega, err)
                self.cmd_pub.publish(cmd)
                return

        best_i = self.path_index
        best_d = float('inf')
        for i in range(self.path_index, min(self.path_index + 25, len(self.current_path))):
            d = math.hypot(self.current_path[i][0] - self.robot_x,
                            self.current_path[i][1] - self.robot_y)
            if d < best_d:
                best_d = d
                best_i = i
        self.path_index = best_i

        L = max(self.lookahead_min,
                min(self.lookahead_max,
                    self.lookahead_gain * max(self.current_v, 0.2)))
        lookahead = None
        for i in range(self.path_index, len(self.current_path)):
            d = math.hypot(self.current_path[i][0] - self.robot_x,
                            self.current_path[i][1] - self.robot_y)
            if d >= L:
                lookahead = self.current_path[i]
                break
        if lookahead is None:
            lookahead = self.current_path[-1]

        dx = lookahead[0] - self.robot_x
        dy = lookahead[1] - self.robot_y
        alpha = self.normalize_angle(math.atan2(dy, dx) - self.robot_theta)
        L_actual = math.hypot(dx, dy)
        curvature = 2.0 * math.sin(alpha) / L_actual if L_actual > 0.01 else 0.0

        v = self.max_v
        if abs(curvature) > self.curvature_threshold:
            v *= self.curvature_threshold / abs(curvature)
        near = self.nearest_wall(self.robot_x, self.robot_y)
        if near < self.proximity_threshold:
            v *= near / self.proximity_threshold
        v = max(self.min_v, min(self.max_v, v))
        omega = curvature * v
        omega = max(-self.max_omega, min(self.max_omega, omega))

        v_baseline = v
        omega_baseline = omega

        obstacles = self.nearby_obstacles()

        if obstacles:
            v, omega = self.dwa_with_vo(v_baseline, omega_baseline, lookahead, obstacles)
            self.get_logger().info(
                f'VO avoidance: {len(obstacles)} obstacle(s). v={v:.2f}, ω={omega:.2f}',
                throttle_duration_sec=1.0)
        else:
            horizon = min(0.5, L / max(v, 0.2))
            if self.trajectory_hits_wall(v, omega, horizon):
                v = self.min_v
                omega = curvature * v
                omega = max(-self.max_omega, min(self.max_omega, omega))
                if self.trajectory_hits_wall(v, omega, horizon):
                    v = 0.0
                    omega = 0.0
            self.annealing_temperature = max(0.0,
                                              self.annealing_temperature - self.annealing_cool_rate)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    def find_initial_target(self):
        if not self.current_path:
            return None
        for pt in self.current_path:
            if math.hypot(pt[0] - self.robot_x, pt[1] - self.robot_y) >= 0.8:
                return pt
        return self.current_path[-1]

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathFollower())
    rclpy.shutdown()


if __name__ == '__main__':
    main()