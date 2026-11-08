#!/usr/bin/env python3
"""
Mission Brain — STAGE 1 (clean base).

Environment: NO loose obstacles (vehicles removed from world).
Behaviour: clean all aisles by shortest path, return home.
Map: self-contained static map (racks + walls) for wall safety.

Cleaning pattern: straight up the aisle centre, straight back. Two passes.
No lateral shifts near walls (that was causing wall-clipping).

States: GO_TO_HUB -> AT_HUB -> CLEANING -> (repeat) -> RETURNING_HOME -> COMPLETE
Motion: rotate-in-place to align, then drive dead straight. Wall reflex on.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
import math


class Aisle:
    def __init__(self, name, center_x):
        self.name = name
        self.center_x = center_x
        self.cleaned = False


class MissionBrain(Node):

    def __init__(self):
        super().__init__('mission_brain')

        # ---- static map ----
        self.resolution = 0.2
        self.spawn_x = 4.0
        self.spawn_y = 4.0
        self.width_cells = int(15.0 / self.resolution)
        self.height_cells = int(20.0 / self.resolution)
        self.grid_clean = None
        self.grid_inflated = self.build_static_map()

        # ---- geometry (odom; spawn at world 4,4) ----
        self.robot_lane_y = -0.5
        self.aisle_entrance_y = 1.5
        self.aisle_back_y = 10.0
        self.hub = (3.5, -0.5)
        self.home = (0.0, 0.0)

        self.aisles = [
            Aisle('aisle_1', -0.5),
            Aisle('aisle_2', 3.5),
            Aisle('aisle_3', 7.5),
        ]

        # ---- motion ----
        self.max_v = 0.25
        self.min_v = 0.06
        self.max_omega = 1.0
        self.align_threshold = math.radians(12)
        self.waypoint_tolerance = 0.25
        self.approach_slowdown = 0.8

        # ---- wall safety ----
        self.wall_stop_dist = 0.35
        self.wall_check_dist = 0.6

        # ---- state ----
        self.state = 'INIT'
        self.committed_aisle = None
        self.clean_waypoints = []
        self.clean_index = 0

        # ---- odom ----
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.have_odom = False

        # ---- ROS ----
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission_status', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info('========================================')
        self.get_logger().info('  MISSION BRAIN STAGE 1 — ALIVE & READY')
        self.get_logger().info('  Waiting for /odom, then starting...')
        self.get_logger().info('========================================')

    # ---------------- MAP ----------------
    def add_wall(self, grid, xc, yc, lx, ly):
        gx0 = int((xc - lx / 2) / self.resolution)
        gy0 = int((yc - ly / 2) / self.resolution)
        gx1 = int((xc + lx / 2) / self.resolution)
        gy1 = int((yc + ly / 2) / self.resolution)
        for gy in range(max(0, gy0), min(self.height_cells, gy1 + 1)):
            for gx in range(max(0, gx0), min(self.width_cells, gx1 + 1)):
                grid[gy][gx] = 1

    def build_static_map(self):
        g = [[0] * self.width_cells for _ in range(self.height_cells)]
        self.add_wall(g, 7.5, 20.0, 15.0, 0.2)
        self.add_wall(g, 7.5, 0.0, 15.0, 0.2)
        self.add_wall(g, 0.0, 10.0, 0.2, 20.0)
        self.add_wall(g, 15.0, 10.0, 0.2, 20.0)
        self.add_wall(g, 2.5, 1.5, 3, 2)
        self.add_wall(g, 5.0, 1.5, 1, 2)
        self.add_wall(g, 12.0, 1.5, 4, 2)
        self.add_wall(g, 1.5, 11.5, 1, 12)
        self.add_wall(g, 5.5, 11.5, 1, 12)
        self.add_wall(g, 9.5, 11.5, 1, 12)
        self.add_wall(g, 13.5, 11.5, 1, 12)
        self.add_wall(g, 0.6, 9.0, 0.8, 1.2)
        self.add_wall(g, 14.4, 9.0, 0.8, 1.2)
        self.grid_clean = [row[:] for row in g]
        out = [[0] * self.width_cells for _ in range(self.height_cells)]
        for gy in range(self.height_cells):
            for gx in range(self.width_cells):
                if g[gy][gx] == 1:
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                            if dx * dx + dy * dy <= 4:
                                ny, nx = gy + dy, gx + dx
                                if 0 <= ny < self.height_cells and 0 <= nx < self.width_cells:
                                    out[ny][nx] = 1
        return out

    def is_wall(self, x_odom, y_odom):
        xw = x_odom + self.spawn_x
        yw = y_odom + self.spawn_y
        gx = int(xw / self.resolution)
        gy = int(yw / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.width_cells or gy >= self.height_cells:
            return True
        return self.grid_inflated[gy][gx] == 1

    def wall_ahead(self):
        cos_t = math.cos(self.robot_theta)
        sin_t = math.sin(self.robot_theta)
        d = 0.1
        while d < self.wall_check_dist:
            for lat in [-0.18, 0.0, 0.18]:
                tx = self.robot_x + d * cos_t + lat * (-sin_t)
                ty = self.robot_y + d * sin_t + lat * cos_t
                if self.is_wall(tx, ty):
                    return d
            d += 0.1
        return float('inf')

    # ---------------- CALLBACK ----------------
    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)
        self.have_odom = True

    # ---------------- DECISION ----------------
    def select_next_aisle(self):
        uncleaned = [a for a in self.aisles if not a.cleaned]
        if not uncleaned:
            return None
        uncleaned.sort(key=lambda a: abs(self.robot_x - a.center_x))
        return uncleaned[0]

    def generate_cleaning_waypoints(self, aisle):
        cx = aisle.center_x
        ay = self.robot_lane_y
        ey = self.aisle_entrance_y
        by = self.aisle_back_y
        # Straight up the centre, straight back. No lateral shifts near walls.
        return [
            (cx, ay),     # corridor approach (aligned)
            (cx, ey),     # enter aisle (straight)
            (cx, by),     # drive to back (straight up centre)
            (cx, ey),     # drive back to entrance (straight down)
            (cx, ay),     # return to corridor
        ]

    # ---------------- MOTION ----------------
    def drive_to(self, target):
        dx = target[0] - self.robot_x
        dy = target[1] - self.robot_y
        dist = math.hypot(dx, dy)
        if dist < self.waypoint_tolerance:
            return 0.0, 0.0, True

        heading_err = self.normalize(math.atan2(dy, dx) - self.robot_theta)

        # Rotate in place first if not aligned
        if abs(heading_err) > self.align_threshold:
            omega = math.copysign(min(self.max_omega, abs(heading_err) * 2.0), heading_err)
            return 0.0, omega, False

        # Aligned — drive straight with only gentle correction
        v = self.max_v
        if dist < self.approach_slowdown:
            v *= max(0.3, dist / self.approach_slowdown)
        v = max(self.min_v, min(self.max_v, v))

        # Gentle heading hold, capped low so it can't curve into walls
        omega = max(-0.3, min(0.3, heading_err * 0.8))

        # WALL REFLEX
        wd = self.wall_ahead()
        if wd < self.wall_stop_dist:
            return 0.0, 0.0, False         # stop dead
        elif wd < self.wall_check_dist:
            v = self.min_v
            omega = 0.0                    # crawl dead straight near walls

        return v, omega, False

    # ---------------- STATE MACHINE ----------------
    def control_loop(self):
        if not self.have_odom:
            return

        if self.state == 'INIT':
            self.state = 'GO_TO_HUB'
            self.get_logger().info('START — heading to hub')

        elif self.state == 'GO_TO_HUB':
            v, w, arrived = self.drive_to(self.hub)
            self.send(v, w)
            if arrived:
                self.state = 'AT_HUB'

        elif self.state == 'AT_HUB':
            self.send(0.0, 0.0)
            nxt = self.select_next_aisle()
            if nxt is None:
                self.state = 'RETURNING_HOME'
                self.get_logger().info('All aisles cleaned — returning home')
            else:
                self.committed_aisle = nxt
                self.clean_waypoints = self.generate_cleaning_waypoints(nxt)
                self.clean_index = 0
                self.state = 'CLEANING'
                self.get_logger().info(f'Cleaning {nxt.name}')

        elif self.state == 'CLEANING':
            if self.clean_index >= len(self.clean_waypoints):
                self.committed_aisle.cleaned = True
                self.get_logger().info(f'DONE {self.committed_aisle.name}')
                self.state = 'GO_TO_HUB'
                return
            target = self.clean_waypoints[self.clean_index]
            v, w, arrived = self.drive_to(target)
            self.send(v, w)
            if arrived:
                self.clean_index += 1

        elif self.state == 'RETURNING_HOME':
            v, w, arrived = self.drive_to(self.home)
            self.send(v, w)
            if arrived:
                self.state = 'COMPLETE'
                self.get_logger().info('MISSION COMPLETE')

        elif self.state == 'COMPLETE':
            self.send(0.0, 0.0)

    # ---------------- UTILS ----------------
    def normalize(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def send(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def publish_status(self):
        cleaned = [a.name for a in self.aisles if a.cleaned]
        m = String()
        m.data = f'STATE={self.state} cleaned={cleaned}'
        self.state_pub.publish(m)

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
                data.append(100 if self.grid_clean[gy][gx] == 1 else 0)
        msg.data = data
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionBrain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()