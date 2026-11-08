#!/usr/bin/env python3
"""
Vehicle simulator — structured warehouse traffic.

Three human-operated vehicles patrol fixed loops at aisle entrances,
following warehouse lane discipline. Staggered timing ensures the
cleaning robot always faces meaningful decisions (some aisles free,
some busy).

Moves vehicles physically in Gazebo AND publishes ground-truth positions
(odom frame) for the mission brain.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import subprocess
import math
import time


class Vehicle:
    def __init__(self, name, color, speed, waypoints, pause_s):
        self.name = name
        self.color = color
        self.speed = speed
        self.waypoints = waypoints
        self.pause_s = pause_s
        self.x, self.y = waypoints[0]
        self.z = 0.3 if name != 'forklift' else 0.4
        self.target_idx = 1
        self.is_paused = False
        self.pause_until = 0.0

    def step(self, dt):
        now = time.time()
        if self.is_paused:
            if now >= self.pause_until:
                self.is_paused = False
            else:
                return
        tx, ty = self.waypoints[self.target_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.12:
            self.is_paused = True
            self.pause_until = now + self.pause_s
            self.target_idx = (self.target_idx + 1) % len(self.waypoints)
            return
        s = self.speed * dt
        if s > dist:
            s = dist
        self.x += (dx / dist) * s
        self.y += (dy / dist) * s


class VehicleSimulator(Node):

    def __init__(self):
        super().__init__('vehicle_simulator')
        self.spawn_x = 4.0
        self.spawn_y = 4.0

        # Routes in WORLD coords. Vehicles patrol between corridor (Y=4.2,
        # clear of robot lane at world Y=3.5) and aisle entrances (Y=6),
        # blocking entrances periodically. Staggered timing.

        # Tugger 1 — blocks aisle 1 entrance (world X=3.5)
        tugger1 = [
            (3.5, 4.2),    # corridor (aisle 1 clear)
            (3.5, 6.0),    # aisle 1 entrance (blocks it)
            (3.5, 4.2),    # back to corridor
        ]
        # Forklift — blocks aisle 2 entrance (world X=7.5), longer pauses
        forklift = [
            (7.5, 4.2),
            (7.5, 6.5),    # into aisle 2 entrance
            (7.5, 4.2),
        ]
        # Tugger 2 — blocks aisle 3 entrance (world X=11.5)
        tugger2 = [
            (11.5, 4.2),
            (11.5, 6.0),
            (11.5, 4.2),
        ]

        self.vehicles = [
            Vehicle('tugger_1', (0.2, 0.4, 0.9), 0.35, tugger1, 5.0),
            Vehicle('forklift', (1.0, 0.5, 0.1), 0.25, forklift, 9.0),
            Vehicle('tugger_2', (0.2, 0.7, 0.3), 0.30, tugger2, 6.0),
        ]

        self.last = time.time()
        self.pos_pub = self.create_publisher(PoseArray, '/detected_obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_obstacle_markers', 10)
        self.create_timer(0.1, self.tick)

        self.get_logger().info('Vehicle simulator (structured traffic) ready')

    def set_pose(self, name, x, y, z):
        req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
               f'orientation: {{x: 0, y: 0, z: 0, w: 1}}')
        try:
            subprocess.run(
                ['ign', 'service', '-s', '/world/warehouse/set_pose',
                 '--reqtype', 'ignition.msgs.Pose',
                 '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '300', '--req', req],
                capture_output=True, timeout=1.0)
        except Exception:
            pass

    def tick(self):
        now = time.time()
        dt = now - self.last
        self.last = now

        for v in self.vehicles:
            v.step(dt)
            self.set_pose(v.name, v.x, v.y, v.z)

        pa = PoseArray()
        pa.header.frame_id = 'odom'
        pa.header.stamp = self.get_clock().now().to_msg()
        for v in self.vehicles:
            p = Pose()
            p.position.x = v.x - self.spawn_x
            p.position.y = v.y - self.spawn_y
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pos_pub.publish(pa)

        ma = MarkerArray()
        for i, v in enumerate(self.vehicles):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'vehicles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = v.x - self.spawn_x
            m.pose.position.y = v.y - self.spawn_y
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = 0.6
            m.scale.y = 0.6
            m.scale.z = 0.6
            m.color.r = v.color[0]
            m.color.g = v.color[1]
            m.color.b = v.color[2]
            m.color.a = 0.85
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()