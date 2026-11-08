#!/usr/bin/env python3
"""
Warehouse traffic simulator -- intelligent dynamic obstacles for testing the
cleaner's avoidance. Phase 2A: ONE forklift loops storage aisle B following
real warehouse rules:

  * KEEP-LEFT lane discipline (Australian / keep-left convention): it drives up
    the WEST side of the aisle and down the EAST side, so it is always on the
    left relative to its own direction of travel.
  * Realistic aisle speed (~0.9 m/s, well under the ~1.3 m/s / 3 mph guidance
    for high-traffic areas), and it SLOWS to a crawl near each turn (corner).

The forklift is published two ways:
  * /traffic_markers : an RViz CUBE so you can SEE it (forklift-shaped box).
  * /forklift_points : a PointCloud2 of its footprint outline, in the 'map'
    frame. The cleaner's costmaps AND the Collision Monitor consume this as an
    obstacle source, so the cleaner detects and yields to it exactly as it
    would from real lidar returns.

(To also show it as a physical model in Gazebo, set that model's pose to
(self.x, self.y, self.yaw) each tick via your Gazebo set_pose service -- the
motion logic here is the single source of truth.)
"""

import math
import struct
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class Agent:
    def __init__(self, name, route, speed, size):
        self.name = name
        self.route = route          # list of (x,y) world waypoints, keep-left baked in
        self.speed = speed          # cruise speed (m/s)
        self.size = size            # (length, width) metres
        self.x, self.y = route[0]
        self.yaw = 0.0
        self.seg = 0                # index of the segment we are driving toward


class WarehouseTraffic(Node):
    def __init__(self):
        super().__init__('warehouse_traffic')
        self.dt = 0.1
        self.turn_slow_dist = 1.5       # start slowing this far from a corner (m)
        self.turn_speed = 0.30          # crawl speed through corners (m/s)

        # ---- Forklift route: loop storage aisle A (world x 2..5), KEEP LEFT ----
        # This is the FIRST aisle the cleaner sweeps, so the interaction happens
        # early and is easy to watch.
        # Going NORTH (+y) the left side is WEST (lower x) -> x = 2.8
        # Going SOUTH (-y) the left side is EAST (higher x) -> x = 4.2
        fork_route = [
            (2.8, 3.0),    (2.8, 19.0),     # up the west side  (left when heading N)
            (4.2, 19.0),   (4.2, 3.0),      # down the east side (left when heading S)
            (2.8, 3.0),                     # close the loop
        ]
        self.agents = [
            Agent('forklift1', fork_route, speed=0.9, size=(1.2, 0.8)),
        ]

        self.mk = self.create_publisher(MarkerArray, '/traffic_markers', 10)
        self.pc = self.create_publisher(PointCloud2, '/forklift_points', 10)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'Traffic running: {len(self.agents)} agent(s), keep-left rules.')

    # ----------------------------------------------------------------- #
    def tick(self):
        points = []
        ma = MarkerArray()
        for i, a in enumerate(self.agents):
            self.advance(a)
            points.extend(self.footprint_points(a))
            ma.markers.append(self.box_marker(a, i))
        self.publish_cloud(points)
        self.mk.publish(ma)

    def advance(self, a):
        """Drive the agent toward its next waypoint; loop at the end. Slows near
        corners so motion looks like a real operated vehicle."""
        tx, ty = a.route[a.seg + 1]
        dx, dy = tx - a.x, ty - a.y
        dist = math.hypot(dx, dy)
        if dist < 0.15:                          # reached this waypoint
            a.seg += 1
            if a.seg >= len(a.route) - 1:        # completed the loop
                a.seg = 0
            return
        v = self.turn_speed if dist < self.turn_slow_dist else a.speed
        a.yaw = math.atan2(dy, dx)
        a.x += v * math.cos(a.yaw) * self.dt
        a.y += v * math.sin(a.yaw) * self.dt

    def footprint_points(self, a):
        """Points sampled around the forklift's rectangular footprint at its
        current pose, in the 'map' frame."""
        L, W = a.size
        c, s = math.cos(a.yaw), math.sin(a.yaw)
        local = []
        nl = max(2, int(L / 0.1))
        nw = max(2, int(W / 0.1))
        for ix in range(nl + 1):
            lx = -L / 2 + L * ix / nl
            local.append((lx, -W / 2))
            local.append((lx, W / 2))
        for iy in range(nw + 1):
            ly = -W / 2 + W * iy / nw
            local.append((-L / 2, ly))
            local.append((L / 2, ly))
        pts = []
        for (lx, ly) in local:
            wx = a.x + c * lx - s * ly
            wy = a.y + s * lx + c * ly
            pts.append((wx, wy, 0.2))
        return pts

    def publish_cloud(self, pts):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'map'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        data = b''.join(struct.pack('fff', px, py, pz) for (px, py, pz) in pts)
        msg = PointCloud2()
        msg.header = h
        msg.height = 1
        msg.width = len(pts)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = data
        self.pc.publish(msg)

    def box_marker(self, a, i):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'traffic'
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = a.x
        m.pose.position.y = a.y
        m.pose.position.z = 0.4
        m.pose.orientation.z = math.sin(a.yaw / 2.0)
        m.pose.orientation.w = math.cos(a.yaw / 2.0)
        m.scale.x = a.size[0]
        m.scale.y = a.size[1]
        m.scale.z = 0.8
        m.color.r = 0.95
        m.color.g = 0.55
        m.color.b = 0.0
        m.color.a = 0.9
        return m


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WarehouseTraffic())
    rclpy.shutdown()


if __name__ == '__main__':
    main()