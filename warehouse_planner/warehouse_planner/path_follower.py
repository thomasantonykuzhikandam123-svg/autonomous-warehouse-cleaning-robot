#!/usr/bin/env python3
"""Pure pursuit follower — follows /planned_path, stops exactly at goal."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import math


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.max_v = 0.60
        self.min_v = 0.06
        self.max_omega = 1.0
        self.lookahead = 0.6
        self.goal_tol = 0.30
        self.align_thresh = math.radians(20)

        self.path = []
        self.arrived = True
        self.aligning = False
        self.rx = self.ry = self.rtheta = 0.0
        self.have_odom = False

        self.create_subscription(Path, '/planned_path', self.path_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.loop)
        self.get_logger().info('Path follower ready')

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self.path:
            self.arrived = False
            self.aligning = True

    def odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rtheta = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def norm(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def far_point(self):
        for px, py in self.path:
            if math.hypot(px - self.rx, py - self.ry) > 0.5:
                return px, py
        return self.path[-1]

    def lookahead_point(self):
        bi, bd = 0, 1e9
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - self.rx, py - self.ry)
            if d < bd:
                bd, bi = d, i
        for i in range(bi, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - self.rx, py - self.ry) >= self.lookahead:
                return px, py
        return self.path[-1]

    def loop(self):
        if not self.have_odom or not self.path or self.arrived:
            self.cmd_pub.publish(Twist())
            return
        gx, gy = self.path[-1]
        if math.hypot(gx - self.rx, gy - self.ry) < self.goal_tol:
            self.cmd_pub.publish(Twist())
            self.arrived = True
            return
        if self.aligning:
            tx, ty = self.far_point()
            err = self.norm(math.atan2(ty - self.ry, tx - self.rx) - self.rtheta)
            if abs(err) < self.align_thresh:
                self.aligning = False
            else:
                c = Twist()
                c.angular.z = math.copysign(0.6, err)
                self.cmd_pub.publish(c)
                return
        lx, ly = self.lookahead_point()
        alpha = self.norm(math.atan2(ly - self.ry, lx - self.rx) - self.rtheta)
        Ld = math.hypot(lx - self.rx, ly - self.ry)
        curv = 2 * math.sin(alpha) / Ld if Ld > 0.01 else 0.0
        v = self.max_v
        if abs(alpha) > 0.5:
            v = self.min_v + (self.max_v - self.min_v) * max(0.0, 1 - abs(alpha) / 1.5)
        v = max(self.min_v, v)
        omega = max(-self.max_omega, min(self.max_omega, curv * v))
        c = Twist()
        c.linear.x = v
        c.angular.z = omega
        self.cmd_pub.publish(c)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathFollower())
    rclpy.shutdown()


if __name__ == '__main__':
    main()