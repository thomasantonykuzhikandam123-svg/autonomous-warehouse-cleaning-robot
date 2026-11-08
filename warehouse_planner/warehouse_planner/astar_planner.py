#!/usr/bin/env python3
"""A* planner — goal on /next_waypoint, path on /planned_path, map on /map."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
import heapq
import math


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        self.resolution = 0.2
        self.spawn_x = 4.0
        self.spawn_y = 4.0
        self.width_cells = int(15.0 / self.resolution)
        self.height_cells = int(20.0 / self.resolution)
        self.grid_clean = None
        self.grid_inf = self.build_map()

        self.rx = self.spawn_x
        self.ry = self.spawn_y

        self.create_subscription(PoseStamped, '/next_waypoint', self.goal_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info('A* planner ready')

    def w2g(self, x, y):
        return int(x / self.resolution), int(y / self.resolution)

    def g2w(self, gx, gy):
        return gx * self.resolution + self.resolution / 2, gy * self.resolution + self.resolution / 2

    def add_wall(self, grid, xc, yc, lx, ly):
        gx0, gy0 = self.w2g(xc - lx / 2, yc - ly / 2)
        gx1, gy1 = self.w2g(xc + lx / 2, yc + ly / 2)
        for gy in range(max(0, gy0), min(self.height_cells, gy1 + 1)):
            for gx in range(max(0, gx0), min(self.width_cells, gx1 + 1)):
                grid[gy][gx] = 1

    def build_map(self):
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
        self.grid_clean = [r[:] for r in g]
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

    def odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x + self.spawn_x
        self.ry = msg.pose.pose.position.y + self.spawn_y

    def goal_cb(self, msg):
        gx = msg.pose.position.x + self.spawn_x
        gy = msg.pose.position.y + self.spawn_y
        self.plan(self.rx, self.ry, gx, gy)

    def free(self, gx, gy):
        if gx < 0 or gy < 0 or gx >= self.width_cells or gy >= self.height_cells:
            return False
        return self.grid_inf[gy][gx] == 0

    def nearest_free(self, cell, mr=15):
        gx0, gy0 = cell
        for r in range(1, mr):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    if self.free(gx0 + dx, gy0 + dy):
                        return (gx0 + dx, gy0 + dy)
        return None

    def astar(self, start, goal):
        opens = [(0, start)]
        came = {}
        gsc = {start: 0}
        nbr = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        while opens:
            _, cur = heapq.heappop(opens)
            if cur == goal:
                p = [cur]
                while cur in came:
                    cur = came[cur]
                    p.append(cur)
                return p[::-1]
            for dx, dy in nbr:
                nb = (cur[0] + dx, cur[1] + dy)
                if not self.free(*nb):
                    continue
                tg = gsc[cur] + math.hypot(dx, dy)
                if nb not in gsc or tg < gsc[nb]:
                    came[nb] = cur
                    gsc[nb] = tg
                    h = math.hypot(nb[0] - goal[0], nb[1] - goal[1])
                    heapq.heappush(opens, (tg + h, nb))
        return None

    def los(self, a, b):
        x0, y0 = a
        x1, y1 = b
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if not self.free(x, y):
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def smooth(self, path):
        if len(path) < 3:
            return path
        res = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self.los(path[i], path[j]):
                    break
                j -= 1
            res.append(path[j])
            i = j
        return res

    def densify(self, corners, step=0.1):
        if len(corners) < 2:
            return corners
        res = [corners[0]]
        for i in range(len(corners) - 1):
            x0, y0 = corners[i]
            x1, y1 = corners[i + 1]
            d = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(d / step))
            for s in range(1, n + 1):
                t = s / n
                res.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        return res

    def plan(self, sx, sy, gx, gy):
        s = self.w2g(sx, sy)
        g = self.w2g(gx, gy)
        if not self.free(*s):
            s = self.nearest_free(s)
        if not self.free(*g):
            g = self.nearest_free(g)
        if s is None or g is None:
            self.get_logger().warn('start/goal blocked')
            return
        gp = self.astar(s, g)
        if gp is None:
            self.get_logger().warn('no path')
            return
        gp = self.smooth(gp)
        corners = [self.g2w(cx, cy) for cx, cy in gp]
        dense = self.densify(corners)
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in dense:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = wx - self.spawn_x
            p.pose.position.y = wy - self.spawn_y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)
        self.get_logger().info(f'path: {len(corners)} corners, {len(dense)} pts')

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
        d = []
        for gy in range(self.height_cells):
            for gx in range(self.width_cells):
                d.append(100 if self.grid_clean[gy][gx] == 1 else 0)
        msg.data = d
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AStarPlanner())
    rclpy.shutdown()


if __name__ == '__main__':
    main()