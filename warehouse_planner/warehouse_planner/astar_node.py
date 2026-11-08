#!/usr/bin/env python3
"""
Original A* node — subscribes to /goal_pose, publishes /map and /planned_path.
Cost map + inflation + Theta*-style smoothing + continuous densification.
"""

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
        self.width_m = 15.0
        self.height_m = 20.0
        self.spawn_x = 4.0
        self.spawn_y = 4.0
        self.width_cells = int(self.width_m / self.resolution)
        self.height_cells = int(self.height_m / self.resolution)

        self.static_grid_clean = None
        self.static_grid_inflated = self.build_static_map()
        self.cost_map = self.build_cost_map(self.static_grid_inflated)

        self.robot_x = self.spawn_x
        self.robot_y = self.spawn_y

        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.create_timer(1.0, self.publish_map)

        self.get_logger().info('A* node ready (subscribes /goal_pose)')

    def world_to_grid(self, x, y):
        return int(x / self.resolution), int(y / self.resolution)

    def grid_to_world(self, gx, gy):
        return (gx * self.resolution + self.resolution / 2,
                gy * self.resolution + self.resolution / 2)

    def add_wall_to(self, grid, xc, yc, lx, ly):
        gx_min, gy_min = self.world_to_grid(xc - lx / 2, yc - ly / 2)
        gx_max, gy_max = self.world_to_grid(xc + lx / 2, yc + ly / 2)
        for gy in range(max(0, gy_min), min(self.height_cells, gy_max + 1)):
            for gx in range(max(0, gx_min), min(self.width_cells, gx_max + 1)):
                grid[gy][gx] = 1

    def build_static_map(self):
        clean = [[0] * self.width_cells for _ in range(self.height_cells)]
        self.add_wall_to(clean, 7.5, 20.0, 15.0, 0.2)
        self.add_wall_to(clean, 7.5, 0.0, 15.0, 0.2)
        self.add_wall_to(clean, 0.0, 10.0, 0.2, 20.0)
        self.add_wall_to(clean, 15.0, 10.0, 0.2, 20.0)
        self.add_wall_to(clean, 2.5, 1.5, 3, 2)
        self.add_wall_to(clean, 5.0, 1.5, 1, 2)
        self.add_wall_to(clean, 12.0, 1.5, 4, 2)
        self.add_wall_to(clean, 1.5, 11.5, 1, 12)
        self.add_wall_to(clean, 5.5, 11.5, 1, 12)
        self.add_wall_to(clean, 9.5, 11.5, 1, 12)
        self.add_wall_to(clean, 13.5, 11.5, 1, 12)
        self.add_wall_to(clean, 0.6, 9.0, 0.8, 1.2)
        self.add_wall_to(clean, 14.4, 9.0, 0.8, 1.2)
        self.static_grid_clean = [row[:] for row in clean]
        return self.inflate(clean, 2)

    def inflate(self, grid, r):
        h = len(grid); w = len(grid[0])
        out = [[0] * w for _ in range(h)]
        for gy in range(h):
            for gx in range(w):
                if grid[gy][gx] == 1:
                    for dy in range(-r, r + 1):
                        for dx in range(-r, r + 1):
                            if dx * dx + dy * dy <= r * r:
                                ny, nx = gy + dy, gx + dx
                                if 0 <= ny < h and 0 <= nx < w:
                                    out[ny][nx] = 1
        return out

    def build_cost_map(self, grid):
        h = self.height_cells; w = self.width_cells
        INF = 9999
        dist = [[INF] * w for _ in range(h)]
        q = []
        for gy in range(h):
            for gx in range(w):
                if grid[gy][gx] == 1:
                    dist[gy][gx] = 0
                    q.append((gx, gy))
        head = 0
        while head < len(q):
            gx, gy = q[head]; head += 1
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if dist[ny][nx] > dist[gy][gx] + 1:
                        dist[ny][nx] = dist[gy][gx] + 1
                        q.append((nx, ny))
        cost = [[0.0] * w for _ in range(h)]
        for gy in range(h):
            for gx in range(w):
                d = dist[gy][gx]
                if d == 0:
                    cost[gy][gx] = float('inf')
                elif d < 2:
                    cost[gy][gx] = 1.0 * math.exp(-d / 1.5)
        return cost

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x + self.spawn_x
        self.robot_y = msg.pose.pose.position.y + self.spawn_y

    def goal_callback(self, msg):
        gx_w = msg.pose.position.x + self.spawn_x
        gy_w = msg.pose.position.y + self.spawn_y
        self.plan_path(self.robot_x, self.robot_y, gx_w, gy_w)

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def is_free(self, gx, gy):
        if gx < 0 or gy < 0 or gx >= self.width_cells or gy >= self.height_cells:
            return False
        return self.static_grid_inflated[gy][gx] == 0

    def astar(self, start, goal):
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct(came_from, current)
            for dx, dy in neighbors:
                nb = (current[0] + dx, current[1] + dy)
                if not self.is_free(*nb):
                    continue
                step = math.hypot(dx, dy)
                extra = self.cost_map[nb[1]][nb[0]]
                tg = g_score[current] + step + extra
                if nb not in g_score or tg < g_score[nb]:
                    came_from[nb] = current
                    g_score[nb] = tg
                    f = tg + self.heuristic(nb, goal)
                    heapq.heappush(open_set, (f, nb))
        return None

    def reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def line_of_sight(self, g1, g2):
        x0, y0 = g1; x1, y1 = g2
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if not self.is_free(x, y):
                return False
            if x == x1 and y == y1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; x += sx
            if e2 < dx:
                err += dx; y += sy

    def smooth(self, path):
        if len(path) < 3:
            return path
        result = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self.line_of_sight(path[i], path[j]):
                    break
                j -= 1
            result.append(path[j])
            i = j
        return result

    def find_nearest_free(self, cell, max_r=12):
        gx0, gy0 = cell
        for r in range(1, max_r):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = gx0 + dx, gy0 + dy
                    if self.is_free(nx, ny):
                        return (nx, ny)
        return None

    def plan_path(self, sx, sy, gx, gy):
        start = self.world_to_grid(sx, sy)
        goal = self.world_to_grid(gx, gy)
        if not self.is_free(*start):
            start = self.find_nearest_free(start)
            if start is None:
                return
        if not self.is_free(*goal):
            goal = self.find_nearest_free(goal)
            if goal is None:
                return
        grid_path = self.astar(start, goal)
        if grid_path is None:
            self.get_logger().warn('No path')
            return
        grid_path = self.smooth(grid_path)
        self.publish_path(grid_path)

    def publish_path(self, grid_path):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        corners = [self.grid_to_world(gx, gy) for gx, gy in grid_path]
        dense = self.densify(corners, 0.1)
        for wx, wy in dense:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = wx - self.spawn_x
            p.pose.position.y = wy - self.spawn_y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def densify(self, corners, step):
        if len(corners) < 2:
            return corners
        result = [corners[0]]
        for i in range(len(corners) - 1):
            x0, y0 = corners[i]; x1, y1 = corners[i + 1]
            dx = x1 - x0; dy = y1 - y0
            dist = math.hypot(dx, dy)
            n = max(1, int(dist / step))
            for s in range(1, n + 1):
                t = s / n
                result.append((x0 + t * dx, y0 + t * dy))
        return result

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


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()