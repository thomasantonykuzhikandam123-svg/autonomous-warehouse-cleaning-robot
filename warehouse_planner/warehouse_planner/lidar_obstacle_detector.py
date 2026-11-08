#!/usr/bin/env python3
"""
Lidar-based dynamic obstacle detector.

Pipeline:
1. Read /scan
2. Convert ranges to (x,y) points in odom frame
3. Filter out points that hit known static walls (from /map)
4. Cluster remaining points (DBSCAN-style)
5. Match clusters between consecutive frames (nearest-neighbor)
6. Estimate velocities from cluster motion
7. Publish positions + velocities

Output: /detected_obstacles (PoseArray of obstacle positions)
        /detected_obstacle_velocities (Float32MultiArray of vx,vy pairs)
        /obstacle_markers (MarkerArray for RViz visualization)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import math
import time


class LidarObstacleDetector(Node):

    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        # Detection parameters
        self.cluster_threshold = 0.4       # points within this distance = same cluster
        self.min_cluster_size = 3          # need at least 3 lidar hits to count
        self.max_cluster_size = 30         # if bigger than this, probably a wall
        self.wall_match_threshold = 0.35   # if point is within this of a wall, ignore
        self.tracking_threshold = 1.0      # max distance to match cluster between frames

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.have_odom = False

        # Map data (static walls)
        self.map_data = None
        self.map_resolution = 0.2
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        # Previous frame's obstacles for velocity tracking
        self.prev_obstacles = []     # list of (x, y, timestamp)

        # ROS
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.obs_pub = self.create_publisher(PoseArray, '/detected_obstacles', 10)
        self.vel_pub = self.create_publisher(Float32MultiArray, '/detected_obstacle_velocities', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_obstacle_markers', 10)

        self.get_logger().info('Lidar obstacle detector ready')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_theta = math.atan2(siny, cosy)
        self.have_odom = True

    def map_callback(self, msg):
        self.map_data = list(msg.data)
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height

    def is_near_static_wall(self, x, y):
        """Check if (x,y) is within wall_match_threshold of any occupied map cell."""
        if self.map_data is None:
            return False
        # Check 3x3 grid of cells around point
        r = int(self.wall_match_threshold / self.map_resolution) + 1
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                cx, cy = gx + dx, gy + dy
                if 0 <= cx < self.map_width and 0 <= cy < self.map_height:
                    if self.map_data[cy * self.map_width + cx] > 50:
                        # Found a wall cell — check distance
                        wx = self.map_origin_x + (cx + 0.5) * self.map_resolution
                        wy = self.map_origin_y + (cy + 0.5) * self.map_resolution
                        if math.hypot(x - wx, y - wy) < self.wall_match_threshold:
                            return True
        return False

    def scan_callback(self, msg):
        if not self.have_odom or self.map_data is None:
            return

        # 1. Convert scan to (x,y) in odom frame
        angle = msg.angle_min
        points = []   # (x, y) in odom frame
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                # Lidar local frame: x is forward
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                # Transform to odom: rotate by robot_theta, translate by robot pos
                ox = self.robot_x + lx * math.cos(self.robot_theta) - ly * math.sin(self.robot_theta)
                oy = self.robot_y + lx * math.sin(self.robot_theta) + ly * math.cos(self.robot_theta)
                points.append((ox, oy))
            angle += msg.angle_increment

        # 2. Filter out points that hit static walls
        dynamic_points = []
        for px, py in points:
            if not self.is_near_static_wall(px, py):
                dynamic_points.append((px, py))

        # 3. Cluster dynamic points
        clusters = self.cluster_points(dynamic_points)

        # 4. Compute centroid of each cluster
        current_obstacles = []
        for cluster in clusters:
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                cx = sum(p[0] for p in cluster) / len(cluster)
                cy = sum(p[1] for p in cluster) / len(cluster)
                current_obstacles.append((cx, cy))

        # 5. Track obstacles between frames (estimate velocity)
        now = time.time()
        velocities = self.track_and_estimate_velocities(current_obstacles, now)

        # 6. Publish
        self.publish_obstacles(current_obstacles, velocities)
        self.publish_markers(current_obstacles, velocities)

    def cluster_points(self, points):
        """Simple distance-based clustering (DBSCAN-lite)."""
        if not points:
            return []
        visited = [False] * len(points)
        clusters = []
        for i in range(len(points)):
            if visited[i]:
                continue
            cluster = [points[i]]
            visited[i] = True
            # Greedy: find all nearby points
            j = 0
            while j < len(cluster):
                cx, cy = cluster[j]
                for k in range(len(points)):
                    if not visited[k]:
                        if math.hypot(points[k][0] - cx, points[k][1] - cy) < self.cluster_threshold:
                            cluster.append(points[k])
                            visited[k] = True
                j += 1
            clusters.append(cluster)
        return clusters

    def track_and_estimate_velocities(self, current_obstacles, now):
        """Match current obstacles to previous ones, compute velocities."""
        velocities = [(0.0, 0.0)] * len(current_obstacles)

        if not self.prev_obstacles:
            self.prev_obstacles = [(x, y, now) for x, y in current_obstacles]
            return velocities

        # For each current obstacle, find nearest previous
        new_prev = []
        for i, (cx, cy) in enumerate(current_obstacles):
            best_d = float('inf')
            best_prev = None
            for px, py, pt in self.prev_obstacles:
                d = math.hypot(cx - px, cy - py)
                if d < best_d and d < self.tracking_threshold:
                    best_d = d
                    best_prev = (px, py, pt)
            if best_prev is not None:
                px, py, pt = best_prev
                dt = now - pt
                if dt > 0.05:
                    vx = (cx - px) / dt
                    vy = (cy - py) / dt
                    # Clip noisy estimates
                    speed = math.hypot(vx, vy)
                    if speed > 1.5:
                        vx *= 1.5 / speed
                        vy *= 1.5 / speed
                    velocities[i] = (vx, vy)
            new_prev.append((cx, cy, now))
        self.prev_obstacles = new_prev
        return velocities

    def publish_obstacles(self, obstacles, velocities):
        pa = PoseArray()
        pa.header.frame_id = 'odom'
        pa.header.stamp = self.get_clock().now().to_msg()
        for x, y in obstacles:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.obs_pub.publish(pa)

        # Velocities as flat array [vx1, vy1, vx2, vy2, ...]
        vel_msg = Float32MultiArray()
        for vx, vy in velocities:
            vel_msg.data.append(float(vx))
            vel_msg.data.append(float(vy))
        self.vel_pub.publish(vel_msg)

    def publish_markers(self, obstacles, velocities):
        ma = MarkerArray()
        for i, ((x, y), (vx, vy)) in enumerate(zip(obstacles, velocities)):
            # Obstacle position marker (yellow cylinder)
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'detected_obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 0.4
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.6
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200000000   # 0.2s — auto-clear if stale
            ma.markers.append(m)

            # Velocity arrow
            speed = math.hypot(vx, vy)
            if speed > 0.05:
                arrow = Marker()
                arrow.header.frame_id = 'odom'
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = 'velocity_arrows'
                arrow.id = i
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                from geometry_msgs.msg import Point
                p_start = Point()
                p_start.x = x
                p_start.y = y
                p_start.z = 0.5
                p_end = Point()
                p_end.x = x + vx * 1.0   # 1s prediction
                p_end.y = y + vy * 1.0
                p_end.z = 0.5
                arrow.points = [p_start, p_end]
                arrow.scale.x = 0.05
                arrow.scale.y = 0.1
                arrow.scale.z = 0.0
                arrow.color.r = 1.0
                arrow.color.g = 0.0
                arrow.color.b = 0.0
                arrow.color.a = 1.0
                arrow.lifetime.sec = 0
                arrow.lifetime.nanosec = 200000000
                ma.markers.append(arrow)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()