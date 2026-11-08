#!/usr/bin/env python3
"""
ground_truth_localizer.py  (v2 -- fixed from Thomas's tf2_echo diagnostics)

Publishes a PERFECT map->odom transform from Gazebo ground truth, replacing
slam_toolbox. Run INSTEAD of slam_localization (never both).

v2 fixes (both proven by the diagnostic output):
  1. SIM TIME -- v1 stamped TF with the WALL clock (1781243671...) while the
     rest of the stack runs on SIM time (24.3...). TF cannot mix clocks ->
     "extrapolation into the past" -> map->base_link unusable -> every Nav2
     goal instantly rejected. v2 forces use_sim_time=True in code, so it is
     correct no matter how it is launched.
  2. WORLD->MAP OFFSET -- gz's OdometryPublisher reports the robot's WORLD
     pose (/odom_truth read (4,4) at spawn, i.e. the spawn's world coords,
     not (0,0)). The map origin IS the spawn point, so: map = world - (4,4).
     Parameters world_to_map_x/y/yaw (default -4, -4, 0) apply this.
     NOTE: because /odom_truth is world-frame, changing the robot's spawn
     later (e.g. charging dock) requires NO change to this node.
  3. Stamps the transform +0.1 s into the future (same trick as AMCL's
     transform_tolerance) so "latest" lookups never fall off the buffer edge.

Chain:  map->odom (this node)  o  odom->base_link (EKF)  =  ground truth.
Any EKF drift is absorbed into map->odom, so the map pose is always exact.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import rclpy.time


def yaw_from_quat(q):
    """Z-axis yaw from a quaternion (2D)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def world_to_map(world_pose, offset):
    """Apply the constant world->map transform.  poses are (x, y, yaw)."""
    xw, yw, tw = world_pose
    ox, oy, oth = offset
    c, s = math.cos(oth), math.sin(oth)
    return (c * xw - s * yw + ox,
            s * xw + c * yw + oy,
            tw + oth)


def compute_map_to_odom(truth_map_pose, ekf_odom_pose):
    """map->odom = map_T_base(truth) o inverse(odom_T_base(EKF)).
    Returns (x, y, yaw) of the odom frame expressed in map."""
    xt, yt, tt = truth_map_pose
    xo, yo, to = ekf_odom_pose
    # base_T_odom = inverse(odom_T_base)
    ix = -(xo * math.cos(to) + yo * math.sin(to))
    iy = -(-xo * math.sin(to) + yo * math.cos(to))
    # compose
    c, s = math.cos(tt), math.sin(tt)
    return (c * ix - s * iy + xt,
            s * ix + c * iy + yt,
            tt - to)


class GroundTruthLocalizer(Node):
    def __init__(self):
        super().__init__('ground_truth_localizer')

        # FIX 1: force sim time regardless of launch arguments.
        self.set_parameters([Parameter('use_sim_time',
                                       Parameter.Type.BOOL, True)])

        # FIX 2: world -> map constant offset (map origin = original spawn).
        self.declare_parameter('world_to_map_x', -4.0)
        self.declare_parameter('world_to_map_y', -4.0)
        self.declare_parameter('world_to_map_yaw', 0.0)
        self.offset = (
            float(self.get_parameter('world_to_map_x').value),
            float(self.get_parameter('world_to_map_y').value),
            float(self.get_parameter('world_to_map_yaw').value),
        )

        self.truth_map = None            # (x, y, yaw) true pose in MAP frame
        self._announced = False
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/odom_truth', self.on_truth, 20)
        self.create_timer(0.02, self.publish_map_odom)      # 50 Hz
        self.get_logger().info(
            'ground_truth_localizer v2 up (sim time forced, world->map offset '
            f'{self.offset}). Waiting for /odom_truth + EKF...')

    def on_truth(self, msg):
        p = msg.pose.pose
        world = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.truth_map = world_to_map(world, self.offset)

    def publish_map_odom(self):
        if self.truth_map is None:
            return
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link',
                                                rclpy.time.Time())
        except Exception:
            return
        ekf = (t.transform.translation.x,
               t.transform.translation.y,
               yaw_from_quat(t.transform.rotation))

        mx, my, mth = compute_map_to_odom(self.truth_map, ekf)

        tf = TransformStamped()
        # FIX 3: future-date the stamp slightly (AMCL-style tolerance).
        tf.header.stamp = (self.get_clock().now()
                           + Duration(seconds=0.1)).to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = mx
        tf.transform.translation.y = my
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = math.sin(mth / 2.0)
        tf.transform.rotation.w = math.cos(mth / 2.0)
        self.br.sendTransform(tf)

        if not self._announced:
            self._announced = True
            self.get_logger().info(
                'PUBLISHING map->odom. Robot true map pose ~ '
                f'({self.truth_map[0]:.2f}, {self.truth_map[1]:.2f}) '
                '-- should read ~(0.00, 0.00) at spawn.')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GroundTruthLocalizer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()