#!/usr/bin/env python3
"""guard_gate.py (v10) - the robot only moves while people_guard.py and the lidar are alive,
and never while the emergency stop is pressed.

    velocity_smoother --cmd_vel_raw--> guard_gate --cmd_vel_safe_in--> collision_monitor --cmd_vel--> robot

Every velocity command passes through unchanged as long as, within the last
HEARTBEAT_S seconds of simulation time,
  - people_guard.py has published /people_guard/hold (it does after every scan it
    has fully processed, empty or not), and
  - the lidar has published /scan.
If either goes quiet (the guard crashed, hung, lost TF or the map, or the lidar
stopped), every command is replaced by zero, so the robot stops instead of driving
on blind. Nav2 Humble's Collision Monitor can't do this itself: it ignores a silent
source rather than stopping. Commands pass again by themselves once both are back.

v10: EMERGENCY STOP. A True on /emergency_stop (std_msgs/Bool) replaces every
command by zero until a False arrives - whatever Nav2 or the guard want, and with
no time limit (an emergency stop stays pressed until someone releases it):
    ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
    ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"

Started by nav2_people.launch.py (so it is up before the guard: until the guard
speaks, the robot stays still). Prints one line when it stops the robot and one
when it lets commands through again; nav2_last.log keeps them for the report.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool

HEARTBEAT_S = 0.35     # s (sim time) of silence from the guard or the lidar before the robot stops


def blocked_reason(t, guard_t, scan_t, limit=HEARTBEAT_S, estop=False):
    """Why commands must not pass at sim time t (None: they may)."""
    if estop:
        return 'EMERGENCY STOP pressed (/emergency_stop)'
    if guard_t is None:
        return 'no message from people_guard.py yet'
    if t - guard_t > limit:
        return f'people_guard.py silent for {t - guard_t:.1f} s'
    if scan_t is None:
        return 'no /scan yet'
    if t - scan_t > limit:
        return f'/scan silent for {t - scan_t:.1f} s'
    return None


def main():
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:
        ExternalShutdownException = KeyboardInterrupt

    class Gate(Node):
        def __init__(self):
            super().__init__('guard_gate', parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            self.guard_t = self.scan_t = None
            self.estop = False
            self.blocked = 'starting'              # why commands are stopped (None: passing)
            self.pub = self.create_publisher(Twist, 'cmd_vel_safe_in', 10)
            self.create_subscription(Twist, 'cmd_vel_raw', self.on_cmd, 10)
            self.create_subscription(PointCloud2, '/people_guard/hold', self.on_guard, 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Bool, '/emergency_stop', self.on_estop, 10)
            print('guard_gate v10: velocity commands pass only while people_guard.py and the '
                  f'lidar are alive (silence > {HEARTBEAT_S} s = stop), and never while '
                  '/emergency_stop is pressed', flush=True)

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_guard(self, msg):
            self.guard_t = self.now()

        def on_scan(self, msg):
            self.scan_t = self.now()

        def on_estop(self, msg):
            on = bool(msg.data)
            if on != self.estop:
                print(f'[{self.now():8.1f}] gate ' + ('EMERGENCY STOP pressed: robot held at zero velocity '
                      'until released' if on else 'emergency stop released'), flush=True)
            self.estop = on
            if on:
                self.pub.publish(Twist())          # at once, not at the next command

        def on_cmd(self, msg):
            t = self.now()
            for k in ('guard_t', 'scan_t'):        # sim time went backwards (world reset)
                if getattr(self, k) is not None and getattr(self, k) > t + 1.0:
                    setattr(self, k, None)
            why = blocked_reason(t, self.guard_t, self.scan_t, estop=self.estop)
            if why is None:
                if self.blocked is not None:
                    print(f'[{t:8.1f}] gate OPEN: guard and lidar alive, commands pass', flush=True)
                    self.blocked = None
                self.pub.publish(msg)
                return
            if self.blocked is None or (self.blocked == 'starting' and why != self.blocked):
                print(f'[{t:8.1f}] gate STOP: {why}; robot held at zero velocity', flush=True)
            self.blocked = why
            self.pub.publish(Twist())

    rclpy.init()
    node = Gate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
