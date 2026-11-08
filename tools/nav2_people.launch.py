"""nav2_people.launch.py (v7) - your Nav2 plus the safety chain for people_guard.py.

    ros2 launch ~/ros2_ws/src/tools/nav2_people.launch.py

Your nav2_slam.launch.py is included unchanged. Around it, two remap rules:

  cmd_vel_smoothed -> cmd_vel_raw       the velocity smoother no longer drives the
                                        wheels; its commands go through the gate and
                                        the Collision Monitor:
      velocity_smoother --cmd_vel_raw--> guard_gate --cmd_vel_safe_in-->
                                         collision_monitor --cmd_vel--> robot
  /scan -> /scan_nav                    Nav2's costmaps read people_guard.py's copy of
                                        the scan, with the people it handles taken out
                                        (it predicts them and holds the robot itself).
                                        Racks, boxes and anyone standing still stay in it.

guard_gate.py (v7) passes commands only while people_guard.py and the lidar are
alive, and sends zero otherwise (fail-safe). The Collision Monitor is outside the
group, so it still reads the raw /scan (its emergency braking sees everything).
people_guard.py also sends Nav2's controller a speed limit near people
(/speed_limit, the topic Nav2's Speed Filter uses), so it eases off and starts gently.

Why the rules win over the file's own remaps: launch_ros (Humble) puts SetRemap
rules before a node's own remappings, and ROS uses the first rule that matches.
check_wiring.py confirms all of it on the running system (stack.sh runs it for you).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

HERE = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    nav2 = os.path.join(get_package_share_directory('warehouse_cleaning_robot'),
                        'launch', 'nav2_slam.launch.py')
    params = os.path.join(HERE, 'collision_monitor_people.yaml')
    return LaunchDescription([
        GroupAction([
            SetRemap(src='cmd_vel_smoothed', dst='cmd_vel_raw'),
            SetRemap(src='/scan', dst='/scan_nav'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2)),
        ]),
        ExecuteProcess(cmd=['python3', os.path.join(HERE, 'guard_gate.py')],
                       name='guard_gate', output='screen', respawn=True, respawn_delay=1.0),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', emulate_tty=True,
             parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_people', output='screen', emulate_tty=True,
             parameters=[{'use_sim_time': True}, {'autostart': True},
                         {'node_names': ['collision_monitor']}]),
    ])
