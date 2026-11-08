import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # the .yaml that pairs with your .pgm (NOT the slam .posegraph)
    map_yaml = os.path.join(
        get_package_share_directory('warehouse_cleaning_robot'),
        'maps', 'slam_map.yaml')   # the occupancy grid SLAM Toolbox built -> visible proof SLAM ran

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml,
            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    # map_server is a lifecycle node; this auto-configures + activates it
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
            # On a slow VM (UTM/M3) the bond heartbeat between the manager and
            # map_server times out, so the manager thinks the server died and
            # tears it down right after activating -> /map never stays up.
            # 0.0 disables the bond check. Same VM-timing class of bug as the
            # BT tick-rate spam and TF tolerance.
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([map_server, lifecycle])