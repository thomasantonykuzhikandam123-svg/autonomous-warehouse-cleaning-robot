import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_cleaning_robot')
    slam_params_path = os.path.join(pkg, 'config', 'slam_localization.yaml')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_path,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        slam_toolbox,
    ])