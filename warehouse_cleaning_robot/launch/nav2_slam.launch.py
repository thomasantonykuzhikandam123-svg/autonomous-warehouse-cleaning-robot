import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_cleaning_robot')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_params_path = os.path.join(pkg, 'config', 'nav2_params.yaml')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([navigation])