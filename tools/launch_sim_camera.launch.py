"""launch_sim_camera.launch.py (v10) - your launch_sim.launch.py, run as it is, with the camera
and, if you want, without the Gazebo window.

    ros2 launch ~/ros2_ws/src/tools/launch_sim_camera.launch.py

It loads warehouse_cleaning_robot's own launch_sim.launch.py and runs it unchanged. The one
difference: when that file turns robot.urdf.xacro into the robot description, it gets
camera/robot_camera.urdf.xacro instead, which includes robot.urdf.xacro as it is and adds the
camera (check_camera_urdf.py proves the robot itself is identical). Plus one more bridge, for
the camera topics. Gazebo, the world, the spawn pose, the other bridges and the EKF are exactly
as in your file, and they follow it if you ever change it.

Camera settings (defaults 640 x 480, 5 Hz, 90 deg) can be changed for one start, e.g.
    EYE_WIDTH=320 EYE_HEIGHT=240 EYE_RATE=10 ros2 launch ...
v10:
    SIM_CAMERA=0     no camera: the robot exactly as in your file (still through this wrapper)
    SIM_HEADLESS=1   Gazebo's server only, no window (gz_args gets '-s'): the window alone used
                     about two CPU cores in the picker run; the simulation, sensors and RViz are
                     unchanged. If your launch file has no gz_args, this says so and changes nothing.
"""
import importlib.util
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
ADDON = os.path.join(HERE, 'camera', 'robot_camera.urdf.xacro')
CAMERA = {'width': '640', 'height': '480', 'rate': '5', 'hfov': '1.5708'}


def camera_mappings(base):
    """xacro arguments for the add-on: the original file's path and the camera settings."""
    maps = {k: os.environ.get('EYE_' + k.upper(), v) for k, v in CAMERA.items()}
    maps['base'] = str(base)
    return maps


def server_only(value):
    """gz_args value with '-s' in front (str, substitution or a list of them)."""
    if isinstance(value, str):
        return '-s ' + value
    if isinstance(value, (list, tuple)):
        return ['-s '] + list(value)
    return ['-s ', value]


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_cleaning_robot')
    original = os.path.join(pkg, 'launch', 'launch_sim.launch.py')
    camera = os.environ.get('SIM_CAMERA', '1') != '0'
    headless = os.environ.get('SIM_HEADLESS', '0') == '1'
    plain = xacro.process_file
    plain_init = IncludeLaunchDescription.__init__
    found = {'gz_args': False}

    def with_camera(path, *args, **kwargs):
        if os.path.basename(str(path)) == 'robot.urdf.xacro':
            return plain(ADDON, mappings=camera_mappings(path))
        return plain(path, *args, **kwargs)

    def include_headless(self, launch_description_source, *args, launch_arguments=None, **kwargs):
        if launch_arguments is not None:
            items = list(launch_arguments.items()) if hasattr(launch_arguments, 'items') else list(launch_arguments)
            new = []
            for k, v in items:
                if isinstance(k, str) and k == 'gz_args':
                    v = server_only(v)
                    found['gz_args'] = True
                new.append((k, v))
            launch_arguments = new
        plain_init(self, launch_description_source, *args, launch_arguments=launch_arguments, **kwargs)

    if camera:
        xacro.process_file = with_camera
    if headless:
        IncludeLaunchDescription.__init__ = include_headless
    try:
        spec = importlib.util.spec_from_file_location('launch_sim_original', original)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ld = mod.generate_launch_description()
    finally:
        xacro.process_file = plain
        IncludeLaunchDescription.__init__ = plain_init
    if headless:
        ld.add_action(LogInfo(msg='SIM_HEADLESS=1: Gazebo server only (no window)' if found['gz_args'] else
                              'SIM_HEADLESS=1 ignored: launch_sim.launch.py passes no gz_args, so Gazebo '
                              'starts with its window as usual'))
    if camera:
        ld.add_action(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                       '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            output='screen'))
    return ld
