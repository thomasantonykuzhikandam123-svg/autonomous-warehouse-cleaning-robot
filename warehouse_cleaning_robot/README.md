# Warehouse Cleaning Robot

A 4-wheeled autonomous mobile robot simulation built with ROS2 Humble and Ignition Gazebo Fortress as part of my Masters in Robotics and Mechatronics at RMIT University.

## What it does
The robot spawns in a Gazebo simulation environment and can be driven using keyboard teleoperation. The project is being developed to implement autonomous navigation with A* path planning and DWA obstacle avoidance for warehouse cleaning applications.

## Technologies Used
- ROS2 Humble
- Ignition Gazebo Fortress (version 6)
- Ubuntu 22.04
- Python
- URDF/Xacro for robot description

## Requirements
- ROS2 Humble
- ros-humble-ros-gz-sim
- ros-humble-ros-gz-bridge
- ros-humble-robot-state-publisher
- teleop-twist-keyboard

## How to Build and Run

Clone the repository into your ROS2 workspace:
cd ~/ros2_ws/src
git clone https://github.com/thomasantonykuzhikandam123-svg/warehouse_cleaning_robot.git

Build the package:
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

Launch the simulation:
ros2 launch warehouse_cleaning_robot launch_sim.launch.py

Drive the robot:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

## Project Status
- [x] Robot URDF with 4 wheels and lidar
- [x] Spawning in Ignition Gazebo
- [x] Keyboard teleoperation
- [ ] Warehouse world with aisles and obstacles
- [ ] A* global path planning
- [ ] DWA local obstacle avoidance
- [ ] Autonomous navigation

## Author
Thomas Antony — RMIT University Masters in Robotics and Mechatronics