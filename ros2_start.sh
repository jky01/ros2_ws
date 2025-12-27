#!/bin/bash
# High-compatibility ROS 2 Startup Script
export HOME=/home/aa
export USER=aa
export SHELL=/bin/bash
source /opt/ros/humble/setup.bash
source /home/aa/ros2_ws/install/setup.bash
ros2 launch rc_car_control car_teleop.launch.py
