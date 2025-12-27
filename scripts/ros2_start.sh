#!/bin/bash
# Minimal ROS 2 Startup Script with Debug Logging
LOGFILE=/home/aa/ros2_ws/scripts/debug.log
mkdir -p /home/aa/ros2_ws/scripts
echo "--- Service Started at $(date) ---" >> $LOGFILE

# Environment
source /opt/ros/humble/setup.bash >> $LOGFILE 2>&1
source /home/aa/ros2_ws/install/setup.bash >> $LOGFILE 2>&1

# Check variables
echo "PYTHONPATH: $PYTHONPATH" >> $LOGFILE
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH" >> $LOGFILE

# Run
ros2 launch rc_car_control car_teleop.launch.py >> $LOGFILE 2>&1
