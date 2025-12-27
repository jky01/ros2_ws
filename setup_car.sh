#!/bin/bash
set -e

echo "--- 開始佈署 RC Car 控制系統 ---"

# 1. 更新並安裝系統依賴
echo "安裝系統依賴項目..."
sudo apt-get update
sudo apt-get install -y \
    ros-humble-joy \
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    python3-pip \
    python3-smbus2 \
    v4l-utils

# 2. 設定 I2C 權限
echo "設定 I2C 存取權限..."
sudo usermod -aG i2c $USER

# 3. 建置工作區
echo "建置 ROS 2 工作區..."
cd ../..
colcon build --packages-select rc_car_control

echo "--- 佈署完成！ ---"
echo "請重新登入或執行 'source ~/ros2_ws/install/setup.bash' 以套用設定。"
