#!/bin/bash
set -e

echo "--- 開始佈署 RC Car 控制系統 (含開機自啟) ---"

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

# 4. 配置系統服務 (開機自啟)
echo "配置系統自啟服務..."
# 確保腳本權限
chmod +x src/rc_car_control/scripts/ros2_start.sh

# 停用舊服務
sudo systemctl stop donkeycar.service || true
sudo systemctl disable donkeycar.service || true

# 安裝新服務
sudo cp src/rc_car_control/ros2_ws.service /etc/systemd/system/ros2_ws.service
sudo systemctl daemon-reload
sudo systemctl enable ros2_ws.service

echo "--- 佈署完成！ ---"
echo "您可以重啟 Jetson 測試自動啟動，或執行以下指令立即啟動："
echo "sudo systemctl start ros2_ws.service"
