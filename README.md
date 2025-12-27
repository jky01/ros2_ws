# Jetson Orin Nano RC Car Control (ROS 2 Humble)

這是一個專為 Jetson Orin Nano 設計的 ROS 2 RC 車輛控制系統，完美復刻並優化了 Donkeycar 的操控體驗。

## 特色功能

- **整合式 Donkeycar 控制**: 單一節點處理手把輸入與馬達輸出，延遲極低。
- **Xbox 手把完美對應**:
  - **左板機 (LT)**: 前進加速。
  - **右板機 (RT)**: 後退加速。
  - **左搖桿 (左右)**: 轉向控制。
  - **RB 鍵 (Index 7)**: 安全鎖定開關 (Toggle Lock/Unlock)。
- **硬體加速影像**: 使用 GStreamer (`nvarguscamerasrc`) 調用 IMX219 鏡頭，提供低延遲、硬體去馬賽克後的 RGB 影像。
- **啟動安全保護**: 自動校準板機初始值，防止啟動或解鎖時發生「暴衝」。

## 硬體需求

- **平台**: NVIDIA Jetson Orin Nano
- **鏡頭**: IMX219 (CSI 介面)
- **馬達驅動器**: PCA9685 (I2C Bus 7)
- **控制器**: Xbox One / Series 無線手把

## 快速佈署說明

如果您重置了系統，請執行以下指令進行快速安裝：

### 1. 安裝與建置

```bash
# 進入您的工作區 (若無請建立)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 複製本專案 (請先將本專案上傳至您的 GitHub)
# git clone [您的 GitHub 網址] rc_car_control

# 執行自動化安裝腳本
cd rc_car_control
chmod +x setup_car.sh
./setup_car.sh
```

### 2. 啟動控制程式

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rc_car_control car_teleop.launch.py
```

### 3. 查看畫面

```bash
ros2 run rqt_image_view rqt_image_view
```

## 專案結構

- `rc_car_control/donkey_control.py`: 核心控制與影像發送節點。
- `launch/car_teleop.launch.py`: 整合啟動文件。
- `setup_car.sh`: 環境自動化配置腳本。

