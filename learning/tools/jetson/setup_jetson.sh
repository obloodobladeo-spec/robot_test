#!/bin/bash
# Jetson(Ubuntu 22.04 + ROS 2 Humble) 첫 설정. sudo 권한 필요.
# usage: git clone -b test/imu-drive-tuning https://github.com/obloodobladeo-spec/robot_test.git
#        bash robot_test/learning/tools/jetson/setup_jetson.sh
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=${WS:-$(cd "$HERE/../../ros2_ws" && pwd)}   # robot_test/learning/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-serial python3-colcon-common-extensions \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-robot-localization ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-tf2-tools
sudo usermod -aG dialout "$USER"          # 다시 로그인해야 적용
sudo cp "$HERE/99-robot-usb.rules" /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
cd "$WS"
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 1
echo "완료. 다시 로그인 후: ls -l /dev/stm32 /dev/lidar ; source $WS/install/setup.bash"
