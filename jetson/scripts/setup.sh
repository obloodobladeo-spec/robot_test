#!/usr/bin/env bash
set -euo pipefail
robot_distro="${1:-humble}"
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "/opt/ros/$robot_distro/setup.bash" ]]; then
    echo "ROS 2 $robot_distro 설치를 먼저 확인하세요: /opt/ros/$robot_distro/setup.bash" >&2
    exit 1
fi
echo "Jetson ROS 2: $robot_distro"
sudo apt-get update
sudo apt-get install -y python3-serial python3-colcon-common-extensions \
    "ros-$robot_distro-geometry-msgs" "ros-$robot_distro-std-msgs" \
    "ros-$robot_distro-launch-ros" "ros-$robot_distro-ament-index-python"
sudo usermod -aG dialout "$(id -un)"
echo "dialout 그룹 적용을 위해 로그아웃 후 SSH에 다시 접속하세요."
bash "$robot_root/scripts/build.sh" "$robot_distro"
