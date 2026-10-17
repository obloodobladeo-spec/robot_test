#!/usr/bin/env bash
set -euo pipefail
robot_distro="${ROS_DISTRO:-humble}"
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "/opt/ros/$robot_distro/setup.bash"
source "$robot_root/robot_ws/install/setup.bash"
set -u
exec ros2 run robot_test teleop_keyboard "$@"
