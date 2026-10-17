#!/usr/bin/env bash
set -euo pipefail
robot_distro="${1:-humble}"
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "/opt/ros/$robot_distro/setup.bash"
set -u
cd "$robot_root/robot_ws"
colcon build --symlink-install --packages-select robot_test
echo "빌드 완료. 설정 파일: $robot_root/robot_ws/src/robot_test/config/robot.yaml"
