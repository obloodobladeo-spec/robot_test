#!/bin/bash
# usage: run_set.sh <tag> <speeds...>
source /opt/ros/humble/setup.bash; source ~/robot_test/learning/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=20
cd ~/robot_test/learning/tools/drive_test
tag=$1; shift
for v in "$@"; do echo "=== $tag 속도 $v ($(date +%T))"; python3 -u lidar_shuttle.py 3 0.5 /tmp/ls/${tag}_v$v $v; done
echo DONE $(date +%T)
