#!/bin/bash
source /opt/ros/humble/setup.bash; source ~/robot_test/learning/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=20
cd ~/robot_test/learning/tools/drive_test
for v in 0.10 0.15 0.20; do echo "=== 속도 $v m/s ($(date +%T))"; python3 lidar_shuttle.py 3 0.5 /tmp/ls/v$v $v; done
echo DONE $(date +%T)
