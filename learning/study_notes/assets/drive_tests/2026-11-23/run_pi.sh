#!/bin/bash
source /opt/ros/humble/setup.bash; source ~/robot_test/learning/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=20
cd ~/robot_test/learning/tools/drive_test
for cfg in "p0 0.15 0.25" "pA 0.10 0.25" "pB 0.25 0.25" "pC 0.15 0.15"; do set -- $cfg
  python3 swd_param_ocd.py kp=$2 ki=$3 | tr " " "\n" | grep -E "^(L_kp|L_ki|R_kp|R_ki|L_circ|R_circ)=" | tr "\n" " "; echo
  echo "=== $1 kp=$2 ki=$3 ($(date +%T))"; python3 -u lidar_shuttle.py 3 0.5 /tmp/ls/$1_v0.10 0.10
done
python3 swd_param_ocd.py kp=0.15 ki=0.25 >/dev/null
echo DONE $(date +%T)
