#!/usr/bin/env python3
"""토픽 도착 간격 통계.  usage: topic_jitter.py <topic> <초>  (LaserScan/Odometry/Imu 등 아무 타입: rosidl 자동)"""
import statistics as st
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros2topic.api import get_msg_class

TOPIC, DUR = sys.argv[1], float(sys.argv[2])
rclpy.init()
n = Node('topic_jitter')
cls = None
t0 = time.time()
while cls is None and time.time() - t0 < 5:
    cls = get_msg_class(n, TOPIC, include_hidden_topics=True)
    rclpy.spin_once(n, timeout_sec=0.2)
arr = []
n.create_subscription(cls, TOPIC, lambda m: arr.append(time.time()), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.02)
g = [(b - a) * 1000 for a, b in zip(arr, arr[1:])]
print('%s: %d개 %.1f Hz | 간격 평균 %.0f ms σ %.0f ms 최대 %.0f ms | 150ms 넘는 간격 %d건' % (
    TOPIC, len(arr), (len(arr) - 1) / (arr[-1] - arr[0]), st.mean(g), st.pstdev(g), max(g), sum(x > 150 for x in g)))
