#!/usr/bin/env python3
"""정지 상태 IMU roll/pitch/yaw 평균 (수평 확인).  usage: level_check.py [초]"""
import math
import statistics as st
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
vals = []


class L(Node):
    def __init__(self):
        super().__init__('level_check')
        self.create_subscription(Imu, '/imu/data', self.cb, 50)

    def cb(self, m):
        q = m.orientation
        r = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        p = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        y = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        vals.append((r, p, y, m.angular_velocity.z))


rclpy.init()
n = L()
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
n.destroy_node()
rclpy.shutdown()
print('n=%d roll %+.2f°±%.3f pitch %+.2f°±%.3f yaw %+.2f° gz %.1f mrad/s' % (
    len(vals), st.mean(v[0] for v in vals), st.pstdev([v[0] for v in vals]),
    st.mean(v[1] for v in vals), st.pstdev([v[1] for v in vals]), st.mean(v[2] for v in vals),
    st.mean(v[3] for v in vals) * 1000))
