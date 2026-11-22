#!/usr/bin/env python3
"""/scan 점을 TF(base_link<-laser)로 변환해, 로봇 기준 앞/왼/오른/뒤 방향의 가장 가까운 물체 거리를 출력.

usage: scan_direction.py [초]   (ROS_DOMAIN_ID를 로봇과 맞출 것)
앞(+x)에 물체를 두었을 때 '앞'에서 가장 가까운 값이 나오면 laser 방향(yaw)이 맞다.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
# 차체 영역(base_link 기준 x -0.05~0.20, y ±0.15) 안의 점은 자기 몸체로 보고 제외
BODY = (-0.05, 0.20, -0.15, 0.15)


class Peek(Node):
    def __init__(self):
        super().__init__('scan_direction')
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.scans = []
        self.create_subscription(LaserScan, '/scan', self.scans.append, 5)


def main():
    rclpy.init()
    n = Peek()
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
    tf = n.buf.lookup_transform('base_link', 'laser', rclpy.time.Time())
    tx, ty = tf.transform.translation.x, tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    print('TF base_link<-laser: x %.3f y %.3f yaw %.1f°  (스캔 %d개)' % (tx, ty, math.degrees(yaw), len(n.scans)))
    sectors = {'앞(+x)': 0, '왼쪽(+y)': 90, '뒤(-x)': 180, '오른쪽(-y)': -90}
    best = {k: (99, 0, 0) for k in sectors}
    for m in n.scans[-5:]:
        for i, r in enumerate(m.ranges):
            if not (math.isfinite(r) and m.range_min < r < m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            x = tx + lx * math.cos(yaw) - ly * math.sin(yaw)
            y = ty + lx * math.sin(yaw) + ly * math.cos(yaw)
            if BODY[0] < x < BODY[1] and BODY[2] < y < BODY[3]:
                continue
            ang = math.degrees(math.atan2(y, x))
            d = math.hypot(x, y)
            for k, c in sectors.items():
                if abs((ang - c + 180) % 360 - 180) <= 20 and d < best[k][0]:
                    best[k] = (d, x, y)
    for k, (d, x, y) in best.items():
        print('  %-9s ±20° 가장 가까운 점: %s' % (k, '%.2f m (x %+.2f, y %+.2f)' % (d, x, y) if d < 99 else '없음'))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
