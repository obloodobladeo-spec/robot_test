#!/usr/bin/env python3
"""직선 거리 실측 비교: 엔코더 odom 이동거리가 목표에 닿으면 정지(끝 1 cm 전부터 감속 없음, 정지 후 관성 포함 기록).

usage: distance_run.py <목표 m> <log> [속도 m/s, 기본 0.05]
출력: 정지 명령 시·완전 정지 후 odom 직선거리(hypot), 이동 방향 성분, 옆밀림, IMU 방향 변화
실측: 출발 전·도착 후 구동축 가운데(바퀴 중심 연결선 중간) 바닥 표시 사이 직선거리
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

TARGET = float(sys.argv[1])
out = open(sys.argv[2], 'w')
V = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Run(Node):
    def __init__(self):
        super().__init__('distance_run')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.odom = None
        self.yaw = None
        self.v = 0.0
        self.create_timer(0.1, self.tick)

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w))
        out.write('%.3f ODOM %.5f %.5f %.5f %.4f\n' % (time.time(), *self.odom, m.twist.twist.linear.x))

    def on_imu(self, m):
        q = m.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        out.write('%.3f IMU %.6f\n' % (time.time(), self.yaw))

    def tick(self):
        t = Twist()
        t.linear.x = self.v
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Run()

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    try:
        end = time.time() + 10
        while (n.odom is None or n.yaw is None) and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        spin(1.0)
        x0, y0, h0 = n.odom
        i0 = n.yaw

        def dist():
            return math.hypot(n.odom[0] - x0, n.odom[1] - y0)

        n.v = V
        t0 = time.time()
        while dist() < TARGET and time.time() - t0 < TARGET / V * 2 + 10:
            rclpy.spin_once(n, timeout_sec=0.02)
        n.v = 0.0
        d_stop = dist()
        spin(2.5)
        x1, y1, _ = n.odom
        dx, dy = x1 - x0, y1 - y0
        fwd = dx * math.cos(h0) + dy * math.sin(h0)
        lat = -dx * math.sin(h0) + dy * math.cos(h0)
        print('odom: 정지 명령 시 %.1f mm → 완전 정지 후 직선거리 %.1f mm (진행방향 %.1f, 옆밀림 %+.1f mm), IMU 방향 변화 %+.2f°, 주행 %.1f s' % (
            d_stop * 1000, math.hypot(dx, dy) * 1000, fwd * 1000, lat * 1000, math.degrees(wrap(n.yaw - i0)), time.time() - t0 - 2.5))
        out.write('%.3f RESULT %.5f %.5f %.5f %.6f\n' % (time.time(), d_stop, math.hypot(dx, dy), lat, wrap(n.yaw - i0)))
    finally:
        for _ in range(5):
            n.pub.publish(Twist())
            time.sleep(0.05)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
