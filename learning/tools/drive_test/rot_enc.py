#!/usr/bin/env python3
"""제자리 회전 시험: 엔코더(/odom) 누적 yaw가 목표에 닿을 때까지 회전하고 IMU 누적 yaw와 비교.

usage: rot_enc.py <목표 deg> <angular rad/s(부호=방향)> <log 파일>
실효 윤거 = 현재 WHEEL_TRACK_MM x (엔코더 회전량 / IMU 회전량)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

TARGET = math.radians(float(sys.argv[1]))
W = float(sys.argv[2])
out = open(sys.argv[3], 'w')


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RotEnc(Node):
    def __init__(self):
        super().__init__('rot_enc_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.e_last = None
        self.e_acc = 0.0
        self.i_last = None
        self.i_acc = 0.0
        self.done = False
        self.t_done = None
        self.create_timer(0.1, self.tick)

    def on_odom(self, m):
        y = 2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)
        if self.e_last is not None:
            self.e_acc += wrap(y - self.e_last)
        self.e_last = y
        out.write('%.3f ENC %.5f\n' % (time.time(), self.e_acc))

    def on_imu(self, m):
        q = m.orientation
        y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.i_last is not None:
            self.i_acc += wrap(y - self.i_last)
        self.i_last = y
        out.write('%.3f IMU %.5f\n' % (time.time(), self.i_acc))

    def tick(self):
        t = Twist()
        if not self.done and self.e_last is not None and abs(self.e_acc) >= TARGET:
            self.done = True
            self.t_done = time.time()
            out.write('%.3f STOP %.5f %.5f\n' % (time.time(), self.e_acc, self.i_acc))
        if not self.done and self.e_last is not None:
            t.angular.z = W
        self.pub.publish(t)


def main():
    rclpy.init()
    n = RotEnc()
    t0 = time.time()
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.done and time.time() - n.t_done > 3:
            break
        if time.time() - t0 > 90:
            print('TIMEOUT')
            break
    for _ in range(3):
        n.pub.publish(Twist())
        time.sleep(0.05)
    print('final enc %.2f imu %.2f deg' % (math.degrees(n.e_acc), math.degrees(n.i_acc)))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
