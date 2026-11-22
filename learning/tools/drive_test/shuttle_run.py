#!/usr/bin/env python3
"""전진·후진 왕복 시험: [IMU 방향 정렬 -> 전진 30 cm -> 정렬 -> 후진 30 cm] x N.

usage: shuttle_run.py <왕복 횟수> <log> [속도 m/s] [거리 m] [각속도 rad/s, 0.001이면 heading hold 끔]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

N = int(sys.argv[1])
out = open(sys.argv[2], 'w')
V = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
DIST = float(sys.argv[4]) if len(sys.argv) > 4 else 0.30
WCMD = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Shuttle(Node):
    def __init__(self):
        super().__init__('shuttle_run')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.create_subscription(Float64MultiArray, '/wheel_speeds', self.on_ws, 50)
        self.odom = None
        self.imu_yaw = None
        self.v = 0.0
        self.w = 0.0
        self.create_timer(0.1, self.tick)

    def log(self, *a):
        out.write('%.3f ' % time.time() + ' '.join(str(x) for x in a) + '\n')

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w))
        self.log('ODOM', *self.odom, m.twist.twist.linear.x, m.twist.twist.angular.z)

    def on_imu(self, m):
        q = m.orientation
        self.imu_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        r = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        p = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        g = m.angular_velocity
        self.log('IMU', self.imu_yaw, r, p, g.x, g.y, g.z)

    def on_ws(self, m):
        self.log('WS', *m.data)

    def tick(self):
        t = Twist()
        t.linear.x = self.v
        t.angular.z = self.w
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Shuttle()

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def until(cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)
            if cond():
                return True
        return False

    def stop(sec):
        n.v = 0.0
        n.w = 0.0
        spin(sec)

    def align(yaw0):
        end = time.time() + 15
        while abs(wrap(yaw0 - n.imu_yaw)) > math.radians(0.7) and time.time() < end:
            e = wrap(yaw0 - n.imu_yaw)
            w = max(-0.3, min(0.3, 2.0 * e))
            n.v = 0.0
            n.w = math.copysign(max(abs(w), 0.15), w)
            spin(0.1)
        stop(1.5)

    def segment(k, name, v, yaw0):
        align(yaw0)
        n.log('MARK', name + '_START', k, *n.odom, n.imu_yaw)
        sx, sy, _ = n.odom
        n.v = v
        n.w = WCMD
        ok = until(lambda: math.hypot(n.odom[0] - sx, n.odom[1] - sy) >= DIST, 15)
        stop(2.0)
        n.log('MARK', name + '_END', k, ok, *n.odom, n.imu_yaw)

    try:
        if not until(lambda: n.odom is not None and n.imu_yaw is not None, 10):
            raise SystemExit('no data')
        stop(1.0)
        yaw0 = n.imu_yaw
        n.log('HOME', *n.odom, yaw0)
        for k in range(1, N + 1):
            segment(k, 'FWD', V, yaw0)
            segment(k, 'BWD', -V, yaw0)
    finally:
        for _ in range(5):
            n.pub.publish(Twist())
            time.sleep(0.05)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
