#!/usr/bin/env python3
"""방향 정렬 후 30 cm씩 이어서 전진(되돌아오지 않음). 누적 옆밀림을 IMU·엔코더 기준으로 계산.

usage: segment_run.py <구간 수> <log> [속도 m/s]
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

NSEG = int(sys.argv[1])
out = open(sys.argv[2], 'w')
V = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
DIST = 0.30


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Seg(Node):
    def __init__(self):
        super().__init__('segment_run')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
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
        self.log('ODOM', *self.odom)

    def on_imu(self, m):
        q = m.orientation
        self.imu_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.log('IMU', self.imu_yaw, m.angular_velocity.z)

    def tick(self):
        t = Twist()
        t.linear.x = self.v
        t.angular.z = self.w
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Seg()

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

    try:
        until(lambda: n.odom is not None and n.imu_yaw is not None, 10)
        stop(1.0)
        yaw0 = n.imu_yaw
        n.log('HOME', *n.odom, yaw0)
        for k in range(1, NSEG + 1):
            if k > 1:
                align(yaw0)
            n.log('MARK', 'SEG_START', k, *n.odom, n.imu_yaw, math.degrees(wrap(n.imu_yaw - yaw0)))
            sx, sy, _ = n.odom
            n.v = V
            until(lambda: math.hypot(n.odom[0] - sx, n.odom[1] - sy) >= DIST, 15)
            stop(2.0)
            n.log('MARK', 'SEG_END', k, *n.odom, n.imu_yaw, math.degrees(wrap(n.imu_yaw - yaw0)))
    finally:
        for _ in range(5):
            n.pub.publish(Twist())
            time.sleep(0.05)
        n.destroy_node()
        rclpy.shutdown()

    # 누적 옆밀림: 시작 방향(IMU yaw0) 기준 좌표계, + 왼쪽
    rows = [line.split() for line in open(sys.argv[2])]
    od = [(float(r[0]), float(r[2]), float(r[3]), float(r[4])) for r in rows if r[1] == 'ODOM']
    im = [(float(r[0]), float(r[2])) for r in rows if r[1] == 'IMU']
    home = [r for r in rows if r[1] == 'HOME'][0]
    h0 = float(home[4])
    yaw0 = float(home[5])
    t_home = float(home[0])
    od = [x for x in od if x[0] >= t_home]
    fx = ly_imu = 0.0
    for a, b in zip(od, od[1:]):
        ds = math.hypot(b[1] - a[1], b[2] - a[2])
        iy = min(im, key=lambda x: abs(x[0] - b[0]))[1]
        fx += ds * math.cos(wrap(iy - yaw0))
        ly_imu += ds * math.sin(wrap(iy - yaw0))
    dx = od[-1][1] - od[0][1]
    dy = od[-1][2] - od[0][2]
    ly_enc = -dx * math.sin(h0) + dy * math.cos(h0)
    fwd_enc = dx * math.cos(h0) + dy * math.sin(h0)
    print('전진 %.0f mm | 옆밀림 IMU 방향 기준 %+.1f mm, 엔코더 기준 %+.1f mm (+ 왼쪽 / - 오른쪽)' % (fwd_enc * 1000, ly_imu * 1000, ly_enc * 1000))
    for r in rows:
        if r[1] == 'MARK':
            print('  %s %s: IMU 방향 %+.2f°' % (r[2], r[3], float(r[-1])))


if __name__ == '__main__':
    main()
