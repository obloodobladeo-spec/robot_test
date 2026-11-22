#!/usr/bin/env python3
"""LiDAR 기준 직진 방향 시험: [전진 D -> 후진 D] x N, 구간마다 스캔 매칭(ICP)으로 실제 회전·옆밀림을 측정.

안전: 출발 전 진행 방향 빈 공간 >= D + 0.30 m 이어야 출발, 주행 중 진행 방향 0.25 m 안에 물체가 들어오면 정지.
usage: lidar_shuttle.py <왕복 N> <거리 m> <log> [속도 m/s=0.05] [각속도 cmd(0.001이면 heading hold 끔)=0]
ICP 결과는 '출발 자세 기준' 로봇 이동(dx 앞, dy 왼쪽, dyaw 반시계 +).
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import Imu, LaserScan

N = int(sys.argv[1])
D = float(sys.argv[2])
out = open(sys.argv[3], 'w')
V = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05
WCMD = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
LASER_X, LASER_YAW = 0.128, math.pi          # URDF base_link -> laser
BODY = (-0.06, 0.21, -0.16, 0.16)           # 몸체 영역 (base_link)
HALF_W = 0.13                               # 진행 경로 폭 절반 (차체 폭 0.16/2 + 여유)
STOP_GAP = 0.15


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Node_(Node):
    def __init__(self):
        super().__init__('lidar_shuttle')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.odom = None
        self.yaw = None
        self.pts = None
        self.scan_t = 0.0
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
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.log('IMU', self.yaw, m.angular_velocity.z)

    def on_scan(self, m):
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, 8.0))
        lx, ly = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        self.raw = np.stack([lx, ly], axis=1)
        c, s = math.cos(LASER_YAW), math.sin(LASER_YAW)
        x = LASER_X + c * lx - s * ly
        y = s * lx + c * ly
        body = (x > BODY[0]) & (x < BODY[1]) & (y > BODY[2]) & (y < BODY[3])
        self.pts = np.stack([x[~body], y[~body]], axis=1)
        self.raw = self.raw[~body]
        self.scan_t = time.time()

    def tick(self):
        t = Twist()
        t.linear.x = self.v
        t.angular.z = self.w
        self.pub.publish(t)

    def clearance(self, direction):
        """진행 방향(+1 앞, -1 뒤) 경로 폭 안의 가장 가까운 물체까지 거리(몸체 끝 기준)."""
        if self.pts is None:
            return 0.0
        x, y = self.pts[:, 0], self.pts[:, 1]
        lane = np.abs(y) < HALF_W
        if direction > 0:
            d = x[lane & (x > 0)] - BODY[1]
        else:
            d = -x[lane & (x < 0)] + BODY[0]
        return float(d.min()) if d.size else 9.0


def icp(src, dst, iters=60, init=(0.0, 0.0, 0.0)):
    """src를 dst에 맞추는 2D 강체 변환 (R, t). src = 나중 스캔, dst = 처음 스캔 (같은 base_link 정의).
    init = (dx, dy, dyaw): 엔코더 거리·IMU 방향으로 만든 출발값. 큰 이동에서 엉뚱한 짝을 막는다."""
    tree = cKDTree(dst)
    c0, s0 = math.cos(init[2]), math.sin(init[2])
    R = np.array([[c0, -s0], [s0, c0]])
    t = np.array([init[0], init[1]])
    cur = src @ R.T + t
    for _ in range(iters):
        d, idx = tree.query(cur, distance_upper_bound=0.15)
        m = np.isfinite(d)
        if m.sum() < 30:
            break
        keep = d[m] < max(0.02, np.percentile(d[m], 80))
        a, b = cur[m][keep], dst[idx[m]][keep]
        ma, mb = a.mean(0), b.mean(0)
        H = (a - ma).T @ (b - mb)
        U, _, Vt = np.linalg.svd(H)
        Ri = Vt.T @ U.T
        if np.linalg.det(Ri) < 0:
            Vt[1] *= -1
            Ri = Vt.T @ U.T
        ti = mb - Ri @ ma
        cur = cur @ Ri.T + ti
        R, t = Ri @ R, Ri @ t + ti
        if np.linalg.norm(ti) < 1e-5 and abs(math.atan2(Ri[1, 0], Ri[0, 0])) < 1e-6:
            break
    d, _ = tree.query(cur, distance_upper_bound=0.15)
    inl = d[np.isfinite(d)]
    rms = float(np.sqrt(np.mean(np.minimum(inl, 0.05) ** 2)))  # 짝지은 점 중 5 cm 이하 기준
    return R, t, rms, int(np.isfinite(d).sum())


def main():
    rclpy.init()
    n = Node_()

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def snapshot():
        """정지 상태 스캔 5장을 모아 하나로 (점 수 늘리기)."""
        pts, raw = [], []
        last = n.scan_t
        while len(pts) < 5:
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.scan_t != last:
                last = n.scan_t
                pts.append(n.pts)
                raw.append(n.raw)
        snapshot.raw = np.vstack(raw)
        return np.vstack(pts)

    def stop(sec):
        n.v = 0.0
        n.w = 0.0
        spin(sec)

    results = []
    try:
        end = time.time() + 10
        while (n.odom is None or n.yaw is None or n.pts is None) and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        if n.pts is None:
            raise SystemExit('no /scan')
        stop(1.0)
        for k in range(1, N + 1):
            for direction, name in ((1, 'FWD'), (-1, 'BWD')):
                free = n.clearance(direction)
                if free < D + STOP_GAP + 1.0 * V:
                    print("%s %d: 진행 방향 빈 공간 %.2f m < %.2f m → 중단" % (name, k, free, D + STOP_GAP + 1.0 * V))
                    n.log('ABORT', name, k, free)
                    return
                s0 = snapshot()
                r0 = snapshot.raw
                o0, y0 = n.odom, n.yaw
                path = []
                n.log('MARK', name + '_START', k, *o0, y0, free)
                n.v = V * direction
                n.w = WCMD
                t0 = time.time()
                blocked = False
                while math.hypot(n.odom[0] - o0[0], n.odom[1] - o0[1]) < D and time.time() - t0 < D / V * 2 + 10:
                    rclpy.spin_once(n, timeout_sec=0.02)
                    path.append((n.odom[0], n.odom[1], n.yaw))
                    if n.clearance(direction) < STOP_GAP + 0.5 * V:
                        blocked = True
                        break
                t_end = time.time() + 2.5
                while time.time() < t_end:
                    n.v = 0.0
                    n.w = 0.0
                    rclpy.spin_once(n, timeout_sec=0.02)
                    path.append((n.odom[0], n.odom[1], n.yaw))
                s1 = snapshot()
                r1 = snapshot.raw
                # IMU 방향으로 적분한 옆밀림 (엔코더 거리 + IMU 방향)
                lat_imu = 0.0
                for (ax, ay, _), (bx, by, byaw) in zip(path, path[1:]):
                    ds = math.hypot(bx - ax, by - ay) * (1 if direction > 0 else -1)
                    lat_imu += ds * math.sin(wrap(byaw - y0))
                np.savez_compressed(sys.argv[3] + '_%s%d.npz' % (name, k), r0=r0, r1=r1)
                o1, y1 = n.odom, n.yaw
                dx, dy = o1[0] - o0[0], o1[1] - o0[1]
                g_fwd = dx * math.cos(o0[2]) + dy * math.sin(o0[2])
                g_lat = -dx * math.sin(o0[2]) + dy * math.cos(o0[2])
                R, t, rms, used = icp(s1, s0, init=(g_fwd, g_lat, wrap(y1 - y0)))
                # s1(나중 base_link)의 점을 s0(처음 base_link)로 옮기는 변환 = 로봇의 이동
                l_yaw = math.atan2(R[1, 0], R[0, 0])
                l_dx, l_dy = t[0], t[1]
                dx, dy = o1[0] - o0[0], o1[1] - o0[1]
                e_fwd = dx * math.cos(o0[2]) + dy * math.sin(o0[2])
                e_lat = -dx * math.sin(o0[2]) + dy * math.cos(o0[2])
                e_yaw = wrap(o1[2] - o0[2])
                i_yaw = wrap(y1 - y0)
                row = (name, k, l_dx, l_dy, l_yaw, e_fwd, e_lat, e_yaw, i_yaw, rms, used, blocked, lat_imu)
                results.append(row)
                n.log('RESULT', *row)
                print('%s %d: LiDAR 이동 %+6.1f mm 옆 %+5.1f mm 회전 %+5.2f° | 엔코더 %+6.1f / %+5.1f / %+5.2f° | IMU %+5.2f° 옆(IMU) %+5.1f mm | rms %.1f mm%s' % (
                    name, k, l_dx * 1000, l_dy * 1000, math.degrees(l_yaw), e_fwd * 1000, e_lat * 1000,
                    math.degrees(e_yaw), math.degrees(i_yaw), lat_imu * 1000, rms * 1000, ' [장애물 정지]' if blocked else ''))
    finally:
        for _ in range(5):
            n.pub.publish(Twist())
            time.sleep(0.05)
        if results:
            for nm in ('FWD', 'BWD'):
                r = [x for x in results if x[0] == nm]
                if r:
                    print('%s 평균: LiDAR 회전 %+.2f° (±%.2f), 옆 %+.1f mm | IMU %+.2f° | 엔코더 %+.2f° | IMU−LiDAR %+.2f°' % (
                        nm, np.degrees(np.mean([x[4] for x in r])), np.degrees(np.std([x[4] for x in r])),
                        1000 * np.mean([x[3] for x in r]), np.degrees(np.mean([x[8] for x in r])),
                        np.degrees(np.mean([x[7] for x in r])), np.degrees(np.mean([x[8] - x[4] for x in r]))))
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
