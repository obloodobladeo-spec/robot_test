#!/usr/bin/env python3
"""정지 상태에서 스캔 5장 묶음 2개를 ICP로 비교 → 0에 가까워야 함. 각 방향 빈 공간도 출력."""
import math
import sys
import time

import numpy as np
import rclpy

sys.argv = [sys.argv[0], '1', '0.1', '/tmp/icp_static.log']
import lidar_shuttle as L  # noqa: E402

rclpy.init()
n = L.Node_()
t0 = time.time()
while n.pts is None and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.05)


def snap():
    pts, last = [], n.scan_t
    while len(pts) < 5:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.scan_t != last:
            last = n.scan_t
            pts.append(n.pts)
    return np.vstack(pts)


a = snap()
time.sleep(1.0)
b = snap()
for k in range(3):
    R, t, rms, used = L.icp(b, a)
    print('정지 ICP %d: dx %+.1f mm dy %+.1f mm yaw %+.3f° rms %.1f mm (%d점)' % (
        k + 1, t[0] * 1000, t[1] * 1000, math.degrees(math.atan2(R[1, 0], R[0, 0])), rms * 1000, used))
    a = b
    b = snap()
print('빈 공간: 앞 %.2f m, 뒤 %.2f m (몸체 끝 기준, 경로 폭 ±%.2f m)' % (n.clearance(1), n.clearance(-1), L.HALF_W))
# 스캔 점이 어느 방향에 얼마나 있는지 (벽 구조 파악)
p = n.pts
ang = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
for lo, hi, name in ((-30, 30, '앞'), (60, 120, '왼쪽'), (150, 210, '뒤'), (-120, -60, '오른쪽')):
    a2 = ang % 360 if lo >= 150 else ang
    sel = (a2 >= lo % 360 if lo >= 150 else a2 >= lo) & (a2 <= hi % 360 if lo >= 150 else a2 <= hi)
    d = np.hypot(p[sel, 0], p[sel, 1])
    print('  %s: 점 %d, 거리 %.2f~%.2f m' % (name, sel.sum(), d.min() if d.size else 0, d.max() if d.size else 0))
n.destroy_node()
rclpy.shutdown()
