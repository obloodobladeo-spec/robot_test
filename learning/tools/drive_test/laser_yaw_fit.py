#!/usr/bin/env python3
"""저장한 원본 스캔(npz)으로 LiDAR 장착 yaw를 바꿔 가며 ICP를 다시 계산 → IMU 기준 옆밀림과 가장 잘 맞는 yaw를 찾는다.

usage: laser_yaw_fit.py <lidar_shuttle 로그 접두사> [...]
"""
import glob
import math
import sys

import numpy as np

sys.argv_saved = sys.argv[1:]
sys.argv = [sys.argv[0], '1', '0.1', '/tmp/_fit.log']
import lidar_shuttle as L  # noqa: E402

rows = []
for prefix in sys.argv_saved:
    res = {}
    for line in open(prefix):
        p = line.split()
        if len(p) > 2 and p[1] == 'RESULT':
            res[(p[2], int(p[3]))] = dict(e_fwd=float(p[7]), e_lat=float(p[8]), i_yaw=float(p[10]), lat_imu=float(p[14]))
    for f in sorted(glob.glob(prefix + '_*.npz')):
        tag = f[len(prefix) + 1:-4]
        key = (tag[:3], int(tag[3:]))
        if key in res:
            z = np.load(f)
            rows.append((key, z['r0'], z['r1'], res[key]))


def to_base(raw, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([L.LASER_X + c * raw[:, 0] - s * raw[:, 1], s * raw[:, 0] + c * raw[:, 1]], axis=1)


print('구간 %d개' % len(rows))
best = None
for d_deg in np.arange(-6.0, 2.01, 0.5):
    yaw = math.pi + math.radians(d_deg)
    err = []
    for key, r0, r1, m in rows:
        R, t, rms, used = L.icp(to_base(r1, yaw), to_base(r0, yaw), init=(m['e_fwd'], m['e_lat'], m['i_yaw']))
        err.append(t[1] - m['lat_imu'])
    e = np.array(err)
    print('  장착 yaw %+5.1f°(=%.1f°): LiDAR 옆 − IMU 옆 평균 %+5.1f mm, |오차| 평균 %.1f mm' % (
        d_deg, 180 + d_deg, e.mean() * 1000, np.abs(e).mean() * 1000))
    if best is None or np.abs(e).mean() < best[1]:
        best = (d_deg, np.abs(e).mean())
print('가장 잘 맞는 보정: %+.1f° (laser yaw %.1f°)' % (best[0], 180 + best[0]))
