#!/usr/bin/env python3
"""lidar_shuttle 로그들의 주행 중 흔들림 비교.  usage: wobble_compare.py <log> [...]"""
import math
import statistics as st
import sys


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


for fn in sys.argv[1:]:
    rows = [ln.split() for ln in open(fn)]
    marks = [(float(r[0]), r[2]) for r in rows if r[1] == 'MARK']
    res = [r for r in rows if r[1] == 'RESULT']
    imu = [(float(r[0]), float(r[2]), float(r[3])) for r in rows if r[1] == 'IMU']
    out = {'FWD': [], 'BWD': []}
    starts = [(t, n[:3]) for t, n in marks]
    ends = [float(r[0]) for r in res]
    for (t0, nm), t1 in zip(starts, ends):
        seg = [x for x in imu if t0 + 1.5 <= x[0] <= t1 - 5.0]   # 끝: 정지 2.5 s + 스냅샷 약 0.6 s + 감속
        if len(seg) < 8:
            continue
        y0 = seg[0][1]
        ys = [math.degrees(wrap(x[1] - y0)) for x in seg]
        n = len(ys)
        tx = list(range(n))
        mt, my = st.mean(tx), st.mean(ys)
        b = sum((a - mt) * (c - my) for a, c in zip(tx, ys)) / sum((a - mt) ** 2 for a in tx)
        resid = [c - (my + b * (a - mt)) for a, c in zip(tx, ys)]
        out[nm].append((max(resid) - min(resid), st.pstdev([x[2] * 1000 for x in seg]), n))
    line = []
    for nm in ('FWD', 'BWD'):
        r = out[nm]
        if r:
            line.append('%s yaw진폭 %.2f° gz σ %4.1f (n=%d, %d샘플)' % ('전진' if nm == 'FWD' else '후진', st.mean(x[0] for x in r), st.mean(x[1] for x in r), len(r), r[0][2]))
    print('%-22s %s' % (fn.split('/')[-1], ' | '.join(line)))
