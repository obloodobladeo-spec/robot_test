#!/usr/bin/env python3
"""shuttle_run.py 로그 요약 한 줄씩 (설정 비교용). 비틀거림 지표 포함.  usage: shuttle_brief.py <log> [라벨]"""
import math
import statistics as st
import sys

rows = [line.split() for line in open(sys.argv[1])]
label = sys.argv[2] if len(sys.argv) > 2 else sys.argv[1]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


marks = [(float(r[0]), r[2], int(r[3])) for r in rows if r[1] == 'MARK']
OD = [(float(r[0]), float(r[2]), float(r[3]), float(r[4])) for r in rows if r[1] == 'ODOM']
IM = [(float(r[0]), *map(float, r[2:8])) for r in rows if r[1] == 'IMU']
WS = [(float(r[0]), float(r[2]) * 1000, float(r[3]) * 1000) for r in rows if r[1] == 'WS']
done = {m[2] for m in marks if m[1] == 'BWD_END'}
N = max(k for k in done if all(j in done for j in range(1, k + 1)))


def t_of(name, k):
    return [m[0] for m in marks if m[1] == name and m[2] == k][0]


out = {}
for d in ('FWD', 'BWD'):
    R = []
    for k in range(1, N + 1):
        t0, t1 = t_of(d + '_START', k), t_of(d + '_END', k)
        im = [x for x in IM if t0 <= x[0] <= t1]
        o = [x for x in OD if t0 <= x[0] <= t1]
        y0 = im[0][1]
        dyi = math.degrees(wrap(im[-1][1] - y0))
        dye = math.degrees(wrap(o[-1][3] - o[0][3]))
        dmax = max((math.degrees(wrap(x[1] - y0)) for x in im), key=abs)
        # 정상 주행 구간(출발 1.5 s 후 ~ 정지 2.3 s 전)의 비틀거림
        ss = [x for x in im if t0 + 1.5 <= x[0] <= t1 - 2.3]
        gz = [x[6] * 1000 for x in ss]
        ys = [math.degrees(wrap(x[1] - y0)) for x in ss]
        # 선형 추세 제거한 yaw 진폭
        n = len(ys)
        tx = list(range(n))
        mt, my = st.mean(tx), st.mean(ys)
        b = sum((a - mt) * (c - my) for a, c in zip(tx, ys)) / sum((a - mt) ** 2 for a in tx)
        res = [c - (my + b * (a - mt)) for a, c in zip(tx, ys)]
        rp = [x[2] for x in ss]
        roll_std = st.pstdev([math.degrees(v) for v in rp])
        R.append((dyi, dye - dyi, dmax, st.pstdev(gz), max(res) - min(res), roll_std))
    out[d] = R
    print('%-18s %s n=%d | IMU최종 %+5.2f°±%.2f | 엔코더-IMU %+5.2f° | 최대꺾임 %+5.2f° | 비틀림: gz σ %4.1f mrad/s, yaw 진폭 %.2f°, roll σ %.2f°' % (
        label, '전진' if d == 'FWD' else '후진', N,
        st.mean(x[0] for x in R), st.pstdev([x[0] for x in R]), st.mean(x[1] for x in R),
        st.mean(x[2] for x in R), st.mean(x[3] for x in R), st.mean(x[4] for x in R), st.mean(x[5] for x in R)))
