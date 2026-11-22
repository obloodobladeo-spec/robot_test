#!/usr/bin/env python3
"""shuttle_run.py 로그 분석 (구간별 전진/후진).  usage: shuttle_analyze.py <log>"""
import math
import statistics as st
import sys

rows = [line.split() for line in open(sys.argv[1])]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


marks = [(float(r[0]), r[2], int(r[3])) for r in rows if r[1] == 'MARK']
OD = [(float(r[0]), float(r[2]), float(r[3]), float(r[4])) for r in rows if r[1] == 'ODOM']
IM = [(float(r[0]), *map(float, r[2:8])) for r in rows if r[1] == 'IMU']
WS = [(float(r[0]), float(r[2]) * 1000, float(r[3]) * 1000) for r in rows if r[1] == 'WS']
done = {m[2] for m in marks if m[1] == 'BWD_END'}
N = max(k for k in done if all(j in done for j in range(1, k + 1)))  # 끝까지 완료된 왕복만


def t_of(name, k):
    return [m[0] for m in marks if m[1] == name and m[2] == k][0]


def nearest(lst, t):
    return min(lst, key=lambda x: abs(x[0] - t))


res = {'FWD': [], 'BWD': []}
print('구간      | IMU Δyaw | 최대꺾임(IMU) | 엔코더Δyaw | 옆밀림 IMU기준 | 옆밀림 엔코더 | 최고속도 L/R | 출발30mm/s')
for k in range(1, N + 1):
    for d in ('FWD', 'BWD'):
        t0 = t_of(d + '_START', k)
        t1 = t_of(d + '_END', k)
        o = [x for x in OD if t0 <= x[0] <= t1]
        im = [x for x in IM if t0 <= x[0] <= t1]
        w = [x for x in WS if t0 <= x[0] <= t1]
        y0 = im[0][1]
        h0 = o[0][3]
        sgn = 1 if d == 'FWD' else -1
        li = 0.0
        for a, b in zip(o, o[1:]):
            ds = math.hypot(b[1] - a[1], b[2] - a[2])
            iy = nearest(im, b[0])[1]
            li += ds * math.sin(wrap(iy - y0))
        li *= sgn  # 후진은 이동 방향이 반대
        dx = o[-1][1] - o[0][1]
        dy = o[-1][2] - o[0][2]
        le = (-dx * math.sin(h0) + dy * math.cos(h0)) * sgn  # IMU 쪽과 같이 이동 방향 기준
        dyaw = math.degrees(wrap(im[-1][1] - y0))
        devs = [math.degrees(wrap(x[1] - y0)) for x in im]
        dmax = max(devs, key=abs)
        de = math.degrees(wrap(o[-1][3] - h0))
        pl = max(abs(x[1]) for x in w)
        pr = max(abs(x[2]) for x in w)
        t30 = next((x[0] - t0 for x in w if abs(x[1]) >= 30 and abs(x[2]) >= 30), float('nan'))
        res[d].append((dyaw, dmax, de, li * 1000, le * 1000, pl, pr, t30))
        print('%s %2d   | %+6.2f°  |   %+6.2f°     |  %+6.2f°   |   %+6.1f mm    |  %+6.1f mm    |   %3.0f/%3.0f    |  %.1f s' % (
            '전진' if d == 'FWD' else '후진', k, dyaw, dmax, de, li * 1000, le * 1000, pl, pr, t30))
print()
for d in ('FWD', 'BWD'):
    r = res[d]
    print('%s 평균: IMU Δyaw %+.2f°±%.2f | 최대꺾임 %+.2f° | 엔코더Δyaw %+.2f° | 옆밀림 IMU %+.1f±%.1f mm, 엔코더 %+.1f mm | 최고 %.0f/%.0f | 출발 %.2f s' % (
        '전진' if d == 'FWD' else '후진',
        st.mean(x[0] for x in r), st.pstdev([x[0] for x in r]), st.mean(x[1] for x in r), st.mean(x[2] for x in r),
        st.mean(x[3] for x in r), st.pstdev([x[3] for x in r]), st.mean(x[4] for x in r),
        st.mean(x[5] for x in r), st.mean(x[6] for x in r), st.mean(x[7] for x in r)))
print('  (옆밀림 + = 이동 방향 기준 왼쪽, − = 오른쪽. 후진은 뒤로 가는 방향 기준)')
print('\n튀는 값 점검 (전체 %.0f s)' % (OD[-1][0] - OD[0][0]))
gaps = [b[0] - a[0] for lst in (OD, IM, WS) for a, b in zip(lst, lst[1:])]
print('  메시지 간격 >150ms: %d건 (최대 %.0f ms)' % (sum(g > 0.15 for g in gaps), max(gaps) * 1000))
mid = []
for k in range(1, N + 1):
    for d in ('FWD', 'BWD'):
        t0 = t_of(d + '_START', k)
        t1 = t_of(d + '_END', k)
        w = [x for x in WS if t0 + 1.5 <= x[0] <= t1 - 2.2]
        mid += [(d, k) for a, b in zip(w, w[1:]) for i in (1, 2) if abs(b[i] - a[i]) > 25]
print('  주행 중 바퀴 속도 급변 >25mm/s/0.1s: %d건 %s' % (len(mid), mid[:8]))
print('  IMU yaw 한 샘플 변화 >3.5°: %d건' % sum(abs(math.degrees(wrap(b[1] - a[1]))) > 3.5 for a, b in zip(IM, IM[1:])))
print('  IMU roll/pitch 한 샘플 변화 >1°: %d건' % sum(abs(math.degrees(b[2] - a[2])) > 1 or abs(math.degrees(b[3] - a[3])) > 1 for a, b in zip(IM, IM[1:])))
