#!/usr/bin/env python3
"""straight_trials.py 로그 분석: 회차별 출발·속도·방향, 튀는 값 점검.  usage: trials_analyze.py <log>"""
import math
import statistics as st
import sys

rows = [line.split() for line in open(sys.argv[1])]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


marks = [(float(r[0]), r[2], int(r[3]), r[4:]) for r in rows if r[1] == 'MARK']
speeds = {int(r[2]): float(r[3]) for r in rows if r[1] == 'SPEED'}
OD = [(float(r[0]), *map(float, r[2:7])) for r in rows if r[1] == 'ODOM']
IM = [(float(r[0]), *map(float, r[2:8])) for r in rows if r[1] == 'IMU']
WS = [(float(r[0]), float(r[2]) * 1000, float(r[3]) * 1000) for r in rows if r[1] == 'WS']
N = max(m[2] for m in marks)


def seg(a_list, a, b):
    return [x for x in a_list if a <= x[0] <= b]


def mark(name, k):
    return [m for m in marks if m[1] == name and m[2] == k][0]


print('회차 속도 | 30mm/s도달 | 최고 L/R | 엔코더Δyaw | IMUΔyaw | IMU최대편차 | 옆밀림 | 정렬오차')
S = []
for k in range(1, N + 1):
    t0 = mark('FWD_START', k)[0]
    t1 = mark('FWD_STOPPED', k)[0]
    al = mark('ALIGN_END', k)
    w = seg(WS, t0, t1)
    o = seg(OD, t0, t1)
    im = seg(IM, t0, t1)
    t30 = next((x[0] - t0 for x in w if x[1] >= 30 and x[2] >= 30), float('nan'))
    pl = max(x[1] for x in w)
    pr = max(x[2] for x in w)
    de = math.degrees(wrap(o[-1][3] - o[0][3]))
    di = math.degrees(wrap(im[-1][1] - im[0][1]))
    dev = max(abs(math.degrees(wrap(x[1] - im[0][1]))) for x in im)
    h = o[0][3]
    lat = (-(o[-1][1] - o[0][1]) * math.sin(h) + (o[-1][2] - o[0][2]) * math.cos(h)) * 1000
    S.append((t30, pl, pr, de, di, dev, lat))
    print(' %2d %3.0f |  %.1f s    | %2.0f/%2.0f  |  %+5.2f°   | %+5.2f° |   %.2f°    | %+5.1fmm | %+.2f°' % (
        k, speeds.get(k, 0.05) * 1000, t30, pl, pr, de, di, dev, lat, float(al[3][-1])))
print('평균: 30mm/s %.2fs, 최고 %.0f/%.0f, 엔코더Δyaw %+.2f°, IMUΔyaw %+.2f°±%.2f (|평균| %.2f°), 최대편차 %.2f°' % (
    st.mean(s[0] for s in S), st.mean(s[1] for s in S), st.mean(s[2] for s in S),
    st.mean(s[3] for s in S), st.mean(s[4] for s in S), st.pstdev([s[4] for s in S]),
    st.mean(abs(s[4]) for s in S), st.mean(s[5] for s in S)))
print('\n튀는 값 점검 (전체 %.0f s)' % (OD[-1][0] - OD[0][0]))
gaps = [b[0] - a[0] for lst in (OD, IM, WS) for a, b in zip(lst, lst[1:])]
print('  메시지 간격 >150ms: %d건 (최대 %.0f ms)' % (sum(g > 0.15 for g in gaps), max(gaps) * 1000))
mid = []
for k in range(1, N + 1):
    t0 = mark('FWD_START', k)[0]
    t1 = mark('FWD_END', k)[0]
    w = seg(WS, t0 + 1.5, t1 - 0.2)
    mid += [(b[0] - t0, k, 'LR'[i - 1], a[i], b[i]) for a, b in zip(w, w[1:]) for i in (1, 2) if abs(b[i] - a[i]) > 25]
print('  주행 중(출발 1.5 s 이후) 바퀴 속도 급변 >25mm/s/0.1s: %d건 %s' % (len(mid), [(k, '%.1fs' % t, s, round(x), round(y)) for t, k, s, x, y in mid][:8]))
print('  IMU yaw 한 샘플 변화 >3.5°: %d건' % sum(abs(math.degrees(wrap(b[1] - a[1]))) > 3.5 for a, b in zip(IM, IM[1:])))
print('  IMU roll/pitch 한 샘플 변화 >1°: %d건' % sum(abs(math.degrees(b[2] - a[2])) > 1 or abs(math.degrees(b[3] - a[3])) > 1 for a, b in zip(IM, IM[1:])))
