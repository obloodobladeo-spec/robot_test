#!/usr/bin/env python3
"""shuttle 로그의 정상 주행 구간 바퀴 속도 흔들림.  usage: wheel_noise.py <log> [라벨]"""
import statistics as st
import sys

rows = [line.split() for line in open(sys.argv[1])]
marks = [(float(r[0]), r[2], int(r[3])) for r in rows if r[1] == 'MARK']
WS = [(float(r[0]), float(r[2]) * 1000, float(r[3]) * 1000) for r in rows if r[1] == 'WS']
done = {m[2] for m in marks if m[1] == 'BWD_END'}
N = max(k for k in done if all(j in done for j in range(1, k + 1)))
L, R, D, J = [], [], [], []
for k in range(1, N + 1):
    for d in ('FWD', 'BWD'):
        t0 = [m[0] for m in marks if m[1] == d + '_START' and m[2] == k][0]
        t1 = [m[0] for m in marks if m[1] == d + '_END' and m[2] == k][0]
        w = [x for x in WS if t0 + 1.5 <= x[0] <= t1 - 2.3]
        L.append(st.pstdev([x[1] for x in w]))
        R.append(st.pstdev([x[2] for x in w]))
        D.append(st.pstdev([x[1] - x[2] for x in w]))
        J.append(st.mean(abs(b[1] - a[1]) + abs(b[2] - a[2]) for a, b in zip(w, w[1:])) / 2)
print('%-14s 바퀴 속도 σ L %.1f R %.1f | 좌우차 σ %.1f | 샘플간 변화 평균 %.1f mm/s' % (
    sys.argv[2] if len(sys.argv) > 2 else '', st.mean(L), st.mean(R), st.mean(D), st.mean(J)))
