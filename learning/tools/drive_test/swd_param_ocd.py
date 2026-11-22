#!/usr/bin/env python3
"""openocd(ST-LINK)로 실행 중 MCU RAM의 제어 파라미터 읽기/쓰기 (Jetson용, 업로드 없이 시험).

usage: swd_param_ocd.py                 # 전체 읽기
       swd_param_ocd.py name=value ...  # 예: R_circ=333.5 h_kp_enc=1.5
주소는 같은 폴더의 fw_symbols.json (노트북에서 ELF 빌드 후 갱신). 재부팅·재업로드하면 펌웨어 기본값으로 돌아간다.
"""
import json
import os
import re
import struct
import subprocess
import sys

S = {k: int(v, 16) for k, v in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fw_symbols.json'))).items()}
M1, M2, RB = S['motor1'], S['motor2'], S['robot']
P = {
    'L_cpr': M1 + 0x1C, 'L_circ': M1 + 0x20, 'L_ff0': M1 + 0x24, 'L_ffk': M1 + 0x28,
    'R_cpr': M2 + 0x1C, 'R_circ': M2 + 0x20, 'R_ff0': M2 + 0x24, 'R_ffk': M2 + 0x28,
    'L_kp': M1 + 0x50, 'L_ki': M1 + 0x54, 'R_kp': M2 + 0x50, 'R_ki': M2 + 0x54,
    'track': RB + 0x08,
    'h_kp_enc': RB + 0x0C, 'h_kp_imu': RB + 0x10, 'h_ki': RB + 0x14, 'h_max': RB + 0x18,
}
ALIAS = {'kp': ('L_kp', 'R_kp'), 'ki': ('L_ki', 'R_ki'), 'ff0': ('L_ff0', 'R_ff0'), 'ffk': ('L_ffk', 'R_ffk')}


def f2u(x):
    return struct.unpack('<I', struct.pack('<f', float(x)))[0]


cmds = ['init']
for a in sys.argv[1:]:
    k, v = a.split('=')
    for name in ALIAS.get(k, (k,)):
        cmds.append('mww 0x%08X 0x%08X' % (P[name], f2u(v)))
for name in P:
    cmds.append('mdw 0x%08X 1' % P[name])
cmds.append('exit')
args = ['openocd', '-f', 'interface/stlink.cfg', '-f', 'target/stm32f4x.cfg']
for c in cmds:
    args += ['-c', c]
r = subprocess.run(args, capture_output=True, text=True, timeout=30)
vals = {}
for line in (r.stdout + r.stderr).splitlines():
    m = re.match(r'^0x([0-9a-fA-F]+):\s+([0-9a-fA-F]+)', line.strip())
    if m:
        addr = int(m.group(1), 16)
        for name, a in P.items():
            if a == addr:
                vals[name] = struct.unpack('<f', struct.pack('<I', int(m.group(2), 16)))[0]
if not vals:
    print(r.stdout[-500:], r.stderr[-800:])
print(' '.join('%s=%.4g' % (n, vals.get(n, float('nan'))) for n in P))
