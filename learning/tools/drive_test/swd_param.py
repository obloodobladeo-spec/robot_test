#!/usr/bin/env python3
"""SWD로 실행 중인 MCU RAM의 제어 파라미터 읽기/쓰기 (펌웨어 재업로드 없이 시험용).

usage: swd_param.py                     # 전체 읽기
       swd_param.py name=value ...      # 예: R_circ=332.2 kp=0.12
재부팅·재업로드하면 펌웨어 기본값으로 돌아간다.
"""
import glob
import re
import struct
import subprocess
import sys

CLI = glob.glob('/opt/st/stm32cubeide_2.2.0/plugins/*cubeprogrammer*/tools/bin/STM32_Programmer_CLI')[0]
NM = glob.glob('/opt/st/stm32cubeide_2.2.0/plugins/*gnu-tools-for-stm32*/tools/bin/arm-none-eabi-nm')[0]
ELF = '/home/kim/stm32_project/board_practice/Debug/board_practice.elf'
sym = {}
for line in subprocess.run([NM, ELF], capture_output=True, text=True).stdout.splitlines():
    p = line.split()
    if len(p) == 3:
        sym[p[2]] = int(p[0], 16)
M1, M2, RB = sym['motor1'], sym['motor2'], sym['robot']
# Motor: config(0x00..0x2B) state(0x2C..). Robot: track 0x08
P = {
    'L_cpr': M1 + 0x1C, 'L_circ': M1 + 0x20, 'L_ff0': M1 + 0x24, 'L_ffk': M1 + 0x28,
    'R_cpr': M2 + 0x1C, 'R_circ': M2 + 0x20, 'R_ff0': M2 + 0x24, 'R_ffk': M2 + 0x28,
    'L_kp': M1 + 0x50, 'L_ki': M1 + 0x54, 'R_kp': M2 + 0x50, 'R_ki': M2 + 0x54,
    'track': RB + 0x08,
}
ALIAS = {'kp': ('L_kp', 'R_kp'), 'ki': ('L_ki', 'R_ki'), 'ff0': ('L_ff0', 'R_ff0'), 'ffk': ('L_ffk', 'R_ffk')}


def run(args):
    r = subprocess.run([CLI, '-c', 'port=SWD', 'mode=HOTPLUG'] + args, capture_output=True, text=True).stdout
    return [line.strip() for line in re.sub(r'\x1b\[[0-9;]*m', '', r).splitlines() if line.strip().startswith('0x')]


writes = []
for a in sys.argv[1:]:
    k, v = a.split('=')
    for name in ALIAS.get(k, (k,)):
        writes += ['-w32', '0x%08X' % P[name], '0x%08X' % struct.unpack('>I', struct.pack('>f', float(v)))[0]]
reads = []
for name in P:
    reads += ['-r32', '0x%08X' % P[name], '4']
lines = run(writes + reads)
vals = {}
for line in lines:
    addr = int(line.split()[0], 16)
    for name, a in P.items():
        if a == addr:
            vals[name] = struct.unpack('>f', bytes.fromhex(line.split()[2]))[0]
print(' '.join('%s=%.4g' % (n, vals.get(n, float('nan'))) for n in P))
