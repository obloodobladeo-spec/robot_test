#!/usr/bin/env python3
"""MCU의 g_imu.yaw_rad를 SWD로 직접 읽으며(약 30 ms 주기) 목표 각도만큼 제자리 회전 후 정지.

회전 명령만 /cmd_vel로 보낸다. 정지 판단·최종 회전량은 모두 MCU 값 기준.
usage: imu_rot_swd.py <목표 deg> <방향 +1 반시계 / -1 시계> <log>
"""
import glob
import math
import re
import struct
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

TARGET = math.radians(float(sys.argv[1]))
SIGN = 1.0 if float(sys.argv[2]) > 0 else -1.0
out = open(sys.argv[3], 'w')
CLI = glob.glob('/opt/st/stm32cubeide_2.2.0/plugins/*cubeprogrammer*/tools/bin/STM32_Programmer_CLI')[0]
NM = glob.glob('/opt/st/stm32cubeide_2.2.0/plugins/*gnu-tools-for-stm32*/tools/bin/arm-none-eabi-nm')[0]
ELF = '/home/kim/stm32_project/board_practice/Debug/board_practice.elf'
G = [int(line.split()[0], 16) for line in subprocess.run([NM, ELF], capture_output=True, text=True).stdout.splitlines()
     if line.endswith(' g_imu')][0]
A_COUNT, A_YAW = G + 0x14, G + 0x40   # rotation_count, yaw_rad
WMAX, WMIN = 0.3, 0.15
SLOW = math.radians(20)
LEAD = math.radians(1.0)   # 0.15 rad/s x (SWD 30 ms + 명령 전달·정지 지연) 근사


def read_imu():
    r = subprocess.run([CLI, '-c', 'port=SWD', 'mode=HOTPLUG', '-r32', '0x%08X' % A_COUNT, '4', '-r32', '0x%08X' % A_YAW, '4'],
                       capture_output=True, text=True).stdout
    v = [line.split()[2] for line in re.sub(r'\x1b\[[0-9;]*m', '', r).splitlines() if line.strip().startswith('0x')]
    return int(v[0], 16), struct.unpack('>f', bytes.fromhex(v[1]))[0]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Cmd(Node):
    def __init__(self):
        super().__init__('imu_rot_swd')
        self.w = 0.0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.tick)

    def tick(self):
        t = Twist()
        t.angular.z = self.w
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Cmd()
    running = [True]

    def spin_loop():
        while running[0] and rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)

    th = threading.Thread(target=spin_loop, daemon=True)
    th.start()
    try:
        # 시작 전 정지 상태 기준값 (5회 평균)
        ys = [read_imu()[1] for _ in range(5)]
        y_last = ys[-1]
        acc = 0.0
        out.write('%.3f START %.6f\n' % (time.time(), y_last))
        n.w = SIGN * WMAX
        t0 = time.time()
        while True:
            c, y = read_imu()
            acc += wrap(y - y_last)
            y_last = y
            out.write('%.3f YAW %d %.6f %.6f\n' % (time.time(), c, y, acc))
            rem = TARGET - abs(acc)
            if rem <= LEAD:
                break
            n.w = SIGN * max(WMIN, min(WMAX, WMAX * rem / SLOW))
            if time.time() - t0 > 180:
                print('TIMEOUT')
                break
        n.w = 0.0
        t_stop = time.time()
        out.write('%.3f STOP %.6f\n' % (t_stop, acc))
        acc_stop = acc
        # 완전히 멈출 때까지 계속 누적 (3 s)
        while time.time() - t_stop < 3.0:
            c, y = read_imu()
            acc += wrap(y - y_last)
            y_last = y
            out.write('%.3f YAW %d %.6f %.6f\n' % (time.time(), c, y, acc))
        print('MCU IMU 회전량: 정지 명령 시 %.2f° → 정지 후 %.2f° (목표 %.0f°)' % (
            math.degrees(acc_stop) * SIGN, math.degrees(acc) * SIGN, math.degrees(TARGET)))
        out.write('%.3f FINAL %.6f\n' % (time.time(), acc))
    finally:
        n.w = 0.0
        time.sleep(0.3)
        running[0] = False
        th.join(timeout=2)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
