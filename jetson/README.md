# Jetson으로 옮길 파일

이 `jetson` 폴더 전체를 SD카드로 옮깁니다. Windows의 STM32 개발 도구나 `firmware` 폴더는 Jetson에 필요하지 않습니다. STM32에는 Windows에서 새 펌웨어를 먼저 업로드합니다.

## 1. SD카드에서 홈으로 복사

Jetson 터미널에서 실제 SD카드 마운트 경로를 확인하고 실행하세요.

```bash
mkdir -p ~/robot_test
cp -r /media/$USER/카드이름/jetson ~/robot_test/
```

결과는 `~/robot_test/jetson/robot_ws`, `~/robot_test/jetson/scripts`여야 합니다. Jetson의 운영체제용 SD카드를 Windows에서 포맷하지 마세요. 이동용 카드의 읽을 수 있는 데이터 파티션으로 파일을 전달하고, 빌드는 홈의 Linux 파일시스템에서 합니다.

## 2. 준비와 빌드

ROS 2 Humble이 이미 설치된 Ubuntu 22.04를 기준으로 구성했습니다. 실제 Jetson의 ROS 설치 여부는 아직 확인하지 않았습니다.

```bash
ls /opt/ros
bash ~/robot_test/jetson/scripts/setup.sh humble
```

설치 스크립트는 필요한 apt 패키지를 설치하고 현재 계정을 `dialout` 그룹에 추가합니다. 이후 로그아웃하고 SSH로 다시 접속하세요. 이미 준비된 환경에서는 다음 명령으로 빌드만 합니다.

```bash
bash ~/robot_test/jetson/scripts/build.sh humble
```

## 3. 실제 로봇 설정

STM32의 ST-LINK USB 포트를 Jetson USB에 연결합니다.

```bash
ls -l /dev/serial/by-id/
nano ~/robot_test/jetson/robot_ws/src/robot_test/config/robot.yaml
```

- `port`: 실제 ST-LINK 가상 시리얼 경로. 가능하면 `/dev/serial/by-id/...` 사용
- `wheel_separation_m`: 좌우 바퀴 중심 사이 거리(m). 기본 0.0은 미설정 표시
- `motor1_side`: 로봇의 전진 방향을 바라봤을 때 모터 1이 `left`인지 `right`인지 설정
- 두 모터 모두 DIR HIGH가 전진, 엔코더 속도가 양수인 현재 배선을 기준으로 함
- 위 두 기하 설정이 미확정이면 구동 활성화가 거부됨. 임의로 바퀴 간격을 입력하지 말 것

`driver.sh`는 소스 폴더의 YAML을 직접 전달하므로 설정 변경 뒤 재빌드 없이 드라이버 재시작으로 반영됩니다. 실행 중 파라미터 변경으로 구동을 켜는 기능은 제공하지 않습니다.

## 4. Windows에서 SSH로 실행

Windows PowerShell 창 두 개에서 각각 접속합니다. ROS는 Jetson에서 실행되므로 Windows에 ROS 2를 설치하거나 Windows와 Jetson 사이에 DDS 네트워크를 구성할 필요가 없습니다.

```powershell
ssh -t 사용자명@Jetson_IP
```

첫 번째 SSH 창: 바퀴를 띄우고 통신만 먼저 확인합니다.

```bash
bash ~/robot_test/jetson/scripts/driver.sh
```

기본은 구동 잠금입니다. 별도 SSH 창에서 상태를 확인할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/robot_test/jetson/robot_ws/install/setup.bash
ros2 topic echo /robot/status
```

실제 바퀴 방향과 설정을 확인한 뒤 드라이버를 Ctrl+C로 종료하고 다음처럼 재실행합니다.

```bash
bash ~/robot_test/jetson/scripts/driver.sh enable_motors:=true
```

두 번째 SSH 창:

```bash
bash ~/robot_test/jetson/scripts/teleop.sh
```

| 키 | 동작 |
| --- | --- |
| w / x | 목표 선속도 +0.01 / -0.01m/s |
| a / d | 목표 각속도 +0.1 / -0.1rad/s |
| s / Space | 즉시 0 명령 |
| Ctrl+C | 정지 명령 후 종료 |

선속도 상한 ±0.2m/s, 각속도 상한 ±0.5rad/s, 각 바퀴 목표 상한 ±200mm/s입니다. 두 속도를 동시에 주면 바퀴 속도 제한에 맞춰 좌우 목표를 같은 비율로 줄입니다. `a`는 ROS 기준 양의 yaw(좌회전)이며 motor1_side 설정이 맞아야 합니다.

TurtleBot3의 증분·키 배치·100ms 입력 대기와 완만한 출력 변경을 참고했습니다. 본 구현은 10Hz 고정 발행, 선속도 0.005m/s·각속도 0.05rad/s씩 출력 접근, 1초간 유효 키 입력이 없으면 정지합니다. TurtleBot3처럼 입력 후 속도를 무기한 유지하는 방식과 이 부분은 다릅니다. 계속 주행하려면 키를 반복 입력합니다.

## 통신과 정지

- `/cmd_vel`: `geometry_msgs/msg/Twist`, linear.x=m/s, angular.z=rad/s
- `/robot/motor_speeds`: `std_msgs/msg/Float64MultiArray`, [모터1, 모터2] 측정 m/s
- `/robot/status`: JSON 문자열, 모터 속도·듀티·STM32 타임아웃 상태
- 드라이버 명령 전송 20Hz, STM32 측정·PI·피드백 10Hz
- cmd_vel이 0.3초간 없거나 정상 피드백이 0.5초간 없으면 드라이버가 0 전송
- STM32는 정상 명령이 0.5초간 없으면 독립적으로 PWM 0, 적분값 0
- USB 오류 후 자동 재연결하지 않음. 원인 확인 후 드라이버를 재시작
- STM32 하드웨어 watchdog이 main 정지 시 약 1초 후 리셋(LSI 공차 있음)
- 통신 정지는 능동적인 위치 유지 브레이크가 아님. 관성 이동이 있을 수 있음
- 200mm/s는 목표 속도 제한이며 실제 속도 초과를 물리적으로 보증하는 인증이 아님
- 첫 시험은 바퀴를 띄워 시행. 바닥 시험·비상 전원 차단은 사용자가 직접 확인

현재 범위는 키보드 → cmd_vel → STM32 두 바퀴 속도 제어입니다. odom/TF, SLAM, Nav2, 카메라 기능은 포함하지 않습니다.

## 문제 해결

- 포트 권한 거부: `id -nG`로 dialout 확인 후 로그아웃·재접속
- 포트 없음: USB가 STM32의 ST-LINK 포트에 꽂혔는지와 실제 장치 경로 확인
- 데이터가 안 나옴: 새 STM32 ELF 업로드, 115200 8N1, Nucleo ST-LINK VCP용 PA2/PA3 연결 상태 확인
- 속도 목표는 바뀌는데 안 움직임: 배터리·MDD10A·공통 GND·50% 출력 상한 확인
- 한 방향에서 측정 부호가 반대: 즉시 정지하고 엔코더 배선·방향 기준 확인
- SSH 비대화형 입력 오류: `ssh -t` 사용. IDE 출력 창 대신 실제 터미널 사용

출처: [ROBOTIS TurtleBot3 Humble teleop](https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py), [ROS 2 차동 구동 제어 설명](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html).
