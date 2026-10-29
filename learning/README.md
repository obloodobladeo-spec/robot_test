# 로봇 제어 학습: 처음부터 C1까지

정리 기준: 2026-10-29. 목표는 **ROS 2 속도 명령으로 실제 바퀴를 제어하고, 측정한 이동과 센서 데이터를 다시 받는 것**입니다. 이 폴더는 업로드 시점의 복사본입니다. 원본 Windows STM32 워크스페이스와 WSL `~/ros2_study`는 수정하지 않았습니다.

계산 흐름과 헷갈렸던 부분은 **[핵심 개념 복습](CONCEPTS.md)**에 따로 정리했습니다.

## 학습 순서와 현재 위치

| 순서 | 배운 내용과 핵심 | 자료 / 확인 범위 |
|---|---|---|
| 1 | GPIO·LED 출력과 토글 | 보드 기본 실습 |
| 2 | 타이머: 클럭·PSC·ARR·CNT·주기 인터럽트 | `stm32/timer_test` |
| 3 | 인터럽트와 메인, 플래그·pending, static·volatile, 디버거 | 짧게 저장·요청하고 계산은 메인에서 처리 |
| 4 | PWM: 주파수·듀티·CCR·preload·LED 밝기 | `stm32/PWM_test`; PWM 하드웨어와 메인 갱신 주기는 다름 |
| 5 | 입력 캡처: 첫 기준·시간 차이·카운터 경계 | `PWM_test`의 현재 저장본에 캡처 코드 포함 |
| 6 | 모터 기본 구동: PWM·DIR·시작·정지·양방향 | MDD10A 테스트 버튼과 STM32 출력 확인 |
| 7 | 엔코더 A/B 위상·×4·방향·한 바퀴 count | `encorder_test`, `446re_sample_project`, `encoder_basic` |
| 8 | count → 회전량·거리·RPM·선속도 | 실제 측정 간격, 16비트 경계 보정 |
| 9 | 목표−실제 오차·P·I·출력 제한·anti-windup | `encoder_basic/Core/Src/motor_control.c` |
| 10 | 좌우 제어·가감속·방향 전환·부하 영향 | 띄운 바퀴와 바닥 시험, 휠·허브·마찰 비교 |
| 11 | 구조체와 C/H 파일 분리 | `motor_control.*`, `robot_control.*` |
| 12 | 차동구동: 선속도·각속도 → 좌우 바퀴 목표 | 구동축 중간점 기준, 윤거230mm |
| 13 | 바퀴 오도메트리: x/y/yaw | 엔코더 추정이며 실제 지면 위치의 독립 검증은 아님 |
| 14 | UART 수신 버퍼·줄 파싱·피드백·시간 초과 정지 | 현재 명령 `V=… W=…` |
| 15 | Python 시리얼 → ROS 2 `/cmd_vel` 구독 | `motor_bridge`, 저장소 루트의 `tools/` |
| 16 | `/wheel_speeds`·`/odom`·TF 발행 | 양·음 속도, 정지, 시간 대비 이동·회전 시험 |
| 17 | Jetson 이전과 USB STM32 연결 | Humble, `~/ros2_ws`, 정지 상태 피드백·odom·TF 확인 |
| 18 | URDF: 몸체·바퀴·받침·라이다·좌표 | `robot_description`, RViz 표시 확인 |
| 19 | launch·CMake 설치·RViz 설정 저장 | `display.launch.py`, `display.rviz` |
| 20 | TF 조회와 관계 합성 | `base_link → laser`, 시험용 `odom → laser` 확인 |
| 21 | `/scan`: 각도·거리 배열·frame·주기 | 가짜 점3개로 표시 원리를 먼저 확인 |
| 22 | 실제 C1: Windows → WSL USB → 제조사 드라이버 | 실제720개 거리값, `frame_id=laser`, 약10Hz 확인 |
| 23 | `/scan` Python 구독 | `scan_reader.py` 작성·수신 확인 사례 있음. 반복 실행 시 발견 문제는 남음 |

기존 [학습 순서 원본](study_notes/notes/2026/10/2026-10-16.md)과 날짜별 노트도 함께 보관했습니다. 과거 기록의 ‘예정’은 당시 상태입니다. 각 STM32 프로젝트는 모든 중간 단계가 아니라 **마지막 저장 상태**입니다. 초기 학습 설명의 타이머 클럭·표본 주기를 현재 프로젝트의 값으로 혼동하지 않습니다.

## 폴더와 역할

```text
learning/
├── stm32/                      # 7개 학습 프로젝트, 현재 encoder_basic
├── ros2_ws/src/
│   ├── motor_bridge/           # 명령·USB·odom·TF·scan 구독
│   ├── robot_description/      # URDF·launch·RViz 설정
│   └── sllidar_ros2/           # 실제 설치한 공식 드라이버, LICENSE 포함
├── study_notes/notes/2026/10/   # 원본 노트
├── study_notes/assets/motor_tests/ # 시험 로그
├── hardware/                   # 원본 장비 기록과 보충
├── SOURCE_SNAPSHOT.json        # 출처·버전·복사 제외 범위
└── VALIDATION.md               # 이번 검증과 한계
```

STM32에는 `timer_test`, `PWM_test`, `encorder_test`, `446re_sample_project`, `mdd10a_standalone`, `sample_mode`, `encoder_basic`을 보관했습니다. 기존 저장소의 별도 CRC 시험 프로젝트는 루트 `firmware/`와 `jetson/`에 그대로 있습니다. 현재 학습 펌웨어와 혼용하지 않습니다.

관련 독립 Python 자료는 루트 `tools/learning_serial.py`, `feedback_example.py`, `keyborad_example.py`, `serial_startup_probe.py`, `motor_log.csv`입니다. Windows 키보드 API나 COM 번호를 쓰는 도구도 있으므로 실행 환경을 확인합니다.

새 `learning/`은 소스·Drivers·CubeMX `.ioc`·링커 스크립트·IDE 프로젝트 파일과 노트를 보관합니다. 워크스페이스 캐시 `.metadata`, IDE 로컬 `.settings`, `Debug/Release`, ROS `build/install/log`, Python 캐시, 중첩 `.git`은 복사하지 않았습니다. `.gitignore`를 추가하지 않았으며 기존 저장소의 빌드 산출물은 유지했습니다.

## STM32와 하드웨어

하드웨어 보유 여부는 [원본 기록](hardware/README.md)을 먼저 확인하고, 현재 연결 여부는 별도로 검사합니다. 최근 모델 치수와 실습 결과는 [보충 기록](hardware/SNAPSHOT_NOTES.md)에 있습니다.

| 기능 | 핀 / 설정 |
|---|---|
| 왼쪽 후륜 모터1 PWM / DIR | PA8 TIM1_CH1 / PC0 |
| 오른쪽 후륜 모터2 PWM / DIR | PA9 TIM1_CH2 / PC1 |
| 모터1 엔코더 | PA0·PA1 TIM2 32비트 |
| 모터2 엔코더 | PA6·PA7 TIM3 16비트 |
| ST-LINK USB VCP / USART2 | PA2 TX·PA3 RX, 115200 8N1 |

- MCU STM32F446RE, MDD10A, 모터2개. GND 공통.
- 바퀴 지름110mm·폭45mm, 윤거230mm, 펌웨어 둘레345.6mm.
- 실측 count/바퀴: 모터1 2465.2, 모터2 2475.6. 이미 ×4로 센 값이므로 다시4를 곱하지 않습니다.
- 현재 `encoder_basic`: PWM20kHz, 제어 계산100ms, 상태 송신1초, Kp=0.1·Ki=0.1, 듀티 상한50%, 바퀴 속도±200mm/s, 가속도 제한100mm/s².
- CNT는 엔코더 신호로 계속 증감합니다. 메인에서 읽는 주기는 엔코더 신호 주파수나 while 전체 반복 주기가 아닙니다.
- `static`은 호출 사이 값 유지·수명에 관한 기능이고 `volatile`은 ISR 등에서 바뀌는 값의 접근에 관한 기능입니다. 원자성·동기화를 보장하지 않습니다.
- P는 현재 오차에 반응하고 I는 시간에 따른 오차를 누적합니다. 출력 포화에서 적분이 계속 쌓이지 않도록 제한합니다. 타이어·허브·접지 문제는 이득 조정만으로 해결되지 않습니다.

현재 학습 규약은 다음과 같습니다.

```text
상위 → STM32: V=100 W=0\n
V는 mm/s, W는 mrad/s

STM32 → 상위: L=100 R=99 X=123 Y=1 A=10\r\n
L/R는 mm/s, X/Y는 mm, A는 mrad
```

브리지는 `/cmd_vel`의 m/s·rad/s를 정수 mm/s·mrad/s로 변환합니다. 선속도는 ±0.2m/s, 각속도는 ±0.5rad/s로 제한하고 0.1초마다 송신합니다. 유효 명령이0.5초 없으면 정지 목표를 보냅니다. STM32에도 명령 시간 초과 처리가 있습니다.

현재 피드백은 정수 단위이며 CRC·순번·MCU 측정시각은 없습니다. ROS 시간은 수신 처리 시각이고, 공분산은 아직 설정하지 않았습니다. USB 분리 예외 처리 등도 완성된 운용 기능으로 보지 않습니다. 기존 CRC 패키지의 안전 기능이 이 코드에도 구현됐다고 가정하지 않습니다.

## 노드·토픽과 TF

| 노드 | 입력 | 출력 / 역할 |
|---|---|---|
| `cmd_vel_receiver` | `/cmd_vel`와 USB 피드백 | `/wheel_speeds`, `/odom`, `odom → base_link` TF |
| `robot_state_publisher` | URDF·`/joint_states` | `/robot_description`, `/tf`, `/tf_static` |
| `joint_state_publisher` | `/robot_description` | 표시용 `/joint_states`; 실제 엔코더 각도는 아님 |
| `sllidar_node` | C1 USB 시리얼 | 실제 `/scan` |
| `scan_reader` | `/scan` | 좌표 이름과 측정점 수 로그 |
| RViz | 모델·TF·센서·odom | 표시, 모터 제어·물리 계산은 수행하지 않음 |

```text
odom                            # 실제 로봇 연결 시 브리지가 발행
└─ base_link                    # 좌우 후륜 구동축 중간점
   ├─ left_wheel_link
   ├─ right_wheel_link
   └─ chassis_link              # x=0.108m
      ├─ front_left_wheel_link
      ├─ front_right_wheel_link
      └─ laser                  # 차체 기준 x=0.042m, z=0.088m
```

차체는250×180×56.4mm이고 후륜축이 뒤 끝에서17mm이므로 중심은 구동축보다108mm 앞입니다. 앞 보조 바퀴 축은 차체 기준92mm인 임시 위치이며 실측 완료가 아닙니다. 앞바퀴는 현재 fixed joint로 외형만 표현합니다.

라이다는 차체 앞 끝에서 뒤로83mm이므로 중심 기준x=42mm입니다. 받침은 지름116mm·높이30mm, 측정면은 C1 바닥에서29.8mm 위이므로 `z=28.2+30+29.8=88mm`입니다. 라이다 외형 중심은 측정면보다9.15mm 아래에 그립니다.

- `base_link → laser`는 `[0.150, 0, 0.088]m`.
- 임시 `odom → base_link`를 앞쪽1m로 발행했을 때 `odom → laser`는 `[1.150, 0, 0.088]m`.
- URDF 길이는m, 회전은rad. joint origin은 부모 기준 자식 좌표이고 visual origin은 해당 링크 안의 외형 배치입니다.
- 바퀴 축 높이에 base_link를 두었으므로 현재 모델의 지면은 그보다55mm 아래입니다. 지면 z=0 표현은 아직 보정하지 않았습니다.
- TF의 고정 관계는 로봇에 붙어 있는 장착 위치입니다. `/scan`만으로 라이다 자체의 방 안 회전량을 알 수는 없습니다.
- `lookup_transform(target, source, Time())`은 source의 자세를 target 기준으로 조회합니다. Listener가 수신한 관계를 Buffer에 보관하며 조회 자체가 로봇을 움직이지는 않습니다.
- 뒷바퀴 joint 이름의 `whell` 철자는 원본 그대로입니다. 실제 JointState 연동 시 URDF 이름과 정확히 맞춰야 합니다.

## ROS 패키지 복원·빌드

Ubuntu22.04 / Humble 기준입니다. 기존 폴더를 덮어쓰지 않도록 새 `robot_learning_ws` 예시를 사용합니다. 원래 PC는 `~/ros2_study`, Jetson은 `~/ros2_ws`입니다.

```bash
# ~/robot_test가 없을 때 실행
git clone https://github.com/obloodobladeo-spec/robot_test.git ~/robot_test
mkdir -p ~/robot_learning_ws/src
cp -a ~/robot_test/learning/ros2_ws/src/. ~/robot_learning_ws/src/
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-serial \
  ros-humble-rviz2 ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-sensor-msgs \
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-std-msgs \
  ros-humble-tf2-ros ros-humble-launch-ros ros-humble-ament-index-python

cd ~/robot_learning_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-select motor_bridge robot_description sllidar_ros2
source install/setup.bash
```

rosdep을 초기화하지 않은 환경에서만 먼저 `sudo rosdep init`을 실행합니다. 원본 보존을 위해 manifest는 변경하지 않았습니다. 원본 `package.xml`에는 일부 의존성 선언이 빠져 있어 위 apt 명령에 필요한 패키지를 명시했습니다.

`--symlink-install`은 설치 대상 파일을 원본에 연결합니다. Python 본문 변경은 노드 재시작으로 반영되지만 실행 등록·CMake 설치 대상·의존성 변경은 재빌드합니다. `setup.py` 문자열 사이 쉼표 누락은 문법 오류 없이 실행 항목을 붙여버릴 수 있습니다.

## C1 연결·실행·종료

학습용 `ros2 topic pub /scan ...`은 먼저 종료합니다. 실제 센서와 가짜 데이터를 같은 토픽에 동시에 발행하지 않습니다.

Windows PowerShell:

```powershell
usbipd list
# BUSID 1-4는 이번 실습값. 현재 CP210x 장치 값을 확인
usbipd bind --busid 1-4
usbipd attach --wsl --busid 1-4
```

bind는 최초 공유 시 관리자 권한이 필요합니다. 이후 attach는 일반 PowerShell에서도 가능합니다. RoboStudio가 포트를 사용하면 연결 해제 후 완전히 종료합니다. WSL 터미널이 켜져 있어야 합니다.

WSL:

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
id -nG
```

dialout이 없으면 `sudo usermod -aG dialout "$USER"` 후 새 로그인에서 확인합니다. WSL에 넘긴 USB는 Windows 프로그램이 동시에 사용할 수 없습니다. WSL 재시작 후 usbipd는 Attached인데 장치가 없을 때는 detach/attach 후 실제 장치 생성을 먼저 확인합니다.

모든 관련 터미널에서 같은 환경을 적용한 뒤 시작합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/robot_learning_ws/install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=0
```

73은 학습용 그룹 번호입니다. LOCALHOST_ONLY=0은 Humble의 기본 네트워크 허용 설정이고1은 PC 내부 진단에 사용했습니다. export는 해당 셸에서 **이후 시작하는 프로그램**에만 적용되므로 이미 실행 중인 노드는 재시작해야 합니다. 개인 `.bashrc`와 `study` 별칭은 저장소에 포함하지 않았습니다.

터미널1:

```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py \
  serial_port:=/dev/ttyUSB0 serial_baudrate:=460800 frame_id:=laser
```

터미널2:

```bash
ros2 launch robot_description display.launch.py
```

터미널3:

```bash
ros2 topic echo /scan sensor_msgs/msg/LaserScan --once --no-daemon --no-arr
ros2 topic hz /scan --spin-time 5
# 주기 확인을 Ctrl+C로 종료한 다음
ros2 run motor_bridge scan_reader
```

RViz Fixed Frame=`base_link`, LaserScan Topic=`/scan`. 저장한 display.rviz를 launch가 불러옵니다. 센서를 차체에 장착하지 않았다면 표시된 로봇과 실제 장애물의 상대 위치를 주행 기준으로 해석하지 않습니다.

발행기 터미널에서 Ctrl+C로 종료하고 **Stop motor 로그와 실제 회전 정지**를 확인합니다. 프로세스나 WSL을 강제 종료해도 모터 구동 상태가 남을 수 있습니다. 이번에는 USB를 복구하고 드라이버 정상 종료 절차로 정지 명령을 보낸 뒤 사용자가 실제 정지를 확인했습니다.

### 실제 `/scan`에서 확인한 값

- health=OK, 펌웨어1.02, Standard 모드.
- 약−π~+π, 각도 간격약0.00874rad, 거리·강도 배열720개, 평균약10Hz.
- `scan_time≈0.1s`와 메시지 수신 주기를 구분합니다. 점 개수도 주파수와 다릅니다.
- `range_min≈0.05m`, 메시지의 `range_max=16m`는 드라이버 상한이며 C1의 모든 표적에서16m 보증이라는 뜻이 아닙니다.
- `ranges[i]` 방향은 `angle_min + i × angle_increment`, 거리는m입니다. 향후 처리에서는 무한대·유효 범위를 검사합니다.
- 0.201초의 최대 수신 간격을 관찰했습니다. max가 계속 표시되는 것과 지연이 반복되는 것은 다릅니다. 원인을 확정하지 않았습니다.
- `scan_reader`는 메시지마다 콜백을 호출하고 로그만1초 간격으로 제한합니다.

## 실제 로봇과 연결

CubeIDE의 기존 프로젝트 가져오기로 `stm32/encoder_basic`을 열고 빌드·업로드합니다. Debug 산출물은 새로 생성합니다. 이번 보관 작업에서는 새 펌웨어 플래시나 구동 시험을 하지 않았습니다.

STM32를 USB로 연결하고 포트와 접근 권한을 확인한 뒤 실행합니다.

```bash
ros2 run motor_bridge cmd_vel_receiver
```

이 노드는 실제 `/cmd_vel`을 STM32로 전달하며 물리적 이동이 발생할 수 있습니다. 모델·라이다만 확인할 때는 필요하지 않습니다. 현재 포트 `/dev/ttyACM0`은 코드에 고정되어 있습니다. 실제 odom을 볼 때는 Fixed Frame=`odom`을 사용하고 `static_transform_publisher`와 브리지가 같은 변환을 중복 발행하지 않게 합니다.

## 오류 기록과 미해결 사항

| 현상 | 확인 내용 |
|---|---|
| 새 셸에서 ros2 명령 없음 | ROS setup을 먼저 source |
| 삭제한 패키지 경로 경고 | 기존 셸의 AMENT/CMAKE_PREFIX_PATH 경로 경고. 최종 빌드 결과와 구분 |
| 빌드 had stderr output | SDK 경고가 출력됨. 실제 빌드는 성공 |
| SDK 배열·미사용 값 경고 | 대부분 호환성·작성 방식. 스레드 함수 형식 변환, 무시된 timeout·취소 쓰기 반환값은 잠재 문제로 남음 |
| 80008004와 종료 | SDK OPERATION_NOT_SUPPORT. 당시 WSL ttyUSB0도 없었음. 센서 고장으로 단정하지 않음 |
| Attached인데 ttyUSB 없음 | USB 재연결로 복구한 사례 있음 |
| 간헐적으로 `/scan` 발견 실패 | **미해결**. 동일 도메인·로컬 설정에서도 재현. 보조 조회 후 다른 조회가 되는 현상 사용자 보고 |
| no-daemon으로 조회됨 | CLI daemon 우회 성공 사례이며 DDS 전체 해결을 뜻하지 않음 |
| spin-time5에서 hz 출력 | 해당 검사 성공. 재실행 안정성을 입증한 것은 아님 |

기본 DDS와 UDP 전용 임시 조회가 모두 수신되어 공유 메모리가 원인이라는 가설은 입증하지 못했습니다. 임시 도메인73·LOCALHOST_ONLY1에서20초간174개·약10.19Hz를 받은 독립 검사도 있었지만 모든 셸의 문제가 해결됐다고 기록하지 않습니다. 코드를 임의로 패치하거나 개인 `.bashrc`·`.wslconfig`·진단 XML을 업로드하지 않았습니다.

## 다음 학습

현재는 **C1 실제 발행 이후 구독 코드 실행·거리값 해석** 단계입니다. 다음 순서는 기존 계획의 기본 bringup → IMU 선정·연결·축·편향 검증 → 바퀴+IMU EKF → 실제 바닥 거리·회전 보정 → SLAM 지도 → 위치 추정·Nav2입니다. 현재 display.launch는 모델 표시 전용이며 통합 bringup·IMU·EKF·SLAM·Nav2를 완료한 것은 아닙니다.

학습에서는 핵심·논리·삽입 위치와 들여쓰기·실행 순서를 먼저 안내하고, 사용자가 작성한 결과를 검사한 뒤 다음으로 넘어갑니다. 질문으로 나온 Gazebo·가상 주행·DMA 등을 임의로 필수 단계에 추가하지 않습니다. 장비 보유는 hardware README에서 먼저 확인하고 현재 연결·검증 상태는 별도로 확인합니다.

## 공식 출처

- [Humble TF Listener 튜토리얼 원본](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.rst)
- [Odometry](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg), [LaserScan](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg)
- [SLAMTEC sllidar_ros2](https://github.com/Slamtec/sllidar_ros2), [C1 launch](https://github.com/Slamtec/sllidar_ros2/blob/main/launch/sllidar_c1_launch.py)
- [별도 공식 rplidar_ros ROS2 브랜치](https://github.com/Slamtec/rplidar_ros/tree/ros2): 이번에 실제 빌드·시험한 것은 sllidar_ros2
- [C1 제조사 데이터시트](https://files.waveshare.com/wiki/RPLIDAR-C1/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf), [사용 설명서](https://files.waveshare.com/wiki/RPLIDAR-C1/SLAMTEC_rplidarkit_usermanual_C1_v1.0_en.pdf)
- [WSL USB](https://learn.microsoft.com/en-us/windows/wsl/connect-usb), [WSL 네트워크](https://learn.microsoft.com/en-us/windows/wsl/networking)
- [Microchip PI 제어 설명](https://onlinedocs.microchip.com/oxy/GUID-69278EBA-9F00-4CEE-8103-2998236856BD-en-US-2/GUID-68C13886-A5D4-4274-9410-D1AE3D79CFF9.html)
