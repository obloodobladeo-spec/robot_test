# 로봇 통합 구성

기준일: 2026-10-23. 목표는 키보드 주행에서 시작하여 위치 추정·지도 작성·자율주행으로 이어지는 후륜 차동구동 로봇입니다. 이 문서는 구성 계획이며, 통합 실행 완료를 의미하지 않습니다.

## 1. 장치별 역할

| 장치 | 맡을 기능 | 현재 상태 |
|---|---|---|
| NUCLEO-F446RE | 엔코더 측정, 좌우 모터 PI 속도 제어, PWM/DIR, 가감속 제한, 통신 단절 정지, 바퀴 오도메트리 계산 | 학습 프로젝트에서 구현·시험 |
| MDD10A | MCU의 PWM/DIR에 따라 두 모터 전력 구동 | 사용 중 |
| Jetson Orin Nano | ROS 2, 시리얼 브리지, 라이다·IMU 드라이버, 센서 융합, SLAM, Nav2 | 통합 검증 예정 |
| RPLIDAR C1 | 주변 거리 측정, `/scan` 발행 | Windows 기본 동작 확인, ROS 2 연결은 미검증 |
| 외장 IMU | 회전 각속도 등 관성 측정, 엔코더와 함께 위치 추정 보완 | 모델·접속 방식·장착 위치 미정 |
| Windows PC | SSH 접속, 키보드 조작, 개발·디버깅, 필요 시 RViz | 개발 환경 |

OpenCR의 모터 제어·센서 전달 역할을 STM32와 ROS 브리지로 나누어 구현합니다. OpenCR의 DYNAMIXEL 제어나 보드 내장 IMU·전원 회로를 그대로 복제하는 구성은 아닙니다. NUCLEO-F446RE에는 이 계획에서 사용할 IMU가 내장되어 있지 않으므로 외장 센서를 선정해야 합니다.

## 2. 데이터 흐름 — 계획

```text
Windows 키보드/SSH
  → Jetson teleop → /cmd_vel
  → STM32 브리지 → USB ST-LINK VCP / USART2
  → STM32 좌우 목표 속도 → PI → PWM/DIR → MDD10A → 모터
                                      ↑                    │
                                      └──── 엔코더 피드백 ──┘

STM32 바퀴 오도메트리 → 브리지 → /wheel/odometry ─┐
외장 IMU → IMU 드라이버 → /imu/data ─────────────┤
                                             → EKF → /odometry/filtered
C1 → sllidar_ros2 → /scan → SLAM/위치 추정 → Nav2 → /cmd_vel
```

토픽 이름은 통합 시 사용할 예정 이름입니다. IMU를 STM32에 연결할지 Jetson에 직접 연결할지는 센서 선정 후 결정합니다. STM32 경유 시 센서 메시지 전달과 타임스탬프도 통신 규약에 포함합니다.

## 3. 좌표계와 TF 담당

| 변환 | 담당 | 조건 |
|---|---|---|
| `map → odom` | SLAM 또는 AMCL | 지도 작성/위치 추정 단계에서 하나만 사용 |
| `odom → base_link` | `robot_localization` EKF | 융합 사용 시 유일한 발행자 |
| `base_link → laser` | URDF와 `robot_state_publisher` | 실제 라이다 장착 위치·방향 반영 |
| `base_link → imu_link` | URDF와 `robot_state_publisher` | 실제 IMU 장착 위치·방향 반영 |

EKF 적용 전 단독 바퀴 오도메트리 시험에서는 브리지가 `odom → base_link`를 발행할 수 있습니다. EKF 적용 시 브리지의 해당 TF 발행을 끕니다. 두 노드가 같은 변환을 동시에 발행하지 않도록 launch 설정으로 선택합니다.

현재 차동구동 계산 기준점은 **후륜 구동 바퀴 두 개의 축 중간점**입니다. `base_link`도 우선 이 기준에 맞추며 몸체 외형 중심과의 차이는 URDF에 반영합니다. ROS 좌표는 전방 +X, 좌측 +Y, 위쪽 +Z를 기준으로 정렬합니다.

엔코더에서 계산한 위치와 속도는 같은 측정에서 나온 값이므로 모두 독립 측정처럼 중복 융합하지 않습니다. 바퀴 속도와 IMU Z축 각속도 등을 시작 후보로 두고, 실제 센서 제공 항목·공분산·장착 방향을 확인한 뒤 EKF 설정을 정합니다. IMU도 편향과 드리프트가 있으며, 추가만으로 미끄러짐이나 위치 오차가 사라지지는 않습니다.

## 4. 통신과 현재 구현의 차이

| 항목 | 현재 학습 `encoder_basic` | 이 저장소의 기존 독립 패키지 |
|---|---|---|
| 위치 | 별도 STM32CubeIDE 워크스페이스 | `firmware/robot_test_f446re` |
| 명령 | `V=<mm/s 정수> W=<mrad/s 정수>\n` | 별도 CRC 기반 프로토콜 |
| 피드백 | 약 1초마다 `L=... R=... X=... Y=...` | 기존 브리지용 상태 메시지 |
| ROS 연결 | 학습용 규약에 맞춘 브리지 필요 | `jetson/robot_ws/src/robot_test`에 기존 브리지 존재 |
| 현재 검증 | STM32 실기·PC 시리얼 시험 | [기존 검증 기록](../VALIDATION.md), 실제 Jetson 통합 미검증 |

기존 브리지에 학습 펌웨어를 바로 연결하지 않습니다. 통합 구현 때 하나의 통신 규약을 선택하고 송수신 양쪽과 문서를 함께 맞춥니다. 현재 학습 순서를 이어갈 때는 학습용 ASCII 명령을 출발점으로 삼습니다.

다음 통신 확장에는 좌우 실제 속도, x/y/yaw, 측정 간격 또는 MCU 시각, 순번을 포함할 계획입니다. 피드백은 우선 100ms 주기를 목표로 하며, 현재 정수 출력의 정밀도도 함께 개선합니다. MCU 시간과 ROS 시간을 그대로 같은 시각으로 취급하지 않고 수신 시각·측정 지연·시간 대응 방식을 정합니다.

## 5. 제어 한계와 정지 책임

- 바퀴 목표 속도 한계: 절댓값 200mm/s. 좌우 속도가 한계를 넘으면 비율을 유지하여 함께 축소합니다.
- PWM 듀티 상한: 50%. 실제 속도는 부하·배터리·마찰에 따라 달라집니다.
- 학습 명령의 입력 허용 범위는 선속도 ±200mm/s, 각속도 ±2000mrad/s입니다. 실제 주행용 teleop 한계는 별도로 낮게 설정합니다. 기존 독립 패키지의 각속도 한계는 0.5rad/s입니다.
- Jetson은 새 `/cmd_vel`이 일정 시간 들어오지 않으면 정지 명령을 보냅니다. 마지막 주행 명령을 계속 재전송하여 유효하게 만들면 안 됩니다.
- STM32는 마지막 **유효 명령** 이후 500ms가 지나면 목표 속도를 0으로 만듭니다. 현재 100ms 제어 주기를 고려하면 반영까지 추가 지연이 생길 수 있고, 바퀴의 기계적 정지는 더 늦을 수 있습니다.
- UART 오류 복구, 재접속 후 정지 상태 유지, 잘못된 메시지·버퍼 초과·피드백 단절도 통합 시험 항목으로 둡니다.

## 6. 패키지 분리안 — 아직 생성하지 않음

기존 `robot_test` 패키지를 유지하면서 기능이 구현되는 시점에 아래 역할로 분리합니다.

```text
jetson/robot_ws/src/
  robot_bridge/        # /cmd_vel ↔ STM32, /wheel/odometry, 상태
  robot_description/   # URDF/xacro, 센서 장착 위치
  robot_bringup/       # 통합 launch, 장치·제어·센서 설정
  robot_localization_config/ # IMU/바퀴 EKF 설정
  robot_navigation/    # 나중 단계: SLAM/Nav2 설정
```

`sllidar_ros2`, IMU 드라이버, `robot_localization`은 해당 공식 패키지를 사용합니다. 사용자 전용 IMU 드라이버는 기존 드라이버 지원 여부를 확인한 뒤에만 검토합니다.

`robot_bringup`은 브리지·로봇 모델·C1·IMU·EKF를 설정에 따라 묶어 실행하는 역할입니다. 키보드 조작, RViz, SLAM, Nav2는 별도 launch로 시작할 수 있게 구성합니다. 필수 센서가 준비되지 않은 시험에서는 명시적인 옵션으로 끄며, 실행하지 않은 기능을 정상 상태로 표시하지 않습니다.

## 7. 하드웨어 기준과 미확정 항목

현재 기록: 모터 1 왼쪽, 모터 2 오른쪽, 후륜 구동, 지름 110mm, 엔코더 1회전당 모터 1 약 2465.2 / 모터 2 약 2475.6카운트, 바퀴 간격 최근 측정 230mm입니다. 차체 축소안 250×180 → 220×160mm는 제안 상태이며 변경 후 실제 바퀴 간격을 다시 측정해야 합니다.

전방 보조 바퀴의 횡마찰, 테이프에 따른 미끄러짐, 출력축·3D 프린트 허브 결합, 적재 무게가 회전과 속도에 영향을 줍니다. 센서 융합과 PI 조정으로 기계적 결합 문제를 대신 해결하지 않습니다.

확정할 항목: 최종 차체·바퀴 간격, 센서 위치, IMU 모델/전압/인터페이스, Jetson ROS 2 환경, 실제 USB 장치 경로. 연결 경로는 가능하면 `/dev/serial/by-id/`로 고정합니다.

## 공식 참고 자료

- [TurtleBot3 Humble bringup](https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble/turtlebot3_bringup/launch/robot.launch.py): 기본 노드와 라이다 실행 구성 비교.
- [OpenCR 공식 설명](https://emanual.robotis.com/docs/en/parts/controller/opencr10/): 보드 기능과 내장 센서 비교.
- [SLAMTEC 공식 ROS 2 드라이버](https://github.com/Slamtec/sllidar_ros2): C1 지원 드라이버와 실행 안내. C1은 LDS-03용 `coin_d4_driver`와 구분합니다.
- [robot_localization 설정 안내](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst): 중복 측정과 센서 입력 선택 원칙. 실제 구현은 설치한 ROS 2 배포판 버전에 맞춰 확인합니다.
- [Nav2 오도메트리 설정 안내](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html): EKF와 `odom → base_link` 구성 참고.
- [REP-103](https://www.ros.org/reps/rep-0103.html), [REP-105](https://www.ros.org/reps/rep-0105.html): 단위·좌표축·프레임 기준.
