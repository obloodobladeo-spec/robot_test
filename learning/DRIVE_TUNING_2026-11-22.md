# IMU 통합·주행 보정 기록 — 2026-11-21 ~ 11-22 (테스트 브랜치)

브랜치 `test/imu-drive-tuning`. 실기 시험 중인 상태이며 값은 바뀔 수 있다. 검증이 끝나면 main에 병합한다.

- 펌웨어: [learning/stm32/board_practice](stm32/board_practice/) (encoder_basic + BNO085 IMU). 상세 변경 이력은 [board_practice/README.md](stm32/board_practice/README.md).
- ROS 2: [learning/ros2_ws/src](ros2_ws/src/) (`motor_bridge`, `robot_description`).
- 시험 도구: [learning/tools/drive_test](tools/drive_test/), 원본 로그: [study_notes/assets/drive_tests/2026-11-21_22](study_notes/assets/drive_tests/2026-11-21_22/).

## 하드웨어 구성 (시험 당시)

| 항목 | 값 |
|---|---|
| MCU / 드라이버 / IMU / LiDAR | NUCLEO-F446RE / MDD10A / GY-BNO085 (I2C 100 kHz) / RPLIDAR C1 |
| 차체 | 220 × 160 × 72 mm, 구동축은 차체 앞 끝에서 183 mm 뒤 |
| 캐스터 | 바퀴형 2개 → 바퀴형 1개 → **볼 캐스터 1개 (앞 가운데)** |
| 하중 | 최종 하중(Jetson 등) 탑재 |
| IMU 장착 | 기울이기 시험으로 IMU 축 = 로봇 축 확인 (+X 앞, +Y 왼쪽, +Z 위) |

## 최종 값 (2026-11-22)

| 항목 | 이전 (main v0.1.x) | 현재 | 근거 |
|---|---|---|---|
| telemetry 주기 | 1000 ms | **100 ms** | slam_toolbox·Nav2에 10 Hz 이상 odom 필요 |
| telemetry 형식 | `L R X Y A` | `+ T N H IS [QX QY QZ QW GX GY GZ QA GA]` | IMU·순번·heading hold 보정값 추가 (ROS는 모르는 key 무시) |
| 바퀴 PI | Kp 0.15 / Ki 0.15 | **Kp 0.15 / Ki 0.25** | 50 cm 왕복 최종 방향 ±0.15° |
| 피드포워드 | 없음 (test: 50 mm/s 전용 12%) | **7.5% + 0.097 × \|제어 목표 속도\|**, 모든 명령 | 바닥 고정 듀티 시험 9~14% |
| 출발 시 적분 | — | 가속 중 양의 적분 누적 억제 | 출발 과속 방지 |
| 방향 전환 대기 | 매번 0.3 s 정지 확인 | 이미 0.3 s 멈춰 있으면 바로 출발 | 직전 동작에 따라 한쪽 바퀴만 늦게 출발하던 문제 |
| heading hold (W=0 직진) | 없음 | P: 3.0 × 엔코더 오차 + 2.0 × (IMU−엔코더) 오차, I: 1.0 × IMU 오차, 최대 ±0.15 rad/s | 직진 시험 |
| 오른쪽 바퀴 둘레 | 345.6 mm | **330.7 mm** (왼쪽 345.6) | 50 cm 왕복: 전진·후진 모두 엔코더가 IMU보다 약 5° 더 돎 |
| IMU yaw 배율 | 없음 | **× 1.0053** (yaw 변화량·gz) | 바닥 테이프 실측 360° × 5: 실제/IMU = 1.0053 ± 0.0020 |
| 실효 윤거 | 234 mm | **243.1 mm** | 보정한 IMU 기준 제자리 360° × 5 (σ 0.6 mm) |
| 클럭 | HSI | HSE Bypass(ST-LINK MCO) 시도, 실패 시 HSI | encorder_test 설정 |
| ROS 수신 주기 | 100 ms 송신 타이머에서 같이 읽음 | 20 ms 별도 타이머 | 발행 간격 σ 19.4 → 3.0 ms |
| `/odom`·`/imu/data` covariance | 비어 있음 | 실측 근거 상수 | EKF 입력 |
| EKF | 없음 | `bringup.launch.py use_ekf:=true` (robot_localization) | yaw는 IMU, 거리는 엔코더를 따름 확인 |
| URDF | 250×180×56.4 차체, 앞바퀴 2 | 220×160×72, 앞바퀴 없음, LiDAR 받침 80×132×44, `imu_link` 추가 | 실측 |

## 주요 시험과 결과

1. **IMU 통합**: Game Rotation Vector + Calibrated Gyro 50 Hz, I2C 오류 0. 정지 yaw 노이즈 약 0.005°, 드리프트 0.1~0.3°/min.
2. **출발 꿈틀거림**: 출발 듀티는 좌우·전후진 약 9.3%(8.8~10%). 피드포워드 교체 후 30 mm/s 도달 1.4~2.4 s → 0.6~0.7 s.
3. **휘는 방향이 매번 달랐던 원인**: 방향 전환 대기(0.3 s)가 직전에 반대로 돌던 바퀴에만 걸림. 수정 후 두 바퀴 동시 출발 10/10.
4. **오른쪽 쏠림**: 캐스터 종류와 무관하게 오른쪽 엔코더가 약 4% 더 셈 → 오른쪽 바퀴 둘레 보정 후 엔코더−IMU 차이 0.5° 이하.
5. **IMU 배율**: 회전을 약 0.5% 작게 잼. BNO085는 SH-2 매뉴얼 v1.2에 자이로 게인 자체 보정 명령이 없어 MCU에서 보정. 보정 후 1080° 회전에서 실측과 1° 안팎.
6. **볼 캐스터 1개 50 cm 왕복** (최종 설정): 전진 −0.14° ± 0.16, 후진 −0.19° ± 0.15.

## Jetson 이전 (2026-11-22)

| 항목 | 상태 |
|---|---|
| Jetson | Orin Nano 8GB, Ubuntu 22.04.5, L4T R36.5, ROS 2 Humble, 사용자 `heosi`, 노트북에서 `ssh jetson`(키 로그인) |
| 설치 | `learning/tools/jetson/setup_jetson.sh` (robot_localization, slam_toolbox, nav2, udev) |
| 장치 이름 | `/dev/stm32`(ST-LINK 0483:374b), `/dev/lidar`(CP2102N 10c4:ea60) |
| 부팅 | 서버 모드(`multi-user.target`). 메모리 bringup 중 1,753 → 622 MB, gnome-shell CPU 37% 제거 |
| 네트워크 | iPhone 핫스팟, `ROS_DOMAIN_ID=20`(Jetson·노트북 모두). **Jetson Wi-Fi 절전 끔**(`/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf`: `wifi.powersave = 2`) → ping 평균 59 → 4 ms, 최대 293 → 6 ms |
| bringup (`use_ekf:=true`) | `/scan`, `/odom`, `/imu/data`, `/odometry/filtered` 모두 10 Hz, TF 정상, 노트북에서 수신 확인 |
| LiDAR 방향 | `laser` yaw 180° 실물과 일치 확인 (`tools/drive_test/scan_direction.py`, RViz `rviz/bringup.rviz`) |
| 배터리 주행 | 50 cm 왕복 3회: 전진 −0.07° ± 0.26, 후진 −0.09° ± 0.45, 튀는 값 0. 엔코더−IMU가 −0.9°/+0.8°로 바뀜(Jetson·배터리 탑재로 하중 변화, 오른쪽 둘레 약 333 mm로 재보정 후보) |

## 남은 항목 (main 병합 전 확인 필요)

- 바퀴 속도 흔들림 σ 5~6 mm/s (비틀거림): 제어 설정과 무관, 기계 요인으로 추정. 미해결.
- 볼 캐스터 높이 조정 (pitch 약 −1.7°), 이후 윤거·직진 재확인.
- 시계 방향 IMU 배율 확인 (반시계 5회로만 결정).
- LiDAR `/scan` 방향(yaw 180°)·위치 실물 확인, slam_toolbox, Jetson 이전, Nav2.
- 시험은 노트북 USB 연결 상태에서 수행 (USB 선 꼬임으로 일부 회전 측정 무효 처리, `discard/`).
