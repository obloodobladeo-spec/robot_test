# Changelog

형식은 [Keep a Changelog](https://keepachangelog.com/ko/1.1.0/)를 따르고, 버전 규칙은 [docs/VERSIONING.md](docs/VERSIONING.md)를 따른다.

## [Unreleased]

실기 시험 중인 작업은 `test/*` 브랜치에 있다. 현재: `test/imu-drive-tuning`. 요약: [learning/DRIVE_TUNING_2026-11-22.md](learning/DRIVE_TUNING_2026-11-22.md)

### Added (test/imu-drive-tuning)
- 펌웨어 `learning/stm32/board_practice`: BNO085 IMU(CEVA SH-2) 통합, telemetry 100 ms·IMU·순번 필드, heading hold(엔코더 P + IMU P/I), 속도 비례 피드포워드, IMU yaw 배율 보정.
- ROS: `/imu/data` 발행, covariance, 20 ms 수신 타이머, `publish_tf` 파라미터, robot_localization EKF(`use_ekf:=true`, `config/ekf.yaml`).
- 시험 도구 `learning/tools/drive_test`와 원본 로그 `learning/study_notes/assets/drive_tests/2026-11-21_22`.

- Jetson 준비: `learning/tools/jetson/99-robot-usb.rules`(`/dev/stm32`, `/dev/lidar`), `setup_jetson.sh`.
- `slam.launch.py` + `config/slam_toolbox.yaml` (online async, base_frame base_link). bringup에 `stm32_port`/`lidar_port` 인자.

- Jetson 이전 완료 (서버 모드, 도메인 20, Wi-Fi 절전 끔), RViz `rviz/bringup.rviz`, 도구 `scan_direction.py`·`topic_jitter.py`, Jetson 배터리 주행 로그.

### Changed (test/imu-drive-tuning)
- 2026-11-23: 바퀴 둘레 L 345.6 → 343.9, R 330.7 → 332.3 mm (LiDAR 스캔 매칭 근거), heading hold 게인을 RAM 설정 `robot.heading`으로.
- 바퀴 PI Kp/Ki 0.15/0.15 → 0.15/0.25, 오른쪽 바퀴 둘레 345.6 → 330.7 mm, 실효 윤거 234 → 243.1 mm, IMU yaw × 1.0053.
- 방향 전환 시 이미 정지해 있던 바퀴는 대기 없이 출발.
- URDF: 차체 220×160×72 mm, 앞바퀴 제거, LiDAR 받침 80×132×44 mm, `imu_link` 추가.

## [0.1.1] - 2026-11-22

### Added
- 브랜치·버전 규칙 [docs/VERSIONING.md](docs/VERSIONING.md), 이 CHANGELOG, `.gitignore`.

### Removed
- 추적 중이던 Python 캐시(`__pycache__/*.pyc`) 10개를 git 추적에서 제외 (생성물).

### Fixed
- 깨진 문서 링크 2개 (`2026-10-19.md`의 encoder_basic 링크, `stm32-timer-pwm/README.md`의 전체 학습 목록 링크).

## [0.1.0] - 2026-11-21

기존 main 상태에 붙인 첫 태그 (커밋 `ef295a5`).

- 학습 펌웨어 `learning/stm32/encoder_basic` (UART `V=… W=…`, PI Kp/Ki 0.15, CPR 2473.6/2464.5, 윤거 234 mm).
- 디버거 전용 시험 프로젝트 `stm32_projects/encorder_test` (Kp/Ki 0.1, 50 mm/s 피드포워드 12%).
- ROS 2 `learning/ros2_ws/src` (`motor_bridge`, `robot_description`, `sllidar_ros2`).
- 모터 튜닝·5회 주행 기록 (`learning/MOTOR_PROGRESS_2026-11-21.md`).
- 기존 독립 시험 패키지(CRC 프레임): `firmware/`, `jetson/`, `build/firmware/`, `tools/`, `tests/`.
