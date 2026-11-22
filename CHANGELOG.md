# Changelog

형식은 [Keep a Changelog](https://keepachangelog.com/ko/1.1.0/)를 따르고, 버전 규칙은 [docs/VERSIONING.md](docs/VERSIONING.md)를 따른다.

## [Unreleased]

실기 시험 중인 작업은 `test/*` 브랜치에 있다. 현재: `test/imu-drive-tuning` (BNO085 IMU 통합, 주행 보정, EKF).

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
