# 주행 시험 도구 (ROS 2 Humble, 노트북/Jetson)

`motor_bridge cmd_vel_receiver`가 실행 중이어야 한다. 모터가 실제로 움직이므로 주변을 비우고 실행한다.
명령이 끊기면 펌웨어가 0.5 s 뒤 정지한다.

| 파일 | 용도 |
|---|---|
| `straight_trials.py` | 전진 30 cm → 출발점까지 후진 → IMU로 방향 정렬, N회 |
| `shuttle_run.py` | [정렬 → 전진 → 정렬 → 후진] N회, 거리·속도 지정, 각속도 0.001이면 heading hold 끔 |
| `segment_run.py` | 정렬 후 30 cm씩 이어서 전진 (되돌아오지 않음) |
| `rot_enc.py` | 엔코더(/odom) 기준 목표 각도까지 제자리 회전, IMU와 비교 → 실효 윤거 |
| `imu_rot_swd.py` | MCU `g_imu.yaw_rad`를 SWD로 직접 읽으며 목표 각도에서 감속·정지 (IMU 배율 실측용) |
| `swd_param.py` | 실행 중 MCU RAM의 바퀴 둘레·CPR·피드포워드·PI·윤거 읽기/쓰기 (재부팅하면 원래 값) |
| `level_check.py` | 정지 상태 roll/pitch 평균 |
| `trials_analyze.py`, `shuttle_analyze.py`, `shuttle_brief.py`, `wheel_noise.py` | 로그 분석 |

SWD 도구는 STM32CubeIDE 2.2.0 동봉 `STM32_Programmer_CLI`·`arm-none-eabi-nm`과
`/home/kim/stm32_project/board_practice/Debug/board_practice.elf` 경로를 가정한다. 다른 PC에서는 경로를 고친다.
로그는 [study_notes/assets/drive_tests/2026-11-21_22](../../study_notes/assets/drive_tests/2026-11-21_22/)에 있다.
