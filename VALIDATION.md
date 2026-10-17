# 검증 기록

작성일: 2026-10-18. 모터를 직접 구동하거나 보드에 업로드하지 않았습니다.

## 완료

| 검사 | 결과 |
| --- | --- |
| ARM GCC 14.3, 새 STM32 프로젝트 전체 컴파일·링크 | ELF·BIN 생성 성공 |
| `python -m unittest discover -s tests -v` | 21건 통과 (50% 듀티 상태 수신 검사 포함) |
| `python -m compileall -q jetson tools tests` | 통과 |
| Git Bash `bash -n` | Jetson 셸 스크립트 4개 통과 |
| PowerShell 파서 | Windows SSH 실행 스크립트 통과 |
| XML 파서 | ROS package.xml과 CubeIDE 프로젝트 메타데이터 통과 |
| ELF 심볼 확인 | USART2 IRQ, 모터 제어, 통신 처리, IWDG 함수 포함 |

ELF 위치: `build/firmware/robot_test_f446re.elf`. 빌드 시 ST 제공 flash_ex.c 내부의 미사용 Banks 인자 경고 3건이 있습니다. 작성한 제어·통신 소스에서는 해당 빌드의 경고가 없습니다.

Python 검사는 ROS와 USB 외부 경계를 테스트용 객체로 대체한 로컬 검사이며, 실제 Jetson에서 rclpy·DDS·USB를 검증한 결과는 아닙니다. Python 도구 MCP의 자동 승인 정책이 메타데이터 조회를 거부하여 로컬 Python CLI로 검증했습니다. Ruff는 로컬에 설치되지 않아 실행하지 않았습니다.

### 검사한 동작

- 전진·후진·양의 yaw 좌회전 변환, 좌우 모터 매핑
- 선속도와 각속도 동시 명령 시 각 바퀴 목표 ±200mm/s 제한
- NaN/Inf·미설정 바퀴 간격 거부
- TurtleBot3 기준 키 증분, 출력 접근, 정지 및 상한
- CRC 표준 벡터 123456789 → 0x29B1, 정상 응답·손상·잘린 프레임·범위 오류
- 기본 구동 잠금, 오래된 cmd_vel 정지, 피드백 단절 시 정지
- 피드백 회복만으로 이전 이동 명령을 재사용하지 않음
- USB 오류 시 연결 잠금, 초과 길이 수신 이후 줄 경계 재동기화

## 실제 실행 전 필요한 확인

1. Jetson SSH 주소, 설치된 ROS 2 배포판 확인(현재 Humble 기준)
2. 모터 1의 좌우 위치와 좌우 바퀴 중심 간격 실측 후 robot.yaml 설정
3. Jetson에서 colcon 빌드와 실제 ST-LINK VCP 장치·포트 권한 확인
4. 새 STM32 ELF 업로드 후 구동 잠금 상태의 상태 수신 확인
5. 바퀴를 띄워 두 방향·정지·USB 단절·SSH 종료 동작 확인
6. 이후 바닥 시험, 실제 속도 및 최종 하중에서의 PI 이득 확인

200mm/s는 목표 속도의 제한입니다. 실제 속도의 과도 응답·슬립과 물리적 정지 거리는 실기 검증이 필요합니다. 사용자 요청으로 robot_test의 PWM 상한과 anti-windup 상한을 50%로 변경했고, Jetson도 50%까지의 듀티 피드백을 수신하도록 맞췄습니다. 기존 학습 프로젝트의 20% 상한은 수정하지 않았습니다.

## 기존 학습 프로젝트 보존

새 프로젝트는 `robot_test/firmware/robot_test_f446re`에 만들었습니다. 기존 STM32CubeIDE/workspace_2.2.0/encoder_basic의 소스·설정에 쓰기 작업을 수행하지 않았고, 기존 프로젝트에 새 펌웨어를 업로드하지 않았습니다.
