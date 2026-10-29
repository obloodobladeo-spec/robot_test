# 외부 코드와 참고 자료

- `learning/ros2_ws/src/sllidar_ros2`: SLAMTEC 공식 저장소 `https://github.com/Slamtec/sllidar_ros2`의 커밋 `34300099fadfc772965962dec837bf436706188f`를 원본 그대로 보관했습니다. 중첩 Git 메타데이터·캐시·무시 설정은 제외했으며 원본 LICENSE와 SDK 파일별 저작권 고지는 유지했습니다. 루트 LICENSE로 외부 코드의 조건을 덮어쓰지 않습니다.
- `learning/stm32/*/Drivers`와 생성 파일: 원본 STM32Cube HAL/BSP·CMSIS 코드와 기존 라이선스·저작권 고지를 보존했습니다. 각 프로젝트의 Drivers 하위 라이선스를 따릅니다.
- `learning/study_notes`·`learning/hardware`: 사용자의 기존 학습·시험 기록 복사본입니다. 기록에 포함된 외부 문서 링크는 해당 문서의 권리를 변경하지 않습니다.

- `firmware/robot_test_f446re/Drivers`: 기존 encoder_basic의 STM32CubeF4 HAL/BSP 및 CMSIS를 복사했습니다. UART/IWDG 모듈은 로컬 STM32Cube_FW_F4_V1.28.3에서 추가했습니다. 각 파일의 기존 저작권 고지와 해당 하위 폴더의 LICENSE.txt를 유지합니다.
- STM32 생성 main/MSP/IRQ/startup/system 파일은 기존 STMicroelectronics 고지를 보존하며 사용자 영역에 robot_test 연결 코드를 추가했습니다.
- Python ROS 패키지와 motor_control/robot_link 및 빌드 스크립트는 이 작업에서 작성했습니다.
- teleop은 ROBOTIS TurtleBot3 Humble의 키 배치·증분·출력 접근 방식을 참고한 독립 구현입니다. 원본 소스를 복사하여 포함하지 않았습니다. 공식 출처는 README에 표시했습니다.
- ROS 2와 pyserial은 Jetson 설치 환경에서 제공하며 이 폴더에 바이너리를 재배포하지 않습니다.
