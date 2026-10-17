# robot_test ROS 2 패키지

ROS 2 Humble / Python 3.10 / pyserial 기준입니다.

작업공간 `robot_ws`에서 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_test
source install/setup.bash
ros2 launch robot_test robot.launch.py
```

기본 실행은 구동 잠금입니다. 설정 파일의 실제 포트·바퀴 간격·모터 좌우 배치를 지정하고, 실제 시험 때만 `enable_motors:=true`를 전달합니다. 키보드는 별도 대화형 SSH 터미널에서 `ros2 run robot_test teleop_keyboard`로 실행합니다.

설치와 SD카드 복사, 전체 시험 순서는 상위 `jetson/README.md`를 참고하세요. 소스 내부는 ROS I/O(`serial_bridge.py`, `teleop_keyboard.py`), 순수 계산(`control.py`), CRC 통신(`protocol.py`)으로 구성됩니다.
