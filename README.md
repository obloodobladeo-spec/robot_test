# robot_test

Jetson Orin Nano와 NUCLEO-F446RE로 두 바퀴 로봇을 키보드 조작하는 최소 실행 구성입니다. 기존 STM32 학습 워크스페이스를 복사한 독립 프로젝트이며 기존 `encoder_basic` 소스는 수정하지 않았습니다.

```text
Windows SSH 터미널
  └─ Jetson teleop → /cmd_vel (Twist)
       └─ Jetson serial_bridge → USB ST-LINK VCP / USART2
            └─ STM32: 엔코더 측정 + 두 모터 PI + PWM/DIR
```

## 폴더

```text
robot_test/
├── firmware/robot_test_f446re/   # Windows CubeIDE 프로젝트, Core·Drivers·ioc 포함
├── build/firmware/              # 바로 업로드할 ELF·BIN 및 빌드 중간 파일
├── jetson/                      # 이 폴더 전체만 SD카드로 Jetson에 전달
│   ├── README.md                # 설치·설정·SSH 실행 순서
│   ├── scripts/                 # setup/build/driver/teleop
│   └── robot_ws/src/robot_test/  # ROS 2 Python 패키지
├── windows/Start-RobotSsh.ps1    # 이 PC에서 SSH 터미널 실행
├── tools/build_firmware.py      # 독립 ARM GCC 빌드, 업로드 없음
└── tests/                       # 하드웨어 없는 회귀 검사
```

Jetson ROS 2 배포판, SSH 주소, 바퀴 간격 및 모터 1의 좌우 위치는 아직 사용자 확인 전입니다. 기본 설정에서는 모터 출력 명령을 잠그며 ROS 2 Humble을 실행 기준으로 사용합니다.

## 처음 사용하는 분을 위한 실행 순서

### 1단계: 이 저장소를 Windows에 받기

GitHub 저장소 페이지에서 **Code → Download ZIP**을 누르고 압축을 풉니다. Git을 사용할 수 있으면 다음 명령으로 받아도 됩니다.

```powershell
git clone https://github.com/obloodobladeo-spec/robot_test.git
cd robot_test
```

이 저장소는 사용자 요청에 따라 **.gitignore 없이 전체 파일을 보관**합니다. STM32 소스·드라이버뿐 아니라 빌드 결과 ELF/BIN, 중간 파일, 현재 생성된 Python 캐시도 포함합니다. 펌웨어만 업로드하려면 아래 ELF를 사용하고, Jetson으로는 `jetson` 폴더를 옮기면 됩니다.

### 2단계: STM32에 펌웨어 넣기 — Windows에서 수행

1. 모터 배터리를 분리합니다. Nucleo는 USB로 Windows에 연결합니다.
2. STM32CubeProgrammer를 열고 오른쪽 연결 방식을 **ST-LINK**로 선택합니다.
3. **Connect**를 누릅니다. 보드가 안 보이면 데이터 통신 가능한 USB 케이블과 ST-LINK 커넥터를 확인합니다.
4. **Erasing & Programming** 화면에서 파일 선택 버튼으로 `build/firmware/robot_test_f446re.elf`를 고릅니다.
5. **Verify programming**을 선택하고 **Start Programming**을 누릅니다. ELF에는 주소 정보가 있으므로 제공된 ELF를 그대로 선택합니다.
6. 완료 후 연결을 종료하고 보드의 RESET 버튼을 누릅니다.

이미 빌드된 ELF는 PWM 상한 50%, 각 바퀴 목표 속도 상한 ±200mm/s 설정입니다. 소스를 변경했다면 먼저 아래의 빌드 명령으로 ELF를 다시 생성해야 합니다. CubeProgrammer는 별도 설치 프로그램이며 이 저장소에 포함되어 있지 않습니다.

### 3단계: Jetson에 파일 옮기기 — SD카드 사용

1. Windows에서 이 저장소 안의 **jetson 폴더 전체**를 이동용 SD카드에 복사합니다.
2. SD카드를 Jetson에 연결합니다. Jetson에서 파일 관리자로 카드가 보이는지 확인합니다.
3. Jetson 홈 폴더에 `robot_test` 폴더를 만들고 그 안에 `jetson` 폴더를 복사합니다.
4. 최종 경로가 **`~/robot_test/jetson/scripts/setup.sh`**인지 확인합니다.

`~`는 Jetson에서 로그인한 사용자의 홈 폴더입니다. Jetson이 현재 부팅에 사용하는 운영체제 SD카드는 포맷하지 마세요. 파일 전달용 카드의 읽을 수 있는 데이터 영역으로 옮기면 됩니다.

### 4단계: Jetson ROS 2 준비와 패키지 빌드

Jetson에서 터미널을 열어 다음을 실행합니다.

```bash
ls /opt/ros
```

`humble`이 표시되면 다음 명령을 실행합니다.

```bash
bash ~/robot_test/jetson/scripts/setup.sh humble
```

비밀번호를 요청하면 Jetson 로그인 비밀번호를 입력합니다. 입력 중 글자가 표시되지 않는 것은 정상입니다. 이 과정은 인터넷 연결이 필요하며 필요한 패키지를 설치하고 USB 시리얼 권한을 설정한 뒤 빌드합니다. 완료 후 **로그아웃했다가 다시 로그인**합니다.

`/opt/ros/humble`이 없으면 먼저 [ROS 2 Humble 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)에 따라 Ubuntu 22.04용 ROS 2를 설치하세요. JetPack 자체에 ROS 2가 자동 포함되는 것은 아닙니다. 다른 배포판이 설치되어 있다면 이름을 확인하고 호환성을 먼저 점검합니다.

### 5단계: 실제 포트·바퀴 간격·모터 위치 입력

Nucleo의 **ST-LINK USB 커넥터를 Jetson USB**에 연결한 다음 실행합니다.

```bash
ls -l /dev/serial/by-id/
nano ~/robot_test/jetson/robot_ws/src/robot_test/config/robot.yaml
```

아래 세 항목을 실제 로봇에 맞게 수정합니다.

| 항목 | 입력 방법 |
| --- | --- |
| `port` | 위 명령으로 확인한 `/dev/serial/by-id/...` 전체 경로. 또는 실제 `/dev/ttyACM0` |
| `wheel_separation_m` | 좌우 바퀴 중심 사이 실측 거리. 예: 300mm라면 `0.300` (예시값이므로 실측 후 입력) |
| `motor1_side` | 전진 방향을 바라볼 때 모터 1이 왼쪽이면 `left`, 오른쪽이면 `right` |

바퀴 **지름 110mm**와 **좌우 중심 간격**은 다른 값입니다. 미설정 상태로는 실제 구동이 활성화되지 않습니다. nano에서 **Ctrl+O → Enter**로 저장하고 **Ctrl+X**로 나옵니다.

### 6단계: Windows에서 SSH로 Jetson 접속

Jetson에서 한 번 실행합니다.

```bash
sudo apt-get install -y openssh-server
sudo systemctl enable --now ssh
whoami
hostname -I
```

`whoami`는 사용자명, `hostname -I`는 IP 주소를 보여줍니다. Windows와 Jetson을 같은 네트워크에 연결합니다. Windows PowerShell 창 두 개를 열고 각각 다음처럼 접속합니다. 사용자명과 IP는 실제 값으로 바꾸세요.

```powershell
ssh -t 사용자명@Jetson_IP
```

최초 접속 때 호스트 키 지문을 확인하고 `yes`로 승인한 뒤 Jetson 비밀번호를 입력합니다. 이 비밀번호는 GitHub 비밀번호가 아닙니다.

### 7단계: 모터가 잠긴 상태에서 통신 확인

첫 번째 SSH 창에서 다음을 실행합니다.

```bash
bash ~/robot_test/jetson/scripts/driver.sh
```

두 번째 SSH 창에서 다음을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/robot_test/jetson/robot_ws/install/setup.bash
ros2 topic echo /robot/status
```

속도와 듀티 상태가 반복 표시되고 `enabled`가 `false`이면 기본 통신 확인 단계입니다. 데이터가 없으면 USB 포트 경로·권한·STM32 펌웨어를 확인합니다. 확인 후 두 창에서 각각 **Ctrl+C**로 종료합니다.

### 8단계: 바퀴를 띄우고 키보드 시험

로봇을 고정하고 모든 바퀴를 공중에 띄운 뒤 배선·공통 GND를 확인하고 모터 배터리를 연결합니다. 두 목표가 0인 초기 상태에서 시작합니다.

첫 번째 SSH 창:

```bash
bash ~/robot_test/jetson/scripts/driver.sh enable_motors:=true
```

두 번째 SSH 창:

```bash
bash ~/robot_test/jetson/scripts/teleop.sh
```

| 키 | 동작 |
| --- | --- |
| `w` | 선속도 목표 +10mm/s |
| `x` | 선속도 목표 -10mm/s |
| `a` | 각속도 목표 +0.1rad/s, 좌회전 |
| `d` | 각속도 목표 -0.1rad/s, 우회전 |
| `s` 또는 Space | 목표 속도를 즉시 0으로 설정 |
| Ctrl+C | 종료 |

처음에는 `w`를 조금씩 눌러 방향부터 확인합니다. 1초간 유효 키 입력이 없으면 정지하며 계속 움직이려면 반복 입력합니다. 각 바퀴 목표는 ±200mm/s, PWM은 최대 50%이며 50%로 고정 구동하는 방식이 아니라 PI가 필요한 듀티를 계산합니다. 정지 명령 뒤에도 관성으로 잠시 회전할 수 있습니다.

### 9단계: 종료

1. teleop 창에서 `s` 또는 Space로 정지하고 바퀴 정지를 확인합니다.
2. teleop 창에서 Ctrl+C를 누릅니다.
3. 드라이버 창에서도 Ctrl+C를 누릅니다.
4. 모터 배터리를 분리합니다.

본 패키지는 실제 Jetson 주행 시험 전 상태입니다. 로컬 검증 범위는 [VALIDATION.md](VALIDATION.md)에 정리했습니다. 통신·방향·정지가 확인된 후 바닥 주행을 시험하세요.

## STM32 빌드와 업로드

Python 3.10 이상과 STM32CubeIDE ARM GCC가 필요합니다.

```powershell
cd C:\Users\heosi\robot_test
python tools\build_firmware.py
```

빌드 결과는 `build/firmware/robot_test_f446re.elf`와 `.bin`입니다. 다른 툴체인 위치는 `--toolchain-dir "ARM_GCC의_bin_폴더"`로 지정합니다. CubeIDE에서는 `File → Import → Existing Projects into Workspace`로 `firmware/robot_test_f446re`를 가져와 빌드·디버그할 수도 있습니다.

모터 배터리를 분리한 상태에서 **새 프로젝트의 ELF**를 업로드하세요. 기존 `encoder_basic` 디버그 구성을 실행하지 않도록 프로젝트와 ELF 이름을 확인합니다. 업로드 후 재부팅 초기 출력은 0이고 정상 USB 명령을 받아야 움직입니다. 실제 업로드와 모터 구동은 이 파일 생성 작업에서 수행하지 않았습니다.

### 핀과 통신

| 기능 | 핀 / 타이머 |
| --- | --- |
| 모터 1 PWM / DIR | PA8 TIM1_CH1 / PC0 |
| 모터 2 PWM / DIR | PA9 TIM1_CH2 / PC1 |
| 엔코더 1 A/B | PA0/PA1 TIM2, 32비트 |
| 엔코더 2 A/B | PA6/PA7 TIM3, 16비트 |
| ST-LINK VCP | PA2 TX / PA3 RX, USART2, 115200 8N1 |

두 DIR 모두 HIGH가 전진이고 엔코더 속도가 양수인 배선 기준입니다. Encoder CPR은 모터 1=2465.2, 모터 2=2475.6, 바퀴 둘레=345.6mm입니다. USB는 Nucleo ST-LINK 커넥터에 연결합니다. PA2/PA3을 Jetson GPIO에 별도로 연결하지 않습니다. STM32·드라이버 GND는 공통입니다.

공통 `motor_control.c/.h`로 두 모터의 측정·PI·방향 전환을 묶었습니다. UART ISR은 바이트만 저장하고 파싱·제어는 main에서 수행합니다. ISR 공유 링 버퍼 인덱스와 오류 플래그에만 volatile을 사용합니다.

- 제어 주기 100ms, PWM 20kHz, PWM 상한 50%
- Kp=0.2, Ki=0.1: 기존 학습값을 사용하며 최종 하중 튜닝은 미완료
- 각 바퀴 목표 ±200mm/s, 명령 초과값은 통신 파서에서 거부
- 방향 전환 전에 출력 0 및 3회 연속 엔코더 변화량 0 확인
- main 정지 시 IWDG 리셋이 있으므로 디버거 중단점에 오래 머물면 리셋될 수 있음
- fault handler는 PWM 채널과 주 출력을 직접 차단

USART2와 IWDG 초기화는 USER CODE 모듈에서 수행합니다. `.ioc`는 기존 타이머·핀 설정을 포함한 참고 원본입니다. CubeMX로 재생성하면 HAL UART/IWDG 모듈 활성화와 UART/추가 소스의 빌드 포함 여부를 재확인하세요. 제공된 빌드 스크립트는 필요한 소스를 명시합니다.

## 통신 프레임

```text
명령: V,sequence,motor1_mm_s,motor2_mm_s*CCCC\n
응답: S,sequence,motor1_mm_s,motor2_mm_s,duty1_centi,duty2_centi,timeout*CCCC\n
```

ASCII, CRC-16/CCITT-FALSE(initial=FFFF, poly=1021), `*` 앞까지 계산합니다. sequence 범위는 0~65535이며 응답에서 마지막 수신 번호를 표시합니다. 듀티는 0.01% 단위입니다. 잘못된 CRC·초과 길이·범위 오류 프레임은 명령 타임아웃을 갱신하지 않습니다.

## Jetson 실행

[jetson/README.md](jetson/README.md)의 순서대로 SD카드 복사 → 설치·빌드 → 바퀴 간격/좌우 설정 → 구동 잠금 상태에서 통신 확인 → 바퀴를 띄운 teleop 시험을 진행합니다.

Windows SSH 실행 예:

```powershell
.\windows\Start-RobotSsh.ps1 -Target 사용자명@Jetson_IP -Mode driver
# 실제 구동 단계에서만
.\windows\Start-RobotSsh.ps1 -Target 사용자명@Jetson_IP -Mode driver -EnableMotors
# 다른 PowerShell 창
.\windows\Start-RobotSsh.ps1 -Target 사용자명@Jetson_IP -Mode teleop
```

## 로컬 검사

```powershell
python -m unittest discover -s tests -v
python -m compileall -q jetson tools tests
python tools\build_firmware.py
```

검증 범위와 실제 실행하지 않은 항목은 [VALIDATION.md](VALIDATION.md)에 기록합니다. STM32 학습 프로젝트 소스는 이 패키지의 수정 대상이 아닙니다.

공식 자료: [ROBOTIS teleop](https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py), [ST Nucleo UM1724](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf), [Microchip PI 제어](https://onlinedocs.microchip.com/oxy/GUID-69278EBA-9F00-4CEE-8103-2998236856BD-en-US-2/GUID-68C13886-A5D4-4274-9410-D1AE3D79CFF9.html).
