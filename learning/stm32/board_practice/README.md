# board_practice — 모터·엔코더 및 BNO085 IMU 학습 기록

## 2026-11-22 변경: 볼 캐스터 1개 주행 조정

- 구성: 앞 볼 캐스터 1개(최종 하중 탑재). 50 cm 왕복(전진·후진), SWD로 RAM 값을 바꿔 가며 비교.

| 단계 | 설정 | 전진 IMU 최종 | 후진 IMU 최종 | 엔코더-IMU (전/후) | yaw 진폭 |
|---|---|---|---|---|---|
| 기준 | R 둘레 345.6, Kp 0.1 / Ki 0.2 | +0.07°±0.20 | +0.89°±0.29 | +5.4° / -4.6° | 1.9° |
| 1 | **R 둘레 330.7** | +0.05°±0.40 | -0.15°±0.47 | +0.5° / 0.0° | 1.8~2.1° |
| 2 | heading hold 끔(비교용) | +0.28°±1.19 | +4.84°±0.81 | | 1.9~3.6° |
| 3 | Kp 0.2 | +0.17°±0.25 | +0.08°±0.14 | | 1.4~1.5° |
| **4 (채택)** | **Kp 0.15 / Ki 0.25** | **-0.14°±0.16** | **-0.19°±0.15** | -0.2° / +0.6° | 1.4~1.8° |

- 바퀴 속도 흔들림 σ 5~6 mm/s는 설정과 무관(heading hold를 꺼도 같음) -> 기계 요인으로 추정, 제어로는 줄지 않음.
- 실효 윤거 245.5 mm 유지(볼 캐스터에서 잰 250 mm는 USB 선 꼬임 영향으로 신뢰 낮음, 재측정 필요).

## 2026-11-21 변경: 방향 전환 대기 수정과 직진 게인 조정

- 방향 전환 대기: 정지 중에도 `stop_samples`를 세어, 이미 0.3 s 이상 멈춘 바퀴는 바로 DIR을 바꾸고 같은 주기에 출발.
  (이전: 직전에 반대로 돌던 바퀴만 0.3 s 늦게 출발 -> 직전 동작에 따라 휘는 방향이 바뀜)
- 캐스터 구리스 후, 30/50/80 mm/s x 3회씩 30 cm 직진 (IMU 최종 방향 오차 |평균|, 최대 편차):

| 단계 | 설정 | |최종| | 최대 편차 |
|---|---|---|---|
| 기준 | 바퀴 Ki 0.1, KP 3(엔코더), KI 0.5 | 1.93° | 2.62° |
| 1 | 바퀴 Ki 0.2 | 1.40° | 2.58° |
| **2 (채택)** | + `HEADING_KP_IMU` 2.0, `HEADING_KI` 1.0 | **0.99°** | **1.69°** |
| 3 | `HEADING_KP_IMU` 3.0, `HEADING_KI` 1.5 | 1.04° | 1.66° (개선 없음) |

- 최종: 바퀴 Kp 0.1 / **Ki 0.2**, heading hold KP 3.0(엔코더) + **KP_IMU 2.0** + **KI 1.0**(IMU).

## 2026-11-21 변경: 출발 꿈틀거림·방향 틀어짐 대책

수정 전 백업: `/home/kim/stm32_backups/board_practice_20261121_212026.tar.gz`

- 측정(바닥, SWD로 kp=ki=0·i_output 고정): 출발 듀티 약 9.3%(좌우·전후진 8.8~10%).
  고정 듀티 9~14% 전진 정상 속도로 `duty ≈ 7.5 + 0.097 v` (L 7.54+0.094v, R 7.48+0.100v).
  같은 듀티에서 R이 약 5% 느렸지만 GitHub 기록상 매번 같은 쪽이 빠르진 않으므로 좌우 고정 보정은 하지 않음.
  후진은 방향 전환 직후 캐스터 회전 저항으로 데이터가 불안정(첫 블록 정지).
- 피드포워드 교체: 50 mm/s 직진 전용·6%p/s 상승 → **모든 명령에서 `offset + slope × |제어 목표 속도|`**
  (`MotorConfig.ff_offset_percent` 7.5, `ff_slope_percent_per_mm_s` 0.097, 좌우 같음).
  가속 중(제어 목표 < 목표)에는 양의 적분 누적 억제.
- heading hold: P는 **엔코더 yaw 오차**(KP 3.0, 지연 없는 좌우 동기화), I는 **IMU yaw 오차**(KI 0.5, 실제 방향).
  IMU가 없으면 I도 엔코더. 최대 보정 ±0.15 rad/s 유지.
- 검증: CubeIDE Debug 빌드 경고 0. 보드 업로드·주행 미확인.

## 2026-11-21 변경: encorder_test의 피드포워드·HSE 클럭 반영

수정 전 백업: `/home/kim/stm32_backups/board_practice_20261121_184920.tar.gz`
기준: GitHub `stm32_projects/encorder_test` (커밋 993d151, ef295a5). 코드를 통째로 옮기지 않고
기존 `motor_control.c/.h`, `robot_control.c/.h` 분리 구조에 기능만 반영했다. 디버거용 `volatile` 명령 변수,
`clock_hse_active/failed` 상태 변수는 옮기지 않았다.

- 기본 듀티 피드포워드 (`motor_control.h` `FEEDFORWARD_*`): 전진 50 mm/s·각속도 0 명령에서만
  `12% × 제어목표속도/50`을 6%p/s로 올려 PI 출력에 더한다(약 2초에 12%). 상태는 `MotorState.feedforward_percent`.
- 상승 중 양의 적분 누적 억제, 음의 보정 허용. 출력 포화 판단은 기본 듀티+P+I 기준.
- 정지(`Motor_Stop`)·방향 전환 시 기본 듀티를 0으로 초기화.
- 적용 여부는 `Robot_SetFeedforward()`가 heading hold 보정 **전** 원래 명령으로 판단한다.
- `SystemClock_Config`: HSE Bypass(ST-LINK MCO 8 MHz) 시도, 실패 시 HSI. PLLM 8/16, SYSCLK 84 MHz 동일.
  CubeMX 생성 영역이므로 재생성 시 다시 적용해야 한다(`.ioc`는 HSI 그대로).
- 검증: CubeIDE Debug 빌드 경고 0. 보드 업로드·주행 미확인.

## 2026-11-21 변경: 모터 PI 게인을 encorder_test 설정으로

수정 전 백업: `/home/kim/stm32_backups/board_practice_20261121_183406.tar.gz`

- 사용자 요청: 코드 전체가 아니라 test 프로젝트의 **구성(설정값)만** 적용. test 코드는 디버깅용이다.
- `motor_control.c` `Motor_Init`: Kp 0.15 → **0.1**, Ki 0.15 → **0.1** (GitHub `encorder_test/Core/Src/main.c`, `MOTOR_PROGRESS_2026-11-21.md`의 시험 설정).
- 그 외 비교 결과 동일: 클록(HSE 8 MHz, SYSCLK 84 MHz), TIM1 PWM Period 4199, TIM2/TIM3 엔코더 Period·필터,
  듀티 50%, 속도 200 mm/s, 가속 100 mm/s², 윤거 234 mm, 바퀴 둘레 345.6 mm.
- CPR은 basic 값(2473.6 / 2464.5)을 유지. GitHub의 encorder_test 사본(11/12)은 2475.6 / 2465.2로 옛 값이고,
  11/21 문서는 시험 시 CPR이 basic과 같았다고 기록한다.
- 출발 피드포워드(12% 듀티 상승)는 test 전용 코드이며 GitHub에도 없어서 옮기지 않았다.

## 2026-11-21 변경: 직진 방향 유지(heading hold)

수정 전 백업: `/home/kim/stm32_backups/board_practice_20261121_182641.tar.gz`

- 배경: GitHub `learning/MOTOR_PROGRESS_2026-11-21.md` — 주로 오른쪽으로 휨, 먼저 출발하는 바퀴가 매번 달라 고정 보상은 부적합.
  바퀴 띄움 시험(2026-11-21)에서도 출발 시 R이 약 0.2 s 늦게 돌아 엔코더 yaw가 -1.6° 틀어짐.
- `Robot_HeadingHold()` (`robot_control.c`): `W=0`, `V!=0`일 때만 동작. 직진 시작 시 yaw를 기준으로 잡고
  PI(`HEADING_KP 1.5`, `HEADING_KI 0.3`)로 각속도 보정, 최대 ±0.15 rad/s. 회전 명령·정지·방향 전환 시 초기화.
- 기준: IMU 값이 최신이면 IMU yaw(실제 차체 방향, 바퀴 지름 차·미끄러짐 포함 보정), 아니면 엔코더 yaw.
  IMU 장착이 뒤집혀 yaw 부호가 반대면 `HEADING_IMU_SIGN`을 -1로 바꾼다.
- telemetry에 `H=<보정 mrad/s>`를 `N=` 뒤에 추가(ROS 파서는 모르는 key를 무시). `uart_message` 192→224.
- 바퀴를 띄우면 IMU yaw가 변하지 않으므로 보정이 0으로 유지된다. 효과는 바닥 주행에서만 확인 가능.
- 검증: 스크래치·CubeIDE Debug 빌드 경고 0. 보드 업로드·바닥 주행 미확인.

## 2026-11-21 변경: IMU 통합과 10 Hz telemetry

수정 전 백업: `/home/kim/stm32_backups/board_practice_20261121_031932.tar.gz`

- **telemetry 주기 1000 ms → 100 ms** (`TELEMETRY_PERIOD_MS`). ROS2 odom/TF가 10 Hz가 된다.
- **IMU 통합**: `uart_project`에서 실물 수신이 확인된 CEVA SH-2 방식을 옮겼다.
  - 추가: `Core/Src/imu_app.c`, `Core/Inc/imu_app.h`, SH-2 라이브러리(`sh2*.c/h`, `shtp.c/h`, `euler.h`), `ThirdParty/CEVA_SH2/`(라이선스).
  - 보고서: Game Rotation Vector(자기계 미사용) + 보정 자이로, 각 50 Hz. I²C는 100 kHz 유지.
  - USART2는 ROS2 전용이므로 IMU 콘솔 출력은 없다. 상태는 Live Expressions의 `g_imu`로 본다.
  - 센서가 없어도 모터 제어는 계속 동작한다. 재연결(수백 ms 대기)은 정지 명령 상태에서만 시도한다.
  - 기존 학습 코드 `imu_control.c/h`는 파일만 남아 있고 `main.c`에서 호출하지 않는다.
- **telemetry 형식** (공백 1칸으로 정리, 기존 `R=..  X=`의 2칸 제거):

```text
L=<mm/s> R=<mm/s> X=<mm> Y=<mm> A=<mrad> T=<MCU ms> N=<seq> IS=1 QX= QY= QZ= QW=(x10000) GX= GY= GZ=(mrad/s) QA= GA=(0~3)
L=... A=... T=... N=... IS=0      ← IMU 값이 없거나 100 ms보다 오래됨
```

검증 범위: CubeIDE 2.2.0 동봉 arm-none-eabi-gcc로 스크래치 사본을 빌드(Core 경고 0, text 46 KB)했다.
CubeIDE 프로젝트 빌드, 보드 업로드, 실물 센서·모터 동작은 확인하지 않았다.
CubeIDE에서 새 파일이 보이지 않으면 프로젝트를 F5로 새로고침한 뒤 전체 빌드한다.

## 이어서 작업할 위치 (2026-11-19 정리)

NUCLEO-F446RE와 GY-BNO085 모듈을 사용한다. 대화에서 `encoder_basic`에
IMU를 통합하는 방향으로 학습했으며, 현재 직접 작성하던 IMU 코드가 있는
작업 폴더는 **`/home/kim/stm32_project/board_practice`**다.
`/home/kim/github_review/robot_test/learning/stm32/encoder_basic`은 별도 참고본이다.
두 폴더를 같은 작업본으로 혼동하지 않는다.

사용자는 코드를 직접 입력하면서 배우는 방식을 요청했다. 다음 작업도 먼저
문법과 동작을 설명하고, 사용자가 작성한 코드를 확인하는 순서로 진행한다.
이번 정리는 문서만 작성했으며 펌웨어 수정·빌드·플래시·실물 시험은 하지 않았다.

## 프로젝트 구분

| 프로젝트 | 역할 | 확인된 진행 상태 |
|---|---|---|
| `board_practice` (이 폴더) | 기존 모터·엔코더 코드에 IMU를 단계적으로 추가하는 학습본 | 리셋, ACK, INT, 4바이트 헤더 읽기까지 작성 |
| `../uart_project` | CEVA SH-2 라이브러리를 사용하는 IMU 단독 동작 참고본 | 이전 대화에서 실제 가속도·각속도·쿼터니언·yaw 시리얼 출력을 확인 |

단독 참고본의 센서 출력 성공이 이 학습본의 전체 IMU 통합 완료를 뜻하지 않는다.
참고본의 자세한 구조와 출력 단위는 [uart_project README](../uart_project/README.md)를 본다.

## CubeMX 설정과 현재 배선 기준

현재 `board_practice.ioc`, `Core/Inc/main.h`, 생성된 초기화 코드 기준이다.

| BNO085 핀 | STM32 연결 / 상태 | 설정 |
|---|---|---|
| VCC | 3V3 | 전원 |
| GND | GND | 공통 접지 |
| SCL | PB8 | I2C1_SCL |
| SDA | PB9 | I2C1_SDA |
| RST | PC5 | Push-pull 출력, 초기 High |
| INT | **PC6** | Pull-up 입력, Low일 때 데이터 준비 |
| PS0 / PS1 | Low | I²C 모드 선택 |
| AD0 | Low | 7비트 주소 0x4A 선택 |
| CS | High | I²C 사용 시 유지할 상태 |

I2C1은 Standard Mode, **100 kHz**, 7비트 주소를 사용한다.
HAL 함수에 주소를 넘길 때는 `0x4A << 1`을 사용한다.
이전 대화의 초기 배선표에는 INT=PC4가 있었지만, 이후 설정과 현재 코드는
**PC6**다. 실물 INT 연결도 PC6인지 확인한다.

PS0/PS1/AD0/CS는 통신 모드·주소 선택용이며 측정값을 읽는 핀이 아니다.
외부 점퍼를 생략할 수 있는지는 해당 모듈의 풀업·풀다운 실장을 확인해야 한다.
SCL/SDA에는 3.3V 기준 풀업이 필요하며 GPIO 내부 풀업만으로 대체하지 않는다.

이전 멀티미터 측정에서 사용자가 기록한 값은 순서대로 4.8 kΩ, 4.8 kΩ,
0.492 MΩ, 6.46 MΩ였다. 당시 측정 지점의 번호 대응이 현재 파일에 남아 있지
않으므로 이 값만으로 특정 모드 선택 핀의 점퍼 제거를 확정하지 않는다.

## 직접 작성한 코드와 현재 호출 흐름

- `Core/Inc/imu_control.h`: HAL 헤더 포함 및 공개 함수 선언.
- `Core/Src/imu_control.c`: IMU GPIO·I²C 접근 함수 구현.
- `Core/Src/main.c`: 초기화와 반복 호출. 기존 모터·UART 처리도 유지되어 있다.
- `Core/Src/motor_control.c`, `robot_control.c`: 모터 제어 및 엔코더 기반 이동량 계산.

| 함수 | 현재 동작 |
|---|---|
| `IMU_RESET()` | RST Low → 10 ms 대기 → High → 150 ms 대기 |
| `IMU_CheckConnection(&hi2c1)` | 0x4A ACK 검사, 2회 시도·timeout 20 ms |
| `IMU_DataReady()` | INT가 Low이면 1, 아니면 0 반환 |
| `IMU_ReadHeader(&hi2c1, imu_header)` | 준비되지 않았으면 `HAL_BUSY`, 준비됐으면 4바이트 I²C 수신 |

현재 `main()`은 주변장치 초기화 후 아래 순서로 실행한다.

```text
IMU_RESET()
  → IMU_CheckConnection()
  → HAL_OK이면 IMU_ReadHeader() 한 번 호출
  → while(1)에서 IMU_DataReady()를 반복 확인
  → 기존 UART 명령 및 주기적인 모터·엔코더 처리
```

디버거에서 볼 변수:

- `imu_status`: ACK 검사 결과.
- `imu_ready`: 현재 INT 준비 상태.
- `imu_header[4]`: 수신한 헤더 4바이트.
- `imu_header_status`: 헤더 읽기 결과. 준비 전이면 `HAL_BUSY`일 수 있다.

현재 반복문에는 지속적인 전체 패킷 수신·보고서 해석이 없다.
`IMU_ReadHeader()`의 성공은 자세 데이터 수신 성공과 다르다.
현재 구현은 SHTP 헤더를 읽는 학습 단계이며 완성된 센서 드라이버가 아니다.

## 이전 실물 동작 확인 기록

IMU 단독 참고본을 실행했을 때 사용자가 다음 형식의 출력을 제공했다.

```text
A_mms2=-726,-652,10031 G_mrads=5,19,-50
Q_1e4=-296,-236,8415,5388 YAW_mdeg=114685
STATUS=2,0,0 AGE_ms=11,0,15 N=22391,17728,17727
```

사용자가 움직였을 때 값이 변하고, 수신 개수 `N`도 증가하는 것을 확인했다.
이는 당시 센서 통신과 보고서 수신이 동작했다는 기록이다. 모든 보고서의 보정
완료, 로봇 장착 축 검증, 모터 동작 중 자기장 영향 또는 주기 보장은 별도다.

## 다음 학습 순서

1. 현재 네 함수의 선언·정의·인자와 CubeMX 핀 설정을 다시 확인한다.
2. INT 준비 시 헤더를 읽고 길이·채널·시퀀스를 해석하는 원리를 배운다.
   길이 필드와 continuation 비트, 버퍼 크기, 짧은 패킷·통신 오류를 다룬다.
3. 헤더 뒤 payload까지 수신하고 전체 SHTP 패킷을 처리하는 순서를 배운다.
   I²C 읽기 시 헤더 반복 등은 단독 참고본의 SH-2 HAL 구현을 참고한다.
4. 보고서 활성화와 수신 해석을 배우고, 우선 yaw·Z축 각속도 등 필요한 값을
   정한다. 기울기 확인이 필요하면 roll/pitch도 유지한다.
5. 기능이 준비되면 별도 앱 계층의 초기화/반복 처리로 정리한다.
   현재 학습본에는 IMU 앱 계층이 아직 추가되지 않았다.
6. 모터 제어 주기를 우선 유지하면서 IMU 수신을 통합한다. 리셋용 `HAL_Delay()`는
   초기화에 한정하고, 반복 처리에서는 긴 대기·과도한 UART 출력을 피한다.
7. 장착 축·단위·yaw 부호·±180° 경계와 수신 지연을 확인한 뒤 엔코더와 결합한다.
   직진 보정 및 기울기 판정은 그 다음 단계다.

지상 주행에서는 자기장 영향도 검토한다. 현재 단독 참고본의 Rotation Vector는
자기계를 사용한다. 자기계를 쓰지 않는 자세 보고서를 선택할 경우 yaw 드리프트와
엔코더/외부 위치 정보와의 결합을 함께 검토하며, 변경 전후를 실제로 비교한다.

## 재개 시 검증

CubeIDE에서 이 폴더의 `board_practice.ioc`와 프로젝트를 연다.
전체 프로젝트를 빌드하며 `main.c`만 따로 업로드하지 않는다.
문법/빌드 확인과 실물 확인을 구분한다. 본 README 작성 시 빌드는 실행하지 않았다.
센서 단독 수신을 먼저 확인하고, 모터 통합 시에는 제어 주기가 밀리지 않는지도 측정한다.
