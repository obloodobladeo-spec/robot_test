# 독립 CubeIDE 프로젝트 스냅샷

현재 테스트 프로젝트 전체의 소스·빌드 설정을 별도 경로로 보관했습니다. 기존 learning/stm32/encorder_test는 이전 스냅샷이며 현재 테스트를 재현하려면 이 폴더를 사용합니다.
CubeIDE에서 File → Import → General → Existing Projects into Workspace로 이 폴더를 선택하고, Build Project로 직접 빌드합니다. 프로젝트명이 같은 기존 프로젝트가 있다면 별도 워크스페이스에서 가져오세요.
.project, .cproject, .ioc, Core, Drivers, Startup 및 링커 스크립트를 포함합니다. Debug/Release 빌드 결과와 개인 IDE 설정은 포함하지 않습니다.
이 프로젝트는 UART 통신이나 통신 시간 초과 정지 기능이 없는 디버거 전용 시험입니다. 기본 프로젝트로 피드포워드를 옮기지는 않았습니다.
[모터 조정과 최종 5회 주행 결과](../../learning/MOTOR_PROGRESS_2026-11-21.md).

# UART 없는 모터·엔코더 디버깅

기본 학습 프로젝트는 저장소의 `learning/stm32/encoder_basic`입니다. 이 프로젝트 `encorder_test`는
그 제어 로직을 가져와 STM32CubeIDE 디버거만으로 시험하도록 구성했습니다.
프로젝트 이름의 기존 철자 `encorder_test`는 그대로 유지합니다.

## 구성과 차이

- 모터 구조체, PI, anti-windup, 가감속, 방향 전환, 로봇 속도 변환 및 오도메트리는
  모두 `Core/Src/main.c`에 있습니다. 별도 `motor_control.h`나 `robot_control.h`는 필요 없습니다.
- HAL 기본 헤더인 기존 `main.h`, 타이머 MSP 및 인터럽트 지원 파일은 사용합니다.
- 프로젝트에 원래 있던 미완성 `motor_control.c/.h`는 파일을 보존하되,
  해당 `.c`를 Debug·Release 빌드에서 제외했습니다. 현재 제어 구현은 `main.c`만 사용합니다.
- UART 초기화·송수신·콜백·파서·통신 시간 초과 정지를 제거했습니다.
- 초기 명령은 0입니다. 디버거에서 지정한 명령은 다시 변경하거나 리셋할 때까지 유지됩니다.
- 디버거에서 외부 변경하는 두 명령 변수에만 `volatile`을 명시했습니다.
- 타이머 설정, 현재 소스의 PI 초기값 Kp=0.1/Ki=0.1, 100ms 제어 주기,
  듀티 상한 50%, 바퀴 속도 상한 ±200mm/s, 가속도 제한 100mm/s²는 원본 기준입니다.
- 모터 1=왼쪽, 모터 2=오른쪽이라는 원본 코드 매핑을 유지했습니다.
  실제 장착·배선이 바뀌었다면 손으로 바퀴를 돌려 대응을 확인합니다.

## HSE 비교 설정

`main.c`의 `CLOCK_USE_HSE=1`은 ST-LINK MCO 8MHz를 HSE Bypass 입력으로 시도합니다.
HSE가 준비되면 PLLM=8, HSI 복귀 경로에서는 PLLM=16이며 두 경로 모두 SYSCLK=84MHz입니다.
TIM1은 PSC=0/ARR=4199를 유지하므로 PWM 설정 20kHz, 제어 주기 100ms도 유지합니다.
실제 보드 버전과 ST-LINK 전원·MCO 연결에 따라 HSE 공급 여부가 달라질 수 있습니다.

- `clock_hse_active=1`, `clock_hse_failed=0`: HSE 준비 확인 후 PLL 소스로 선택.
- `clock_hse_active=0`, `clock_hse_failed=1`: HSE 시작 실패, HSI로 복귀.
- `CLOCK_USE_HSE=0`으로 빌드: HSI 사용, 두 상태값 모두 0.

HSE 선택과 누락 시 복귀·HSI 강제 선택 경로는 모의 검사했고 ARM 빌드도 통과했습니다.
11월16일 보드에서 HSE 활성 상태를 확인했습니다. 클럭 변경만의 흔들림 개선 효과는 확정하지 않았습니다.
클럭 설정은 main.c의 사용자 수정입니다. `.ioc`의 기존 클럭 계획은 유지했으며 CubeMX에서 다시 생성하면
SystemClock_Config가 덮어써질 수 있으므로 이 변경을 재적용해야 합니다.

이전 출발 보상은 한 차례 제거했으며, 현재는 아래 전진 피드포워드 비교 기능을 적용한 상태입니다. Kp/Ki는 0.1/0.1입니다.

## CubeIDE 실행 순서

1. 모터 배터리를 분리하고 ST-LINK USB를 Windows에 연결합니다.
   WSL에 넘겼다면 PowerShell의 `usbipd list`로 BUSID를 확인하고
   `usbipd detach --busid 실제번호`로 해제합니다.
2. CubeIDE에서 `encorder_test`를 선택하고 F5로 새로고침합니다.
   프로젝트가 없다면 `File → Import → General → Existing Projects into Workspace`로 가져옵니다.
3. `Build Project`를 실행합니다. 결과 파일은 `Debug/encorder_test.elf`입니다.
4. 이 프로젝트의 `encorder_test Debug` 구성을 선택해 보드에 다운로드하고 디버깅합니다.
   `encoder_basic` 디버그 구성을 선택하지 않도록 프로젝트명과 ELF 경로를 확인합니다.
5. Live Expressions에 아래 항목을 추가하고 F8로 실행합니다.
6. 명령 0을 확인한 뒤 바퀴를 띄우고 모터 배터리를 연결합니다.
7. 우선 `angular_speed_rad_s=0`, `linear_speed_mm_s=50`으로 시험합니다.
8. 정지할 때 두 명령을 모두 0으로 바꾸고 실제 속도와 CCR이 0인지 확인합니다.
   디버깅 종료 전 모터 배터리를 분리합니다.

디버거를 중단한다고 PWM 출력까지 자동으로 0이 되지는 않습니다.
이 프로젝트에는 UART 시간 초과 정지가 없으므로 명령 0으로 정지하는 순서를 지킵니다.

## 입력 명령

| 변수 | 단위 | 예 |
|---|---|---|
| `linear_speed_mm_s` | mm/s | `50` 전진, `-50` 후진, `0` 정지 |
| `angular_speed_rad_s` | rad/s | `0` 직진, `0.3` 양의 회전, `-0.3` 반대 회전 |

별도 `manual_debug` 스위치나 ROS/motor_bridge 실행은 필요 없습니다.
`motor1.state.target_speed_mm_s`는 로봇 명령으로 매번 갱신하므로,
목표 변경은 위 두 입력 변수로 합니다.

## Live Expressions

다음을 한 항목씩 추가합니다. 모터 2는 `motor1`을 `motor2`로 바꿉니다.

```c
linear_speed_mm_s
angular_speed_rad_s

motor1.state.target_speed_mm_s
motor1.state.control_target_speed_mm_s
motor1.state.speed_mm_s
motor1.state.rpm
motor1.state.previous_count
motor1.state.delta_count
motor1.state.delta_distance_mm
motor1.state.duty_percent
motor1.state.speed_error
motor1.state.i_output
motor1.state.kp
motor1.state.ki

htim2.Instance->CNT
htim3.Instance->CNT
htim1.Instance->CCR1
htim1.Instance->CCR2

robot.state.x_mm
robot.state.y_mm
robot.state.yaw_rad
```

`previous_count`는 가장 최근 측정 구간에서 저장한 카운트입니다.
`CNT`는 타이머 하드웨어의 현재 원시 카운트이고, `delta_count`는 부호가 있는 구간 변화량입니다.
TIM2는 32비트, TIM3는 16비트이므로 0에서 역방향으로 돌리면 원시 CNT가 큰 값으로 순환할 수 있습니다.

## 핀

| 기능 | 모터 1 | 모터 2 |
|---|---|---|
| PWM → MDD10A | PA8 / TIM1_CH1 → PWM1 | PA9 / TIM1_CH2 → PWM2 |
| DIR → MDD10A | PC0 → DIR1 | PC1 → DIR2 |
| 엔코더 A/B | PA0·PA1 / TIM2 | PA6·PA7 / TIM3 |

STM32·드라이버·엔코더 GND는 공통 연결합니다. PA2·PA3의 UART 기능은 사용하지 않습니다.
전진 방향으로 손으로 돌렸을 때 해당 모터 속도가 양수인지 먼저 확인합니다.

## 바퀴 교체 시

`main.c`의 각 `motor.config`에서 `wheel_circumference_mm`을 실측 구름 둘레로 바꿉니다.
현재 값은 기존 지름 110mm 바퀴 기준으로 두 바퀴 둘레 모두 345.6mm,
CPR은 10회전 실측 평균인 모터1 2473.6 / 모터2 2464.5를 사용하며,
윤거는 `WHEEL_TRACK_MM=234.0f`입니다.
2026-11-09 사용자가 기존 바퀴로 복귀하고 구동 바퀴 중심 간격을 정확히 234mm로 확인했습니다.
둘레는 지름에 원주율을 곱한 기하학적 값이며, 최종 하중의 실제 구름 둘레로 보정한 값은 아닙니다.
2026-11-09 재측정: 모터1은 0→24736, 모터2는 21→24666이며 각각 10회전했습니다.
모터2는 시작 카운트 21을 빼서 (24666−21)/10=2464.5로 계산했습니다.
바퀴 크기와 윤거는 소프트웨어 환산값이므로 이 값만 변경할 때 CubeMX 핀·타이머 설정은 바꾸지 않습니다.

## CubeMX 및 원본 보존

`encorder_test.ioc`도 같은 타이머·핀 및 UART 비활성 상태로 맞췄습니다.
재생성할 때 사용자 코드 보존을 켜고, `USER CODE` 영역의 단일 파일 제어 코드가 유지되는지 확인합니다.

변경 전 `encorder_test` 전체는 다음 파일로 백업했습니다.

`C:\Users\heosi\stm32_backups\encorder_test_before_motor_debug_20261109_154933.zip`

테스트 프로젝트를 처음 만들 때는 `encoder_basic`을 수정하지 않았습니다.
이후 요청에 따라 두 프로젝트의 윤거·바퀴 둘레·CPR을 동일하게 맞췄습니다.
이번 설정 반영에서는 보드 다운로드나 모터 구동을 하지 않았습니다.

## 생성 시 검증

- CubeIDE 2.2.0의 ARM GCC로 Debug 컴파일·링크 완료: 오류 0, 경고 0.
- 실제 제어 코드와 메인 반복문을 HAL 모의 환경에서 실행해 다음 세 가지를 확인했습니다.
  UART 없이 50mm/s 명령이 2초간 유지됨, 0 명령으로 적분·양쪽 CCR이 0이 됨,
  16비트 및 32비트 엔코더 카운터 경계 처리.
- 최초 테스트 생성 시 원본 `encoder_basic`의 파일 해시를 대조해 변경이 없음을 확인했습니다.
- 위 항목은 최초 생성 시 검증입니다. 이후 현재 전진 보상 버전은 보드에 적용해 전진 시험을 수행했으며, 회전·후진 보상은 검증하지 않았습니다.

## 전진 피드포워드 비교 시험 (2026-11-16)
전진 50mm/s, 각속도 0에서만 기본 듀티 12% 후보를 적용합니다. FEEDFORWARD_TEST_ENABLE=0이면 기존 PI로 비교합니다. 보상은 6%p/s로 상승하여 약 2초에 12%가 되며 정지·방향 전환에서는 초기화합니다. Kp/Ki는 0.1을 유지합니다. 다른 속도와 회전·후진에는 보상하지 않습니다. 이는 실험 설정이며 기본 프로젝트에 반영하지 않았습니다. 새 보상 상태 변수로 RAM 주소가 달라질 수 있으므로 기존 주소 고정 기록 스크립트를 재사용하기 전에 ELF 심볼을 확인해야 합니다. 기본 듀티 상승 중에는 양의 적분 추가 누적을 제한하고 과속을 줄이는 보정은 허용합니다. 현재 버전은 빌드·링크, 보드 다운로드 검증 및 11월16~17일 전진 반복 시험을 완료했습니다.
