# 개별 모터 시험, 3초 제자리 회전 시험과 엔코더 확인

이 프로젝트는 CubeIDE 구조(`Core/Src`, `Core/Inc`, `Drivers`)를 유지합니다.
`main.c`는 주변장치/엔코더 초기화와 루프를 연결하고, `motor_sequence.c/.h`는
비차단 모터 시험 단계와 ARR 기반 duty 계산을 담당합니다.
기존 blocking `mdd10a_test.c/.h`는 수정하지 않았으며 현재 main에서 호출하지 않습니다.

| 기능 | STM32 핀/타이머 |
|---|---|
| 모터1 PWM / DIR | PA8 TIM1_CH1 / PC0 |
| 모터2 PWM / DIR | PA9 TIM1_CH2 / PC1 |
| 모터1 엔코더 A/B | PA0/PA1 TIM2 |
| 모터2 엔코더 A/B | PA6/PA7 TIM3 |

TIM1은 84MHz, PSC=0, ARR=4199, PWM1 active high로 20kHz입니다.
20% compare는 `(ARR+1)*20/100=840`입니다. 개별 시험 뒤 회전 단계에서는 두 모터를 동시에 구동합니다.
MDD10A와 STM32는 GND를 공통으로 연결합니다. MCU 핀 이름과 보드 커넥터 위치는
구별하고 실제 보드의 PC0/PC1 연결 및 솔더 브리지 설정을 확인하세요.

펌웨어를 실행/리셋하면 **자동으로** 다음 순서를 한 번 수행합니다.

1. 두 PWM을 0으로 확정하고 HAL 채널 시작 후 3초 대기.
2. 모터1 DIR LOW → 1ms 대기 → 20% 1초 → 두 PWM0으로 3초 정지.
3. 모터1 DIR HIGH → 1ms 대기 → 20% 1초 → 두 PWM0으로 3초 정지.
4. 모터2 DIR LOW → 1ms 대기 → 20% 1초 → 두 PWM0으로 3초 정지.
5. 모터2 DIR HIGH → 1ms 대기 → 20% 1초 → 두 PWM0으로 3초 정지.
6. 회전용 DIR 설정 → 1ms 대기 → 양쪽 20%로 3초 구동 → 두 PWM0으로 영구 정지.

총 약22초이며 루프 지연만큼 길어질 수 있습니다. DIR 변경 전에 3초간 0%를
유지하며 CCR preload를 update event로 반영합니다. LOW/HIGH가 실제 전진/후진 중
무엇인지는 배선에 따라 달라집니다. 3초는 실제 기계 정지 감지가 아닙니다.
회전 방향은 `Core/Src/motor_sequence.c`의 `SPIN_DIR1_STATE`, `SPIN_DIR2_STATE`로
설정하며 기본값은 둘 다 `GPIO_PIN_RESET`입니다. 좌우 대칭 장착과 동일 배선 극성을
전제로 하므로 바퀴를 띄워 차체 기준 한쪽 전진/다른 쪽 후진인지 먼저 확인하세요.
양쪽이 같은 전진 방향이면 `SPIN_DIR2_STATE`를 `GPIO_PIN_SET`으로 바꿉니다.
현재 동작은 각도 제어가 아닌 3초간 회전 출력 시험입니다. 실제 회전 여부와 각도는
바닥 마찰과 배선에 따라 달라지며 엔코더 피드백으로 보정하지 않습니다.
모터를 고정하고 바퀴를 띄워 확인하세요. 디버거로 정지하면 마지막 PWM이 유지될 수
있으므로 구동 중 breakpoint 사용에 주의하세요. 소프트웨어 PWM0은 전원 차단이 아닙니다.

`HAL_GetTick()`의 unsigned 차분으로 시간이 경과했는지 확인하므로 `HAL_Delay`로
루프를 막지 않습니다. 엔코더 값은 10ms마다 계속 갱신됩니다.
CubeIDE Expressions/Live Expressions에서 다음 값을 관찰하세요.

- `motor_test_phase`: `MOTOR_TEST_STARTUP_WAIT` → 모터1 LOW/HIGH 시험과 정지 →
  모터2 LOW/HIGH 시험과 정지 → `MOTOR_TEST_SPIN_SETTLE` → `MOTOR_TEST_SPIN_RUN`
  → `MOTOR_TEST_DONE`. `FAILED`는 시작 검사/오류 상태입니다.
- `motor1_count`: 기존 `int32_t` TIM2 카운터 표시.
- `motor2_count`: 기존 `int64_t` 변수에 `(int16_t)TIM3_CNT`를 저장합니다.
  16-bit 카운터의 signed wrap 표시를 유지하며 64-bit 누적 이동량은 아닙니다.

오류 시 기존 `Error_Handler`가 TIM1을 끄고 PA8/PA9 및 DIR을 GPIO LOW로 강제합니다.
완료 후 카운터 관찰은 계속되며 자동 반복하지 않습니다. MCU 리셋하면 시험을 재시작합니다.

빌드: CubeIDE에서 `446re_sample_project` 선택 → Project → Build Project(Debug).
결과는 `Debug/446re_sample_project.elf`입니다. `motor_sequence.c`는 기존 `Core`
source entry로 함께 빌드됩니다. 사용자 코드 연결은 main의 USER CODE 영역에 있습니다.
`.ioc`, MSP/IRQ와 엔코더 초기화는 이번 변경에서 유지합니다.
보드 업로드/실제 모터 구동은 수행하지 않았으며 빌드는 물리적 동작 검증을 대신하지 않습니다.
