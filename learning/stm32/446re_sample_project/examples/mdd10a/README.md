# MDD10A 두 모터 20% 시험 — STM32F446RE HAL

이 폴더는 기존 `446re_sample_project` 안에 추가한 독립 예제입니다.
기존 `Core/Src/main.c`, `.ioc`, `.cproject`는 수정하지 않았습니다.
`src/mdd10a_test.c`와 `src/mdd10a_test.h`를 CubeMX 설정 후 명시적으로 통합해야 합니다.
보드 업로드나 실제 모터 시험은 수행하지 않았습니다.

## 배선

| MDD10A 입력 | STM32F446RE |
|---|---|
| GND | STM32 GND (공통 접지) |
| PWM2 | PA9 / TIM1_CH2 / AF1 |
| DIR2 | PC1 / GPIO output push-pull |
| PWM1 | PA8 / TIM1_CH1 / AF1 |
| DIR1 | PC0 / GPIO output push-pull |

입력 커넥터 순서 `GND / PWM2 / DIR2 / PWM1 / DIR1`은 사용자 제공 정보이며
실물 보드의 핀 표시와 대조해야 합니다. 모터1은 MDD10A 모터1 출력,
모터2는 모터2 출력에 연결합니다. STM32는 USB로, MDD10A 모터 전원은
배터리와 퓨즈로 공급합니다. 배터리 양극을 STM32 신호 핀에 연결하지 않습니다.
모터 전원은 MCU 초기화 전 PWM 신호가 LOW로 유지되는 것이 확인된 뒤 공급하세요.
리셋·부팅 중 MCU 핀은 high impedance일 수 있으므로 필요한 경우 PWM 입력에
외부 pull-down을 사용하세요. 전원 재인가나 MCU 리셋은 시험을 다시 실행합니다.

## 기존 프로젝트와 충돌 확인

`446re_sample_project`는 STM32F446RE를 사용합니다. 기존 `.ioc` 및 HAL 소스에서
PA8, PA9, PC0, PC1, TIM1은 사용되지 않습니다. 기존 PWM은 TIM3의 PA6, PA7,
PB0, PB1이며 I2C는 PB8/PB9, ADC는 PA0, UART는 PA2/PA3입니다.
번호가 같은 PB8/PB9와 PA8/PA9는 서로 다른 핀입니다.
`encorder_test`도 F446RE이지만 엔코더 실습 전용이며, `sample_mode`는 F103이라
이 예제의 대상이 아닙니다. 실제 외부 배선 충돌은 소스만으로 확인할 수 없습니다.

## CubeMX 설정

1. 기존 `.ioc`의 작업용 복사본 또는 별도 브랜치에서 설정합니다.
2. PA8 = TIM1_CH1, PA9 = TIM1_CH2, AF1, push-pull, no-pull.
3. PC0/PC1 = GPIO_Output, push-pull, no-pull, 초기 출력 LOW.
4. TIM1 Clock Source = Internal Clock, CH1/CH2 = PWM Generation CH1/CH2.
5. Counter Mode = Up, edge-aligned. PWM mode = PWM mode 1, polarity = High,
   Pulse = 0 (두 채널 모두), idle state = Reset, fast mode = Disable.
6. Prescaler/Period는 아래 계산에 맞추고 Repetition Counter = 0.
   ARR preload는 Enable, CCR preload는 HAL PWM 설정으로 활성화됩니다.
7. Complementary CH1N/CH2N, slave mode, one-pulse, break, automatic output,
   TIM1 interrupt 및 DMA는 모두 Disable. CH3/CH4는 사용하지 않습니다.
8. 생성된 `MX_TIM1_Init()`와 `HAL_TIM_MspPostInit()`가 TIM1 클럭 및 PA8/PA9
   AF1을 설정하는지 확인합니다. 별도로 PWM Start를 미리 호출하지 않습니다.
   HAL_Init/SysTick과 전역 인터럽트가 정상이어야 HAL_Delay가 진행됩니다.

권장 시작 주파수는 **20 kHz**입니다. 드라이버 실물 사양 및 모터 발열·소음에
맞게 확인하세요. TIM1은 APB2 타이머이며 edge-aligned upcounting에서:

```text
f_PWM = f_TIM1 / ((PSC + 1) × (ARR + 1))
ARR = f_TIM1 / ((PSC + 1) × f_PWM) - 1
```

STM32F446의 TIMPRE=0인 일반 설정은 APB2 prescaler가 1이면
`f_TIM1 = PCLK2`, 1보다 크면 `f_TIM1 = 2 × PCLK2`입니다.
TIMPRE=1이면 규칙이 달라지므로 Clock Configuration의 APB2 timer clock을
직접 확인하세요. SYSCLK나 PCLK2를 무조건 타이머 클럭으로 사용하지 마세요.

| 실제 TIM1 clock | PSC | ARR | PWM | 20% CCR |
|---|---:|---:|---:|---:|
| 84 MHz (현재 프로젝트: SYSCLK 84 MHz, APB2 /1, TIMPRE=0) | 0 | 4199 | 20 kHz | 840 |
| 180 MHz (예: APB2 /2, PCLK2 90 MHz, TIMPRE=0) | 0 | 8999 | 20 kHz | 1800 |

코드는 현재 ARR에서 `(ARR + 1) × 20 / 100`을 계산합니다. TIM1 16-bit 범위와
계산의 경계를 위해 ARR은 4~65534만 허용합니다. 계산은 정수 버림이므로
분주가 딱 맞지 않을 때 실제 duty에 작은 오차가 있습니다.

## main.c 통합 예시

CubeIDE에서 `src/mdd10a_test.c`를 `Core/Src`, 헤더를 `Core/Inc`에 복사하거나,
예제 `src`를 source location과 include path에 등록하세요. 한 방식만 선택해서
중복 컴파일하지 않습니다. 기존 프로젝트가 폴더를 자동 검색하는 경우에도
헤더 include path를 확인해야 합니다. 현재 예제는 자동 실행되도록 연결되어 있지 않습니다.

```c
/* USER CODE BEGIN Includes */
#include "mdd10a_test.h"
/* USER CODE END Includes */

/* main(), HAL_Init()/SystemClock_Config() 뒤 */
MX_GPIO_Init();
MX_TIM1_Init();  /* CubeMX가 생성한 초기화: CH1/CH2 pulse=0 */

/* USER CODE BEGIN 2: 초기화가 모두 끝난 후 딱 한 번 */
Mdd10aTestResult result = MDD10A_TestRunOnce(&htim1);
if (result != MDD10A_TEST_OK) {
    /* 함수가 PWM 핀을 LOW로 강제한 상태. 자동 재시도하지 않음. */
    Error_Handler();
}
/* USER CODE END 2 */

while (1) {
    /* 정상 완료 후 PWM은 둘 다 0%. 여기서 함수를 반복 호출하지 않음. */
}
```

기존 main에는 GPIO/UART/I2C 초기화가 중복되는 실습 코드가 있습니다.
이 호출은 마지막 주변장치 초기화 이후에 두고, 이후 GPIO/TIM1 재초기화 및
TIM1 레지스터를 변경하는 다른 코드가 실행되지 않게 하세요.
TIM1과 네 핀은 이 예제가 독점 사용하며 ISR 또는 다른 스레드에서 호출하지 않습니다.

## 실행 순서와 정지 동작

시작 전에 핀 모드/AF, timer mode/polarity, CCR=0, HAL 채널 상태,
timer 미시작, IRQ/DMA 미사용을 검사합니다. 오류나 한 채널의 HAL 시작 실패 시
TIM1 MOE/채널/카운터를 끄고 PA8/PA9를 GPIO LOW로 강제합니다.
실패 후 다시 실행하려면 설정을 고친 뒤 명시적으로 MCU를 리셋해야 합니다.

정상 순서: PWM 둘 다 0 → 1초 대기 → DIR LOW에서 모터1 20% 1초 →
둘 다 0으로 3초 → 모터2 20% 1초 → 0으로 3초 → 두 모터20% 1초 →
0으로 3초 → DIR HIGH로 반전 → 0 상태로 100ms → 같은 세 시험 및 정지 →
마지막 PWM 둘 다 0. 약 25.1초 후 반환하며 자동 반복하지 않습니다.
최종에는 LOW 신호를 유지하기 위해 두 PWM 채널을 0%로 계속 켜 둡니다.
방향 LOW/HIGH가 실제 전진/후진 중 무엇인지는 모터 배선에 따라 달라집니다.

3초 대기는 기계적인 완전 정지를 측정하지 않습니다. 관성/부하가 커서 정지가
늦으면 `STOP_TIME_MS`를 늘리세요. 엔코더나 전류 제한 피드백은 없는 시험입니다.
모터를 고정하고 바퀴를 띄운 상태에서 시작하며, 이상 시 배터리를 차단하세요.
20%에서 모터가 기동하지 않는 경우도 있습니다. 축이 막힌 상태로 두지 마세요.
PWM=0의 브레이크/코스트 동작과 전기적 정지 특성은 실제 MDD10A 사양에 따라
확인해야 하며 소프트웨어의 PWM0은 물리적인 전원 차단을 뜻하지 않습니다.

## 로컬 검증

CubeIDE bundled `arm-none-eabi-gcc`와 이 프로젝트의 실제 ST HAL/CMSIS 헤더를
사용해 Cortex-M4 대상으로 `-Wall -Wextra -Werror -fsyntax-only` 검증합니다.
`tests/verify_compile.ps1`을 이 예제 루트에서 실행하세요:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\verify_compile.ps1
```

이 검증은 보드 동작, 실제 PWM 주파수, 전원, 모터 정지 시간 및 main 통합을
검증하지 않습니다. 기존 프로젝트 전체를 빌드하거나 보드로 업로드하지 않습니다.
