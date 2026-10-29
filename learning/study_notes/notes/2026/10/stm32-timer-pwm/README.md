# STM32 타이머·인터럽트·PWM 학습 기록

이번 학습 범위는 NUCLEO-F446RE와 STM32CubeIDE/CubeMX의 `timer_test`, `PWM_test` 프로젝트입니다. 여러 날에 걸쳐 익힌 개념과 디버깅 결과를 함께 정리했습니다. GPIO는 기존 LED BSP 함수 사용과 PWM 핀 충돌 확인 범위에서 다뤘습니다.

[전체 학습 목록](../../../../README.md)

## 확인 수준과 현재 소스

대화에서 사용자가 확인한 보드 동작, 저장된 소스를 읽어 확인한 상태, 앞으로 적용할 예제를 구분합니다. 이 문서를 작성하면서 빌드·다운로드·실행·시간 측정은 하지 않았습니다.

| 대상 | 확인 내용 |
| --- | --- |
| 사용자 확인 | PWM Pulse가 0일 때 LED가 켜지지 않았고, 200으로 설정한 뒤 정상 동작함 |
| 사용자 확인 | GDB Expressions에서 함수 내부 static 변수의 실제 심볼 이름을 사용해 값을 조회함 |
| 현재 `timer_test/Core/Src/main.c` | `SystemClock_Config()` 호출, PSC 8399·ARR 999, 100 ms마다 콜백; `cnt == 5`에서 LED 토글, `sec == 10`에서 `seconds++` |
| 현재 `PWM_test/Core/Src/main.c` | PSC 83·ARR 999·PWM1·High·Pulse 1000, PWM 시작, `HAL_Delay(10)`으로 상승·하강; 하강 조건은 `brightness > 0` |
| 현재 소스에 남은 문제 | PWM main에 `SystemClock_Config()` 호출이 없음. 함수 정의가 있어도 호출되지 않으면 의도한 84 MHz 설정을 적용하지 못함 |
| 미적용 예제 | 아래 HALGetTick 밝기 변화, TIM3 요청 방식은 현재 소스에 적용하거나 보드에서 검증하지 않음 |

원본 프로젝트 경로는 `C:\Users\heosi\STM32CubeIDE\workspace_2.2.0`입니다. 문서 작성에서는 펌웨어와 모터 설정을 수정하지 않았습니다.

## 1. PSC·CNT·ARR·CCR

다음 계산은 **TIM2 입력 클럭이 84 MHz일 때**를 가정합니다. 타이머 클럭과 CPU 클럭은 설정에 따라 다릅니다. 이 실습의 APB1 분주가 2이면 PCLK1은 42 MHz이고 TIM2 클럭은 84 MHz입니다.

```text
CNT 증가 주파수 = TIM2 클럭 / (PSC + 1)
업데이트 주기 = (PSC + 1) × (ARR + 1) / TIM2 클럭
PWM 주파수 = TIM2 클럭 / ((PSC + 1) × (ARR + 1))
```

- `PSC`: 분주값. PSC 83은 84분주, PSC 8399는 8400분주입니다.
- `CNT`: 타이머를 시작하면 하드웨어가 클럭에 따라 자동으로 올리는 카운터입니다.
- `ARR`: Up 모드에서 CNT가 0부터 ARR까지 센 뒤 다음 주기로 넘어갑니다. ARR 999는 1000카운트입니다.
- `CCR1`: 채널 1 비교값. PWM1·High·Up 조건에서는 `CNT < CCR1`일 때 출력이 High입니다.

| PSC | ARR | 계산 결과 | 용도 |
| ---: | ---: | --- | --- |
| 8399 | 4999 | 0.5초마다 업데이트 | 인터럽트마다 LED 토글하면 켜짐·꺼짐 전체 주기는 1초 |
| 8399 | 999 | 0.1초마다 업데이트 | 5회마다 LED 토글, 10회마다 초 카운터 증가 |
| 83 | 999 | 1 ms 주기, 1 kHz | LED PWM |

## 2. 타이머 인터럽트와 main

```text
HAL_TIM_Base_Start_IT(&htim2)
  → TIM2 업데이트 이벤트와 인터럽트 요청
  → TIM2_IRQHandler()
  → HAL_TIM_IRQHandler(&htim2)
  → HAL_TIM_PeriodElapsedCallback(&htim2)
  → 중단했던 main 실행으로 복귀
```

타이머 하드웨어는 CPU의 main 코드와 독립적으로 카운트합니다. 단일 CPU 코어에서 main과 ISR이 같은 순간에 실행되는 것은 아닙니다. 인터럽트가 들어오면 CPU가 main을 잠시 멈추고 ISR을 실행한 뒤 복귀합니다.

현재 콜백은 다음 구조입니다. 초기화와 시작 코드는 위의 현재 소스 표를 참고합니다.

```c
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t cnt = 0;
    static uint8_t sec = 0;

    if (htim->Instance == TIM2) {
        cnt++;
        sec++;
        if (cnt == 5) {
            BSP_LED_Toggle(LED2);
            cnt = 0;
        }
        if (sec == 10) {
            sec = 0;
            seconds++;
        }
    }
}
/* USER CODE END 4 */
```

`static` 지역 변수는 함수 호출이 끝나도 값이 유지되지만 이름의 접근 범위는 해당 함수입니다. 현재 `seconds`는 `uint8_t`이므로 255 다음에는 0으로 순환하며, 무제한 누적 시간은 아닙니다.

ISR은 짧게 유지합니다. 긴 대기나 밝기 변화 루프는 main에서 처리하고 ISR은 요청만 남기는 구조를 학습했습니다. 현재 `timer_test`는 LED를 콜백에서 직접 토글하며, 선언된 `led_toggle_req`는 사용하지 않습니다.

`volatile`은 공유 변수 접근이 최적화로 생략되는 문제를 피하는 데 쓰지만 원자성이나 동기화를 보장하지 않습니다. 하나의 플래그는 반복 요청을 합쳐 버릴 수 있고, main이 플래그를 읽고 지우는 사이 ISR이 새 요청을 쓰면 그 요청이 사라질 수 있습니다. 모든 이벤트를 세려면 카운터와 안전한 읽기·갱신 방법을 별도로 설계해야 합니다.

### `=!`와 `!=` 오타

```c
if (led_toggle_req =! 0) { /* led_toggle_req = !0: 1 대입, 조건 참 */ }
if (led_toggle_req != 0) { /* 0이 아닌지 비교 */ }
```

`=!`는 부등호 연산자가 아닙니다. `= !0`으로 해석되어 1을 대입하므로 main 루프에서 LED가 빠르게 토글되는 원인이 됩니다.

## 3. 디버거에서 static 변수 보기

실습에서는 `-O0`, `-g3` 빌드 설정을 확인했습니다. 우선 콜백 내부 중단점에서 해당 스택 프레임을 선택하고 `cnt`, `sec`를 조회합니다. main 프레임에서는 함수 지역 변수 이름을 찾지 못할 수 있습니다.

이번 디버깅 당시 timer ELF의 심볼은 `cnt.1`, `sec.0`이었고 다음 Expressions 조회를 사용자가 확인했습니다.

```text
*(unsigned char *)&'cnt.1'
*(unsigned char *)&'sec.0'
```

접미사는 컴파일러가 만든 이름으로 재빌드하거나 코드가 바뀌면 달라질 수 있습니다. 이 표현식을 다른 빌드에 그대로 적용하지 말고 프레임 범위 조회를 먼저 시도합니다.

- `F8`: 다음 중단점까지 Resume.
- `F6`: Step Over로 다음 소스 줄 진행.
- Debug와 C/C++ perspective는 화면 구성입니다. 망치 아이콘의 Build와 실행 재개는 별도 동작입니다.
- 중단점은 정상 실행 시간 흐름을 방해합니다. 멈춘 상태에서 LED 주기나 정확한 PWM 주기를 실측할 수 없습니다.

## 4. TIM2 PWM과 보드 LED

NUCLEO-F446RE의 LD2는 PA5에 연결됩니다. PWM에서는 PA5를 **AF1·TIM2_CH1**로 설정하고 `HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1)`으로 시작합니다.

`BSP_LED_Init(LED2)`를 PWM 핀 설정 뒤에 호출하면 PA5를 일반 GPIO 출력으로 다시 설정할 수 있습니다. PWM 프로젝트에서는 이 충돌을 제거해야 합니다. 현재 PWM main에는 해당 호출이 없습니다.

84 MHz·PSC 83·ARR 999·PWM1·High 조건의 듀티비는 다음과 같습니다.

| CCR1/Pulse | 듀티비 |
| ---: | ---: |
| 0 | 0% |
| 100 | 10% |
| 200 | 20% |
| 500 | 50% |
| 900 | 90% |
| 1000 | 100% |

듀티비와 사람이 느끼는 LED 밝기는 선형 관계가 아닙니다. Pulse 0에서 LED가 꺼졌고 200에서 켜진 것은 사용자 확인 결과입니다. 현재 저장된 초기 Pulse는 1000이며, 루프가 곧 CCR을 0부터 다시 씁니다.

### CubeMX 옵션에서 배운 점

- Clock Source를 Internal로 명시하는 것이 설정 이해에 좋습니다. 다만 미선택만으로 반드시 정지한다고 단정할 수는 없습니다. 리셋 기본 내부 클럭 경로로 PWM이 가능한 경우가 있으며, 외부 클럭 설정은 해당 신호가 필요합니다.
- Auto-reload preload는 ARR 버퍼, Output Compare preload는 CCR 버퍼에 관계됩니다. 활성화하면 일반적으로 다음 Update 이벤트에 새 값이 적용됩니다. 설정에 따라 소프트웨어 UG로 업데이트를 발생시킬 수도 있습니다.
- CCR 버퍼는 대기열이 아닙니다. 1 ms 주기 안에 여러 값을 쓰면 다음 업데이트에서는 마지막 값이 적용됩니다.
- Output polarity High는 PWM 활성 구간의 출력 극성입니다.
- Fast mode는 지원되는 트리거 조건에서 출력 비교 응답 지연을 줄이는 옵션이며 PWM 주파수를 높이는 설정이 아닙니다.

## 5. HAL_Delay 밝기 변화와 unsigned 오류

학습한 상승 루프는 0부터 1000까지 10씩 올리고 각 단계에서 `HAL_Delay(10)`을 호출합니다. 하강 루프는 1000부터 `brightness > 0`인 동안 10씩 내립니다. 하강에서 빠진 0은 다음 상승 루프 첫 단계에서 적용됩니다.

`uint32_t brightness`에 `brightness >= 0`을 쓰면 조건이 항상 참입니다. 0에서 10을 빼면 unsigned 연산으로 4294967286이 되어 CCR에 비정상적인 큰 값을 쓰게 됩니다. 현재 저장된 하강 조건은 `> 0`으로 수정되어 있습니다. 이 조건은 현재처럼 시작값 1000과 간격 10이 맞아떨어지는 예제에 사용합니다.

`HAL_Delay(10)`은 main을 대기시키지만 PWM 하드웨어는 계속 출력합니다. 기본 HAL의 SysTick 시간 기준으로 약 10 ms를 기다리며, 1 kHz PWM이면 대략 10주기에 해당합니다. PWM 경계에 정확히 동기화한 10주기 대기는 아닙니다.

현재 PWM main에는 `HAL_Init()` 뒤의 `SystemClock_Config()` 호출이 빠져 있습니다. 함수 정의는 존재하지만 84 MHz 계산을 실제로 적용하려면 호출 복원이 필요합니다. 문서에서는 수정하지 않았고, 현재 보드에 다운로드된 코드 상태도 새로 확인하지 않았습니다.

## 6. HALGetTick으로 대기 없이 밝기 바꾸기

다음은 **현재 소스에 미적용한 학습 예제**입니다. 기존 `brightness` 선언과 두 for 루프를 각각 교체하는 예제로, 중복 추가하지 않습니다. CubeMX의 기존 GPIO·TIM2 초기화와 클럭 설정은 유지하고, 빠진 `SystemClock_Config()` 호출은 main의 원래 시스템 클럭 설정 위치에 복원해야 합니다.

기본 HAL 구성에서 SysTick은 1 ms마다 `uwTick`을 증가시키며 `HAL_GetTick()`은 현재 값을 읽습니다. `now`는 현재 시간, `last_update`는 마지막 밝기 변경 기준 시간입니다.

```c
/* USER CODE BEGIN PV */
uint32_t brightness = 0;
int32_t step = 10;
uint32_t last_update = 0;
/* USER CODE END PV */
```

초기화 뒤의 기존 `USER CODE 2` 블록을 다음과 같이 구성합니다.

```c
/* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, brightness);
last_update = HAL_GetTick();
/* USER CODE END 2 */
```

기존 while 안의 `USER CODE 3`에서 for 루프를 아래 내용으로 교체합니다.

```c
/* USER CODE BEGIN 3 */
uint32_t now = HAL_GetTick();
if ((uint32_t)(now - last_update) >= 10U) {
    last_update = now;
    int32_t next = (int32_t)brightness + step;

    if (next >= 1000) {
        next = 1000;
        step = -10;
    } else if (next <= 0) {
        next = 0;
        step = 10;
    }

    brightness = (uint32_t)next;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, brightness);
}
/* USER CODE END 3 */
```

unsigned 차이 계산은 Tick이 32비트 최댓값에서 0으로 돌아가는 rollover를 다룹니다. 루프가 충분히 자주 실행되는 조건을 전제로 합니다. 부호 있는 `step`과 중간값 `next`로 감소 시 underflow를 피하며 밝기는 0~1000으로 제한합니다.

`last_update = now` 방식은 루프가 늦어지면 늦어진 시간을 기준으로 다음 단계를 예약하므로 일정한 기준 시각에서 드리프트할 수 있습니다. 밀린 단계를 한꺼번에 따라잡지 않는 간단한 밝기 변화 예제입니다. 대기하지 않으므로 main에서 다른 일을 수행할 수 있습니다.

## 다음 학습 제안과 종료 상태

- TIM2는 1 kHz PWM을 유지하고 TIM3가 10 ms마다 요청을 남기며 main이 밝기를 변경하는 구조: 제안만 했으며 미구현.
- Input Capture와 엔코더 실습: 이번 학습의 후속 주제이며 여기서는 미구현.
- PWM의 `SystemClock_Config()` 호출을 복원한 뒤 실제 클럭과 출력 주기 확인: 현재 남은 확인 항목.

이번 작업은 학습 README 작성과 목록 연결로 종료합니다. 새 실습 실행이나 추가 기능 구현은 진행하지 않았습니다.
