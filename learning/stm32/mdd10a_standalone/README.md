# MDD10A 독립 시험 프로젝트

`Core/Src/mdd10a_test.c`에 유일한 `main()`과 GPIO/clock/TIM1 초기화,
HAL MSP, SysTick 및 fault handler를 함께 정의했습니다. `main.c`는 없으며
다른 프로젝트의 초기화 함수나 소스에 의존하지 않습니다. C 파일명은 시작
함수 이름과 무관합니다. Reset_Handler가 C 런타임을 준비한 뒤 `main()`을 호출합니다.
HAL/CMSIS, startup assembly와 linker script는 MCU 실행을 위한 필수 지원입니다.

## 구성과 추가한 내용

```text
mdd10a_standalone/
  .project / .cproject          CubeIDE import 및 Debug/Release 설정
  build.ps1                    ARM GCC 독립 컴파일/ELF 링크/검증
  STM32F446RETX_FLASH.ld        FLASH 512KB, RAM 128KB 링커 설정
  Core/Inc/                    시험 API와 ST HAL configuration
  Core/Src/mdd10a_test.c        main, 초기화, 시험 시퀀스, IRQ
  Core/Src/system_stm32f4xx.c   SystemInit, SystemCoreClock
  Core/Src/syscalls.c           미지원 console/filesystem의 명시적 오류 반환
  Core/Startup/                STM32F446RE vector table, Reset_Handler
  Drivers/                    로컬 복사한 ST HAL/CMSIS 지원
```

기존 `446re_sample_project`의 ST HAL/CMSIS 헤더, 필요한 HAL 소스 12개,
system/startup/FLASH linker와 CubeIDE 설정을 복사해 자립 구조로 만들었습니다.
기존 `examples/mdd10a`의 시험 함수를 새 프로젝트에 복사하고 초기화와 main을
추가했습니다. 기존 프로젝트 및 기존 예제는 변경하지 않았습니다.
필요 없는 ADC/I2C/UART/BSP 구현은 추가하지 않았습니다. ST 원본의 저작권/라이선스
표시는 복사 파일에 유지되어 있으며 배포 전 해당 라이선스를 확인하세요.
소스 집합은 CubeIDE와 build.ps1 모두 `Core`/`Drivers` 아래 C 및 assembly입니다.
`.ioc`는 없으며 CubeMX 생성물에 의존하지 않는 수동 프로젝트입니다.
CubeMX 프로젝트로 전환/재생성하면 main.c 생성과 설정 덮어쓰기 가능성이 있으므로
그대로 Generate Code를 적용하지 마세요.

## 기존 핀 설정과 새 프로젝트

| MCU 핀/주변장치 | 기존 446re_sample_project | 새 프로젝트 |
|---|---|---|
| PA8 / TIM1_CH1 | 미사용 | PWM1 → MDD10A PWM1 |
| PA9 / TIM1_CH2 | 미사용 | PWM2 → MDD10A PWM2 |
| PC0 | 미사용 | GPIO push-pull DIR1 |
| PC1 | 미사용 | GPIO push-pull DIR2 |
| PA6, PA7, PB0, PB1 / TIM3 | PWM CH1~4 | 미사용 |
| PB8, PB9 / I2C1 | LCD 통신 | 미사용 |
| PA0 / ADC1 | ADC 입력 | 미사용 |
| PA2, PA3 / USART2 | UART TX/RX | 미사용 |
| PA5 / LED2 | LED 출력(BSP) | 미사용 |
| PC13 / user button | EXTI/BSP | 미사용 |
| TIM2 | 기본 타이머 인터럽트 | 미사용 |
| SysTick | HAL tick | HAL tick 및 HAL_Delay 유지 |
| GND | 공통 기준 | MDD10A GND와 공통 접지 |

새 펌웨어는 내부 HSI를 PLL로 사용하므로 외부 발진기가 필요하지 않습니다.
SWD PA13/PA14는 디버깅용 기본 상태를 유지합니다.

기존 CubeIDE 설정의 보드명은 `NUCLEO-F446RE`지만 실물 모델/리비전은 확인하지
않았습니다. 아래는 **NUCLEO-F446RE, MB1136인 경우**의 접근 위치입니다.
커넥터 pin 번호는 MCU 패키지 pin 번호와 다릅니다.

| 신호 | Arduino | ST Morpho | 조건 |
|---|---|---|---|
| PA8 PWM1 | D7 / CN9 pin8 | CN10 pin23 | UM1724 Table29 |
| PA9 PWM2 | D8 / CN5 pin1 | CN10 pin21 | UM1724 Table29 |
| PC1 DIR2 | A4 | CN7 pin36 | PC1 **또는 PB9**, bridge 설정 의존 |
| PC0 DIR1 | A5 | CN7 pin38 | PC0 **또는 PB8**, bridge 설정 의존 |
| GND | GND | CN10 pin20 또는 pin9 | 보드 silk 및 방향 확인 |

UM1724 Rev17 Table10에서 SB56/SB51 ON, SB46/SB52 OFF이면 PC1/PC0가
A4/A5 및 CN7에 연결됩니다. 반대로 SB46/SB52 ON, SB56/SB51 OFF이면
PB9/PB8이 연결됩니다. **Morpho CN7도 bridge와 무관한 PC0/PC1 직결이라고
가정하면 안 됩니다.** 보드 리비전과 실제 솔더 브리지를 확인해야 합니다.
솔더 브리지 변경은 수행하지 않았습니다.
출처: [ST UM1724, Table10 및 Table29](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf).

입력 커넥터 순서 `GND/PWM2/DIR2/PWM1/DIR1`은 사용자 제공 기준이며 실물
표시와 대조하세요. STM32는 USB 전원, MDD10A는 배터리+퓨즈로 공급하고
배터리 양극을 MCU 신호에 연결하지 않습니다. 두 장치 GND는 공통으로 연결합니다.

## 클럭, PWM와 동작

HSI 16MHz, PLL M16/N336/P4 → SYSCLK/HCLK 84MHz,
APB1 /2 = 42MHz, APB2 /1 = 84MHz, TIMPRE=0 → TIM1 84MHz.
HAL_RCC_ClockConfig가 SystemCoreClock와 SysTick을 갱신하며,
SysTick_Handler가 HAL_IncTick을 호출하여 HAL_Delay가 진행됩니다.

TIM1 internal clock, up/edge aligned, PWM1 active high, PSC=0, ARR=4199,
RCR=0, CH1/CH2 Pulse=0입니다. `84MHz / ((0+1)*(4199+1)) = 20kHz`.
20% CCR은 `(ARR+1)*20/100 = 840`으로 ARR 기반 계산합니다.
PA8/PA9 AF1이며 complementary output, break, IRQ/DMA는 사용하지 않습니다.
HAL_TIM_PWM_Start가 CH1/CH2와 advanced timer MOE를 활성화합니다.

GPIO PWM LOW → clock/TIM1 초기화 → PWM 둘 다0 및 설정 검사 → HAL 시작 →
1초 대기 → DIR LOW에서 모터1 20%1초/정지3초 → 모터2 같은 시험 →
동시 같은 시험 → PWM0으로3초 정지 후 DIR HIGH →100ms → 세 시험 반복 →
최종 둘 다0. 약25.1초이며 main에서 한 번 호출 후 WFI 대기합니다.
재부팅/리셋은 새 시험을 시작합니다. 최종 0% PWM은 계속 활성화되어 LOW 유지합니다.
오류/fault/초기화/HAL 시작 실패는 MOE 및 채널/타이머를 끄고 PWM 핀을 GPIO LOW로
강제합니다. `mdd10a_result`로 정상 완료와 시험 오류 상태를 디버거에서 확인합니다.

3초는 물리적 정지 감지가 아니므로 실제 관성에 맞게 늘리세요. 방향의 전진/후진은
모터 배선에 따라 달라집니다. 바퀴를 띄우고 모터를 고정해 시험하며 전원 차단을
준비하세요. PWM0의 brake/coast 동작은 드라이버 사양에 따릅니다.
부팅 초기에는 GPIO high impedance이므로 필요하면 PWM 외부 pull-down을 사용하고
신호 LOW가 확보된 뒤 모터 전원을 공급하세요. 실제 보드/모터 시험은 수행하지 않았습니다.

## 독립 빌드

프로젝트 루트에서:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1
```

다른 설치 위치에서는 `-ToolchainBin '...\tools\bin'`을 전달하세요.
툴체인만 설치 위치를 사용하며 소스/지원파일은 이 프로젝트 내부입니다.
결과는 `build/mdd10a_standalone.elf`, `.map`, `.hex`입니다.
newlib nano/nosys, Cortex-M4 hard-float로 링크하며 application은 warnings-as-errors,
ST Drivers의 unused parameter 경고만 제외합니다. ELF main 심볼1개와 main.c 없음도 검사합니다.

## CubeIDE import/build/debug/upload

1. File → Import → General → Existing Projects into Workspace에서 이 폴더 선택.
   원본 프로젝트를 동시에 복사/변경할 필요가 없습니다.
2. `mdd10a_standalone`의 Debug 설정을 선택하여 Project → Build Project.
   산출물은 `Debug/mdd10a_standalone.elf`입니다. Release 설정도 제공합니다.
3. 실제 연결 확인 후 Debug As → STM32 C/C++ Application에서 ST-LINK/SWD와
   이 프로젝트 ELF를 선택합니다. 다운로드 전 모터 전원을 끄고 핀/접지를 확인하세요.
4. 업로드/디버그 Resume는 시험 실행을 시작합니다. breakpoint로 오래 멈추면 PWM이
   마지막 duty로 유지될 수 있으므로 모터 전원이 켜진 상태로 운전 중 중단하지 마세요.

이 문서의 업로드 절차는 사용자 사용 안내이며 에이전트는 업로드/실행하지 않았습니다.
CubeIDE headless 검증은 실제 작업용 workspace와 분리한
`$env:TEMP\mdd10a-standalone-cubeide-validation`을 사용했습니다.
Eclipse workspace를 프로젝트 내부에 두면 import 경로 중첩으로 실패할 수 있습니다.
`main.c`가 없어도 startup의 main 심볼 참조로 정상 링크됩니다.

## 확인한 빌드 결과

ARM GCC 수동 빌드 및 CubeIDE 2.2.0 headless Debug import/build 모두 최종 ELF
링크에 성공했습니다. CubeIDE 결과는 **0 errors, 0 warnings**입니다.
두 경로 모두 text=10516, data=16, bss=1648 bytes이며 main 심볼1개입니다.
startup의 Reset_Handler/SystemInit 및 실제 SysTick/fault handler가 ELF에 포함됩니다.
빌드 검증은 코드 링크만 확인하며 실제 파형/전원/모터 정지는 보드에서 검증해야 합니다.
