#include "mdd10a_test.h"

#define TEST_DUTY_PERCENT 20U
#define RUN_TIME_MS 1000U
#define STOP_TIME_MS 3000U
#define DIR_PINS (GPIO_PIN_0 | GPIO_PIN_1)
#define PWM_PINS (GPIO_PIN_8 | GPIO_PIN_9)

static uint8_t attempted;

/* Error path: disconnect TIM1 from the two PWM pins and drive them LOW.
 * This also handles partially successful HAL_TIM_PWM_Start calls.
 * These four pins must belong exclusively to this example.
 */
static void fail_safe(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE | TIM_BDTR_AOE);
    CLEAR_BIT(TIM1->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
    HAL_GPIO_WritePin(GPIOA, PWM_PINS, GPIO_PIN_RESET);
    gpio.Pin = PWM_PINS;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOC, DIR_PINS, GPIO_PIN_RESET);
    gpio.Pin = DIR_PINS;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static uint8_t config_valid(TIM_HandleTypeDef *timer)
{
    if ((timer == NULL) || (timer->Instance != TIM1) ||
        (__get_IPSR() != 0U) || (__get_PRIMASK() != 0U)) {
        return 0U;
    }
    if ((__HAL_RCC_TIM1_IS_CLK_ENABLED() == 0U) ||
        (__HAL_RCC_GPIOA_IS_CLK_ENABLED() == 0U) ||
        (__HAL_RCC_GPIOC_IS_CLK_ENABLED() == 0U)) {
        return 0U;
    }
    /* PA8/PA9 = AF1 TIM1, PC0/PC1 = push-pull outputs. */
    if (((GPIOA->MODER >> 16U) & 0xFU) != 0xAU ||
        ((GPIOA->AFR[1] & 0xFFU) != 0x11U) ||
        ((GPIOA->OTYPER & PWM_PINS) != 0U) ||
        ((GPIOC->MODER & 0xFU) != 0x5U) ||
        ((GPIOC->OTYPER & DIR_PINS) != 0U)) {
        return 0U;
    }
    /* Continuous edge-aligned upcounting, internal clock, PWM1 active high.
     * Reject an already running timer, slave mode and complementary outputs.
     * No break/automatic-output configuration is used by this simple test.
     */
    if ((TIM1->CR1 & (TIM_CR1_CEN | TIM_CR1_DIR | TIM_CR1_CMS |
                      TIM_CR1_OPM | TIM_CR1_UDIS)) != 0U ||
        (TIM1->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U ||
        (TIM1->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S |
                        TIM_CCMR1_OC1M | TIM_CCMR1_OC2M)) !=
            (TIM_OCMODE_PWM1 | (TIM_OCMODE_PWM1 << 8U)) ||
        (TIM1->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E |
                       TIM_CCER_CC1P | TIM_CCER_CC2P |
                       TIM_CCER_CC1NE | TIM_CCER_CC2NE |
                       TIM_CCER_CC1NP | TIM_CCER_CC2NP)) != 0U ||
        (TIM1->BDTR & (TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_BKE)) != 0U ||
        TIM1->RCR != 0U || TIM1->ARR < 4U || TIM1->ARR > 65534U ||
        TIM1->CCR1 != 0U || TIM1->CCR2 != 0U ||
        timer->ChannelState[0] != HAL_TIM_CHANNEL_STATE_READY ||
        timer->ChannelState[1] != HAL_TIM_CHANNEL_STATE_READY) {
        return 0U;
    }
    return 1U;
}

static void set_duty(TIM_HandleTypeDef *timer, uint32_t motor1,
                     uint32_t motor2)
{
    /* Upcount PWM1: duty = CCR / (ARR + 1), rounded down. */
    const uint32_t period_ticks = __HAL_TIM_GET_AUTORELOAD(timer) + 1U;
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1,
                          period_ticks * motor1 / 100U);
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2,
                          period_ticks * motor2 / 100U);
    /* Latch both zero/nonzero CCR preload values immediately. No timer IRQs
     * or DMA may be enabled for this example; UG also resets the counter.
     */
    timer->Instance->EGR = TIM_EVENTSOURCE_UPDATE;
    __HAL_TIM_CLEAR_FLAG(timer, TIM_FLAG_UPDATE);
}

static void stop_and_wait(TIM_HandleTypeDef *timer)
{
    set_duty(timer, 0U, 0U);
    HAL_Delay(STOP_TIME_MS);
}

static void test_direction(TIM_HandleTypeDef *timer)
{
    set_duty(timer, TEST_DUTY_PERCENT, 0U);
    HAL_Delay(RUN_TIME_MS);
    stop_and_wait(timer);
    set_duty(timer, 0U, TEST_DUTY_PERCENT);
    HAL_Delay(RUN_TIME_MS);
    stop_and_wait(timer);
    set_duty(timer, TEST_DUTY_PERCENT, TEST_DUTY_PERCENT);
    HAL_Delay(RUN_TIME_MS);
    stop_and_wait(timer);
}

Mdd10aTestResult MDD10A_TestRunOnce(TIM_HandleTypeDef *timer)
{
    if (attempted != 0U) {
        return MDD10A_TEST_ALREADY_ATTEMPTED;
    }
    attempted = 1U;
    if (config_valid(timer) == 0U || TIM1->DIER != 0U) {
        fail_safe();
        return MDD10A_TEST_BAD_CONFIG;
    }
    HAL_GPIO_WritePin(GPIOC, DIR_PINS, GPIO_PIN_RESET);
    set_duty(timer, 0U, 0U);
    /* HAL enables advanced-timer MOE as well as the selected channel. */
    if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1) != HAL_OK) {
        fail_safe();
        return MDD10A_TEST_START_FAILED;
    }
    if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2) != HAL_OK) {
        fail_safe();
        return MDD10A_TEST_START_FAILED;
    }
    HAL_Delay(1000U);
    test_direction(timer);
    /* The previous stop has latched PWM=0 and waited 3 seconds. */
    HAL_GPIO_WritePin(GPIOC, DIR_PINS, GPIO_PIN_SET);
    HAL_Delay(100U);
    test_direction(timer);
    set_duty(timer, 0U, 0U);
    /* Keep PWM channels running at 0% so the pins remain actively LOW. */
    return MDD10A_TEST_OK;
}

/* Standalone application entry. No generated main.c is used. */
static TIM_HandleTypeDef pwm_timer;
volatile Mdd10aTestResult mdd10a_result = MDD10A_TEST_BAD_CONFIG;

void Error_Handler(void)
{
    fail_safe();
    __disable_irq();
    while (1) { /* Inspect mdd10a_result in the debugger. */ }
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clocks = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16U;
    osc.PLL.PLLN = 336U;
    osc.PLL.PLLP = RCC_PLLP_DIV4;
    osc.PLL.PLLQ = 7U;
    osc.PLL.PLLR = 2U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }
    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV2;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    CLEAR_BIT(RCC->DCKCFGR, RCC_DCKCFGR_TIMPRE);
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void motor_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* Hold both PWM pins LOW before connecting them to TIM1. */
    HAL_GPIO_WritePin(GPIOA, PWM_PINS, GPIO_PIN_RESET);
    gpio.Pin = PWM_PINS;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOC, DIR_PINS, GPIO_PIN_RESET);
    gpio.Pin = DIR_PINS;
    HAL_GPIO_Init(GPIOC, &gpio);
}

static void motor_timer_init(void)
{
    TIM_OC_InitTypeDef channel = {0};
    TIM_BreakDeadTimeConfigTypeDef safety = {0};
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();
    pwm_timer.Instance = TIM1;
    pwm_timer.Init.Prescaler = 0U;
    pwm_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm_timer.Init.Period = 4199U; /* 84 MHz / 4200 = 20 kHz. */
    pwm_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm_timer.Init.RepetitionCounter = 0U;
    pwm_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&pwm_timer) != HAL_OK) { Error_Handler(); }
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    channel.OCIdleState = TIM_OCIDLESTATE_RESET;
    channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&pwm_timer, &channel, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&pwm_timer, &channel, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    safety.OffStateRunMode = TIM_OSSR_DISABLE;
    safety.OffStateIDLEMode = TIM_OSSI_DISABLE;
    safety.LockLevel = TIM_LOCKLEVEL_OFF;
    safety.DeadTime = 0U;
    safety.BreakState = TIM_BREAK_DISABLE;
    safety.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    safety.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&pwm_timer, &safety) != HAL_OK) {
        Error_Handler();
    }
    gpio.Pin = PWM_PINS;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

void SysTick_Handler(void) { HAL_IncTick(); }
void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }

int main(void)
{
    if (HAL_Init() != HAL_OK) { Error_Handler(); }
    motor_gpio_init();
    SystemClock_Config();
    motor_timer_init();
    mdd10a_result = MDD10A_TestRunOnce(&pwm_timer);
    if (mdd10a_result != MDD10A_TEST_OK) { Error_Handler(); }
    while (1) { __WFI(); } /* No automatic repeat. PWM remains 0%. */
}
