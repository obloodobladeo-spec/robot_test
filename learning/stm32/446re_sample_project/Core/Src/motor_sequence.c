#include "motor_sequence.h"

#define TEST_DUTY_PERCENT 20U
#define STOP_MS 3000U
#define RUN_MS 1000U
#define SPIN_MS 3000U
#define SETTLE_MS 1U

/* 좌우 대칭 장착과 동일한 모터 배선 극성을 전제로 합니다.
 * 먼저 바퀴를 띄워 좌우가 차체 기준 서로 반대 방향으로 도는지 확인합니다.
 * 같은 전진 방향으로 돌면 SPIN_DIR2_STATE만 GPIO_PIN_SET으로 변경합니다.
 */
#define SPIN_DIR1_STATE GPIO_PIN_RESET
#define SPIN_DIR2_STATE GPIO_PIN_RESET

typedef enum { STEP_STOP, STEP_DIR1_LOW, STEP_DIR1_HIGH,
               STEP_DIR2_LOW, STEP_DIR2_HIGH, STEP_RUN1, STEP_RUN2,
               STEP_SPIN_DIR, STEP_RUN_BOTH } StepAction;

typedef struct {
    uint32_t duration_ms;
    StepAction action;
} MotorStep;

/* 개별 모터 시험 뒤 두 모터를 함께 구동하는 3초 회전 시험을 수행합니다.
 * DIR 변경 단계는 반드시 PWM=0인 정지 단계 뒤에만 옵니다.
 */
static const MotorStep steps[MOTOR_TEST_DONE + 1] = {
    [MOTOR_TEST_IDLE] = {0U, STEP_STOP},
    [MOTOR_TEST_STARTUP_WAIT] = {STOP_MS, STEP_STOP},
    [MOTOR_TEST_M1_LOW_SETTLE] = {SETTLE_MS, STEP_DIR1_LOW},
    [MOTOR_TEST_M1_LOW_RUN] = {RUN_MS, STEP_RUN1},
    [MOTOR_TEST_STOP_AFTER_M1_LOW] = {STOP_MS, STEP_STOP},
    [MOTOR_TEST_M1_HIGH_SETTLE] = {SETTLE_MS, STEP_DIR1_HIGH},
    [MOTOR_TEST_M1_HIGH_RUN] = {RUN_MS, STEP_RUN1},
    [MOTOR_TEST_STOP_AFTER_M1_HIGH] = {STOP_MS, STEP_STOP},
    [MOTOR_TEST_M2_LOW_SETTLE] = {SETTLE_MS, STEP_DIR2_LOW},
    [MOTOR_TEST_M2_LOW_RUN] = {RUN_MS, STEP_RUN2},
    [MOTOR_TEST_STOP_AFTER_M2_LOW] = {STOP_MS, STEP_STOP},
    [MOTOR_TEST_M2_HIGH_SETTLE] = {SETTLE_MS, STEP_DIR2_HIGH},
    [MOTOR_TEST_M2_HIGH_RUN] = {RUN_MS, STEP_RUN2},
    [MOTOR_TEST_STOP_BEFORE_SPIN] = {STOP_MS, STEP_STOP},
    [MOTOR_TEST_SPIN_SETTLE] = {SETTLE_MS, STEP_SPIN_DIR},
    [MOTOR_TEST_SPIN_RUN] = {SPIN_MS, STEP_RUN_BOTH},
    [MOTOR_TEST_DONE] = {0U, STEP_STOP}
};

volatile MotorTestPhase motor_test_phase = MOTOR_TEST_IDLE;
static TIM_HandleTypeDef *pwm_timer;
static uint32_t phase_started;
static uint32_t test_compare;

/* 초기화 검사: 현재 예제의 TIM1/핀 설정으로만 실행합니다. 엔코더 TIM2/3는
 * 건드리지 않습니다. 실패 시 main의 기존 Error_Handler가 PWM 핀을 LOW로 만듭니다.
 */
static uint8_t config_valid(TIM_HandleTypeDef *timer)
{
    if (timer == NULL || timer->Instance != TIM1 ||
        __get_IPSR() != 0U || __get_PRIMASK() != 0U ||
        __HAL_RCC_TIM1_IS_CLK_ENABLED() == 0U ||
        __HAL_RCC_GPIOA_IS_CLK_ENABLED() == 0U ||
        __HAL_RCC_GPIOC_IS_CLK_ENABLED() == 0U) {
        return 0U;
    }
    if (((GPIOA->MODER >> 16U) & 0xFU) != 0xAU ||
        (GPIOA->AFR[1] & 0xFFU) != 0x11U ||
        (GPIOA->OTYPER & (GPIO_PIN_8 | GPIO_PIN_9)) != 0U ||
        (GPIOC->MODER & 0xFU) != 0x5U ||
        (GPIOC->OTYPER & (GPIO_PIN_0 | GPIO_PIN_1)) != 0U ||
        (TIM1->CR1 & (TIM_CR1_CEN | TIM_CR1_DIR | TIM_CR1_CMS |
                      TIM_CR1_OPM | TIM_CR1_UDIS)) != 0U ||
        (TIM1->SMCR & (TIM_SMCR_SMS | TIM_SMCR_ECE)) != 0U ||
        (TIM1->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S |
                        TIM_CCMR1_OC1M | TIM_CCMR1_OC2M)) !=
            (TIM_OCMODE_PWM1 | (TIM_OCMODE_PWM1 << 8U)) ||
        (TIM1->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC1P |
                       TIM_CCER_CC2P | TIM_CCER_CC1NE | TIM_CCER_CC2NE |
                       TIM_CCER_CC1NP | TIM_CCER_CC2NP)) != 0U ||
        (TIM1->BDTR & (TIM_BDTR_MOE | TIM_BDTR_AOE | TIM_BDTR_BKE)) != 0U ||
        TIM1->DIER != 0U || TIM1->RCR != 0U ||
        TIM1->ARR < 4U || TIM1->ARR > 65534U ||
        TIM1->CCR1 != 0U || TIM1->CCR2 != 0U) {
        return 0U;
    }
    return 1U;
}

/* 계산/출력: PWM1 up-count의 duty는 CCR/(ARR+1)입니다.
 * CCR preload를 UG로 즉시 반영하여 0%가 실제 적용된 뒤 DIR을 변경합니다.
 * 이 TIM1에는 update IRQ/DMA가 없으며 UG는 엔코더 타이머에 영향을 주지 않습니다.
 */
static void set_outputs(uint32_t motor1_compare, uint32_t motor2_compare)
{
    __HAL_TIM_SET_COMPARE(pwm_timer, TIM_CHANNEL_1, motor1_compare);
    __HAL_TIM_SET_COMPARE(pwm_timer, TIM_CHANNEL_2, motor2_compare);
    pwm_timer->Instance->EGR = TIM_EVENTSOURCE_UPDATE;
    __HAL_TIM_CLEAR_FLAG(pwm_timer, TIM_FLAG_UPDATE);
}

static void enter_phase(MotorTestPhase phase)
{
    const StepAction action = steps[phase].action;
    /* 개별 시험에서는 한쪽만, 회전 시험에서는 양쪽에 20%를 적용합니다.
     * DIR 변경/정지/완료 단계에서는 양쪽 PWM을 먼저 0으로 반영합니다.
     */
    set_outputs((action == STEP_RUN1 || action == STEP_RUN_BOTH) ? test_compare : 0U,
                (action == STEP_RUN2 || action == STEP_RUN_BOTH) ? test_compare : 0U);
    switch (action) {
    case STEP_DIR1_LOW: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); break;
    case STEP_DIR1_HIGH: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); break;
    case STEP_DIR2_LOW: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); break;
    case STEP_DIR2_HIGH: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); break;
    case STEP_SPIN_DIR:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SPIN_DIR1_STATE);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SPIN_DIR2_STATE);
        break;
    default: break;
    }
    /* 실제 출력 적용 시점부터 측정하므로 늦게 호출돼도 정지시간을 줄이지 않습니다. */
    phase_started = HAL_GetTick();
    motor_test_phase = phase;
}

HAL_StatusTypeDef MotorSequence_Init(TIM_HandleTypeDef *timer)
{
    if (motor_test_phase != MOTOR_TEST_IDLE) {
        return HAL_ERROR; /* 완료 후에도 자동 재시작 금지. */
    }
    motor_test_phase = MOTOR_TEST_FAILED;
    if (config_valid(timer) == 0U) {
        return HAL_ERROR;
    }
    pwm_timer = timer;
    test_compare = ((__HAL_TIM_GET_AUTORELOAD(timer) + 1U) * TEST_DUTY_PERCENT) / 100U;
    set_outputs(0U, 0U);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2) != HAL_OK) {
        return HAL_ERROR; /* 즉시 호출자의 Error_Handler로 LOW 강제. */
    }
    /* HAL은 advanced timer MOE도 켭니다. 이후 3초 대기가 startup 여유를 포함합니다. */
    enter_phase(MOTOR_TEST_STARTUP_WAIT);
    return HAL_OK;
}

void MotorSequence_Task(uint32_t now)
{
    const MotorTestPhase phase = motor_test_phase;
    if (phase < MOTOR_TEST_STARTUP_WAIT || phase >= MOTOR_TEST_DONE) {
        return;
    }
    /* unsigned 차분으로 HAL tick의 32-bit wrap을 처리합니다. 한 번에 한 단계만 전환. */
    if ((uint32_t)(now - phase_started) >= steps[phase].duration_ms) {
        enter_phase((MotorTestPhase)(phase + 1));
    }
}
