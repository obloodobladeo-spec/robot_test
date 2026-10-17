#include "motor_control.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

typedef struct {
    TIM_HandleTypeDef *encoder;
    uint32_t pwm_channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    uint8_t counter_bits;
    float counts_per_turn;
} MotorConfig;

static const MotorConfig config[MOTOR_COUNT] = {
    {&htim2, TIM_CHANNEL_1, M1_DIR_GPIO_Port, M1_DIR_Pin, 32, 2465.2f},
    {&htim3, TIM_CHANNEL_2, M2_DIR_GPIO_Port, M2_DIR_Pin, 16, 2475.6f}
};

MotorState motors[MOTOR_COUNT];
static uint32_t previous_count[MOTOR_COUNT];
static uint32_t previous_tick;

static void apply_duty(uint32_t index)
{
    float duty = motors[index].duty_percent;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > MOTOR_MAX_DUTY_PERCENT) duty = MOTOR_MAX_DUTY_PERCENT;
    motors[index].duty_percent = duty;
    __HAL_TIM_SET_COMPARE(&htim1, config[index].pwm_channel,
        (uint32_t)((float)(__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U) * duty / 100.0f));
}

void Motor_EmergencyStop(void)
{
    /* Fault handler에서도 호출 가능: HAL tick이나 통신을 기다리지 않습니다. */
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

void Motor_StopAll(void)
{
    for (uint32_t i = 0; i < MOTOR_COUNT; ++i) {
        motors[i].target_mm_s = 0.0f;
        motors[i].duty_percent = 0.0f;
        motors[i].i_output = 0.0f;
        motors[i].stop_samples = 0;
        apply_duty(i);
    }
}

HAL_StatusTypeDef Motor_Init(void)
{
    for (uint32_t i = 0; i < MOTOR_COUNT; ++i) {
        motors[i] = (MotorState){.kp = 0.2f, .ki = 0.1f, .direction = 1};
        apply_duty(i);
        HAL_GPIO_WritePin(config[i].dir_port, config[i].dir_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COUNTER(config[i].encoder, 0);
        if (HAL_TIM_Encoder_Start(config[i].encoder, TIM_CHANNEL_ALL) != HAL_OK)
            return HAL_ERROR;
        previous_count[i] = __HAL_TIM_GET_COUNTER(config[i].encoder);
    }
    previous_tick = HAL_GetTick();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) return HAL_ERROR;
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Motor_EmergencyStop();
        return HAL_ERROR;
    }
    return HAL_OK;
}

void Motor_SetTargets(int16_t motor1_mm_s, int16_t motor2_mm_s)
{
    const int16_t values[MOTOR_COUNT] = {motor1_mm_s, motor2_mm_s};
    for (uint32_t i = 0; i < MOTOR_COUNT; ++i) {
        int32_t value = values[i];
        if (value > MOTOR_MAX_SPEED_MM_S) value = MOTOR_MAX_SPEED_MM_S;
        if (value < -MOTOR_MAX_SPEED_MM_S) value = -MOTOR_MAX_SPEED_MM_S;
        motors[i].target_mm_s = (float)value;
        if (value == 0) {
            motors[i].duty_percent = 0.0f;
            motors[i].i_output = 0.0f;
            motors[i].stop_samples = 0;
            apply_duty(i);
        }
    }
}

void Motor_Update(uint32_t now)
{
    const uint32_t dt_ms = now - previous_tick;
    if (dt_ms < 100U) return;
    previous_tick = now;
    for (uint32_t i = 0; i < MOTOR_COUNT; ++i) {
        MotorState *m = &motors[i];
        const uint32_t count = __HAL_TIM_GET_COUNTER(config[i].encoder);
        int32_t delta;
        if (config[i].counter_bits == 16) {
            delta = (int32_t)count - (int32_t)previous_count[i];
            if (delta > 32767) delta -= 65536;
            else if (delta < -32768) delta += 65536;
        } else {
            delta = (int32_t)(count - previous_count[i]);
        }
        previous_count[i] = count;
        m->raw_count = count;
        m->delta_count = delta;
        const float turns = (float)delta / config[i].counts_per_turn;
        m->speed_mm_s = turns * 345.6f * 1000.0f / (float)dt_ms;
        m->rpm = turns * 60000.0f / (float)dt_ms;
        /* 제어 계산은 main에서 수행하며 인터럽트에서는 계산하지 않습니다. */
        if (dt_ms > 250U) {
            Motor_StopAll();
            continue;
        }
        const int8_t requested = (m->target_mm_s > 0.0f) ? 1 :
            (m->target_mm_s < 0.0f) ? -1 : 0;
        if (requested == 0) {
            m->duty_percent = 0.0f;
            m->i_output = 0.0f;
            m->stop_samples = 0;
        } else if (requested != m->direction) {
            m->duty_percent = 0.0f;
            m->i_output = 0.0f;
            m->stop_samples = (delta == 0) ? (uint8_t)(m->stop_samples + 1U) : 0;
            if (m->stop_samples >= 3U) {
                HAL_GPIO_WritePin(config[i].dir_port, config[i].dir_pin,
                    requested > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
                m->direction = requested;
                m->stop_samples = 0;
            }
        } else {
            m->stop_samples = 0;
            const float error = (float)m->direction * (m->target_mm_s - m->speed_mm_s);
            const float p = m->kp * error;
            const float candidate = m->i_output + m->ki * error * ((float)dt_ms / 1000.0f);
            const float output = p + candidate;
            if (!((output > MOTOR_MAX_DUTY_PERCENT && error > 0.0f) ||
                  (output < 0.0f && error < 0.0f)))
                m->i_output = candidate;
            m->duty_percent = p + m->i_output;
        }
        apply_duty(i);
    }
}
