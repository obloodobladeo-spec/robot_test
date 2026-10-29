/*
 * motor_control.c
 *
 *  Created on: 2026. 10. 18.
 *      Author: heosi
 */


#include "motor_control.h"

static void Motor_ApplyDuty(Motor *motor, float duty_percent)
{
    if (duty_percent < 0.0f){
        duty_percent = 0.0f;
    }
    else if (duty_percent > MOTOR_DUTY_LIMIT_PERCENT){
        duty_percent = MOTOR_DUTY_LIMIT_PERCENT;
    }

    motor->state.duty_percent = duty_percent;

    const uint32_t period_counts =
    		__HAL_TIM_GET_AUTORELOAD(motor->config.pwm_timer);
    const uint32_t compare =
    		(uint32_t)((float)period_count * duty_percent / 100.0f);
    __HAL_TIM_SET_COMPARE(
    		motor->config.pwm_timer,
			motor->config.pwm_channel,
			compare
	);
}

void Motor_Stop(Motor *motor){
	motor->state.target_speed_mm_s = 0.0f;
	motor->state.i_output = 0.0f;
	motor->state.stop_samples = 0;

	Motor_ApplyDuty(Motor, 0.0f);
}

HAL_StatusTypeDef Motor_Init(Motor *motor){
	motor->state = (MotorState){
		.kp = 1.0f,
		.ki = 1.0f,
		.direction = 1
	};

	Motor_ApplyDuty(motor, 0.0f);

	HAL_GPIO_WritePin(
			motor->config.dir_port,
			motor-config.dir_pin,
			GPIO_PIN_SET
		);

	HAL_TIM_SET_COUNTER(motorr->config.encoder_timer, 0);

	if (HAL_TIM_Encoder_Start(
			motor->config.encoder_timer,
			TIM_CHANNEL_ALL) != HAL_OK)
	{
		return HAL_ERROR;
	}

	motor->state.previous_count =
			__HAL_TIM_GET_COUNTER(moter->config.encoder_timer);

	if (HAL_TIM_PWM_Start(
			motor->config.pwm_timer,
			motor->config.pwm_channel) != HAL_OK)
	{
		Moter_Stop(motor);
		return HAL_ERROR;
	}

	return HAL_OK;
}

void Moter_Update(Moter *moter, uint32_t sample_dt_ms){
	if (sample_dt_ms == 0U){
		return;
	}
	const current_count =
			__HAL_TIM_GET_COUNTER(moter->config.encoder_timer);

	int32_t delta_count;

	if (moter->config.encoder_bits == 16U){
		delta_count = current_count - moter->state.previous_count;
	}
	if (delata_count > 32767){
		delta_count -= 65536;
	}

}
