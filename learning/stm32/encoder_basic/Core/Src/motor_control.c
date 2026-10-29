/*
 * motor_control.c
 *
 *  Created on: 2026. 10. 18.
 *      Author: heosi
 */


#include "motor_control.h"
#include <math.h>

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
    		__HAL_TIM_GET_AUTORELOAD(motor->config.pwm_timer) + 1U;
    const uint32_t compare =
    		(uint32_t)((float)period_counts * duty_percent / 100.0f);
    __HAL_TIM_SET_COMPARE(
    		motor->config.pwm_timer,
			motor->config.pwm_channel,
			compare
	);
}

void Motor_Stop(Motor *motor){
	motor->state.target_speed_mm_s = 0.0f;
	motor->state.control_target_speed_mm_s = 0.0f;
	motor->state.speed_error = -motor->state.speed_mm_s;
	motor->state.i_output = 0.0f;
	motor->state.stop_samples = 0;

	Motor_ApplyDuty(motor, 0.0f);
}

HAL_StatusTypeDef Motor_Init(Motor *motor){
	motor->state = (MotorState){
		/* 반복한 바닥 시험에서 목표 부근의 속도 변동이 작았던 값입니다.
		 * Kp가 크면 속도 변화에 따른 듀티 변화도 커집니다. */
		.kp = 0.1f,
		.ki = 0.1f,
		.direction = 1
	};

	Motor_ApplyDuty(motor, 0.0f);

	HAL_GPIO_WritePin(
			motor->config.dir_port,
			motor->config.dir_pin,
			GPIO_PIN_SET
		);

	__HAL_TIM_SET_COUNTER(motor->config.encoder_timer, 0);

	if (HAL_TIM_Encoder_Start(
			motor->config.encoder_timer,
			TIM_CHANNEL_ALL) != HAL_OK)
	{
		return HAL_ERROR;
	}

	motor->state.previous_count =
			__HAL_TIM_GET_COUNTER(motor->config.encoder_timer);

	if (HAL_TIM_PWM_Start(
			motor->config.pwm_timer,
			motor->config.pwm_channel) != HAL_OK)
	{
		Motor_Stop(motor);
		return HAL_ERROR;
	}

	return HAL_OK;
}

void Motor_Update(Motor *motor, uint32_t sample_dt_ms){
	if (sample_dt_ms == 0U){
		return;
	}
	const uint32_t current_count =
			__HAL_TIM_GET_COUNTER(motor->config.encoder_timer);

	int32_t delta_count;

	if (motor->config.encoder_bits == 16U){
		delta_count =
				(int32_t)current_count - (int32_t)motor->state.previous_count;

		if (delta_count > 32767){
			delta_count -= 65536;
		}
		else if (delta_count < -32768){
			delta_count += 65536;
		}
	}
	else {
		delta_count =
				(int32_t)(current_count - motor->state.previous_count);
	}
	motor->state.previous_count = current_count;

	delta_count *= motor->config.encoder_sign;
	motor->state.delta_count = delta_count;

	const float turns =
			(float)delta_count / motor->config.counts_per_turn;

	motor->state.delta_distance_mm =
			turns * motor->config.wheel_circumference_mm;

	motor->state.speed_mm_s =
			motor->state.delta_distance_mm
			* 1000.0f / (float)sample_dt_ms;

	motor->state.rpm =
			turns * 60000.0f / (float)sample_dt_ms;

	const int8_t requested_direction =
			(motor->state.target_speed_mm_s > 0.0f) ? 1 :
			(motor->state.target_speed_mm_s < 0.0f) ? -1 : 0;

	if (requested_direction == 0){
		Motor_Stop(motor);
		return;
	}
	if (requested_direction != motor->state.direction){
		Motor_ApplyDuty(motor, 0.0f);
		motor->state.control_target_speed_mm_s = 0.0f;
		motor->state.i_output = 0.0f;

		if (delta_count == 0){
			motor->state.stop_samples++;
		}
		else{
			motor->state.stop_samples = 0;
		}
		if (motor->state.stop_samples >= 3U){
			HAL_GPIO_WritePin(
				motor->config.dir_port,
				motor->config.dir_pin,
				requested_direction > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET
			);

			motor->state.direction = requested_direction;
			motor->state.stop_samples = 0;
		}

		return;
	}

	motor->state.stop_samples = 0;

	const float dt_s = (float)sample_dt_ms / 1000.0f;

	const float max_change = MOTOR_ACCELERATION_LIMIT_MM_S2 * dt_s;

	float target_change =
			motor->state.target_speed_mm_s - motor->state.control_target_speed_mm_s;

	if (target_change > max_change){
		target_change = max_change;
	}

	else if (target_change < -max_change){
		target_change = -max_change;
	}

	motor->state.control_target_speed_mm_s += target_change;

	motor->state.speed_error =
			motor->state.control_target_speed_mm_s - motor->state.speed_mm_s;

	const float control_error =
			(float)motor->state.direction * motor->state.speed_error;

	const float p_output =
		motor->state.kp * control_error;

	const float i_candidate =
		motor->state.i_output
		+ motor->state.ki * control_error * dt_s;

	const float output_candidate = p_output + i_candidate;

	/* 출력 제한을 더 벗어나게 하는 적분은 저장하지 않습니다. */
	if (!((output_candidate > MOTOR_DUTY_LIMIT_PERCENT &&
		   control_error > 0.0f) ||
		  (output_candidate < 0.0f &&
		   control_error < 0.0f)))
	{
		motor->state.i_output = i_candidate;
	}

	/* 공통 출력 함수에서 듀티 제한과 실제 CCR 갱신을 수행합니다. */
	Motor_ApplyDuty(
		motor,
		p_output + motor->state.i_output
	);
}

void Motor_SetTargetSpeed(Motor *motor, float target_speed_mm_s){
	if (!isfinite(target_speed_mm_s)){ // isfinite()는 NaN·무한대 같은 잘못된 계산값을 걸러냄
		Motor_Stop(motor);
		return;
	}

	if (target_speed_mm_s > MOTOR_SPEED_LIMIT_MM_S){
		target_speed_mm_s = MOTOR_SPEED_LIMIT_MM_S;
	}
	else if (target_speed_mm_s < -MOTOR_SPEED_LIMIT_MM_S){
		target_speed_mm_s = -MOTOR_SPEED_LIMIT_MM_S;
	}

	if (target_speed_mm_s == 0){
		Motor_Stop(motor);
		return;
	}
	motor->state.target_speed_mm_s = target_speed_mm_s;
}
