/*
 * motor_control.h
 *
 *  Created on: 2026. 10. 18.
 *      Author: heosi
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "main.h"

#define MOTOR_DUTY_LIMIT_PERCENT 50.0f
#define MOTOR_SPEED_LIMIT_MM_S 200.0f
#define MOTOR_ACCELERATION_LIMIT_MM_S2 100.0f

// 모터 하나 측정갑과 제어

typedef struct{
	float target_speed_mm_s;
	float control_target_speed_mm_s;
	float speed_mm_s;
	float delta_distance_mm;
	float rpm;

	float duty_percent;
	float speed_error;
	float i_output;

	float kp;
	float ki;

	uint32_t previous_count;
	int32_t delta_count;

	int8_t direction;
	uint8_t stop_samples;
} MotorState;

typedef struct{
	TIM_HandleTypeDef *encoder_timer;
	TIM_HandleTypeDef *pwm_timer;
	uint32_t pwm_channel;

	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;

	uint32_t encoder_bits;
	int8_t encoder_sign;

	float counts_per_turn;
	float wheel_circumference_mm;
} MotorConfig;

typedef struct{
	MotorConfig config;
	MotorState state;
} Motor;

HAL_StatusTypeDef Motor_Init(Motor *moter);

void Motor_SetTargetSpeed(Motor *motor, float target_speed_mm_s);

void Motor_Update(Motor *motor, uint32_t sample_dt_ms);

void Motor_Stop(Motor *motor);

#endif /* INC_MOTOR_CONTROL_H_ */
