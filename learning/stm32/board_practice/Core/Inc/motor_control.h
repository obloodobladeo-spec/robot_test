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
/* 방향 전환 전 정지 확인: 엔코더 변화 0이 연속 이 횟수(100 ms 주기)면 멈춘 것으로 본다. */
#define MOTOR_STOP_CONFIRM_SAMPLES 3U

/* 기본 듀티 피드포워드: duty = offset + slope * |제어 목표 속도|.
 * 바퀴별 offset/slope는 MotorConfig에 둔다(main.c). 0으로 빌드하면 기존 PI만 사용한다.
 * 근거: 2026-11-21 바닥 고정 듀티 시험(9~14%, 전진). 같은 듀티에서 오른쪽이 약 5% 느렸다. */
#define FEEDFORWARD_ENABLE 1U

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
	float feedforward_percent;  /* 현재 기본 듀티 (상승 중 값 포함) */

	float kp;
	float ki;

	uint32_t previous_count;
	int32_t delta_count;

	int8_t direction;
	uint8_t stop_samples;
	uint8_t feedforward_enable; /* 1 = 피드포워드 적용 명령 */
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

	float ff_offset_percent;        /* 정지 마찰 보상 (속도 0 근처 기본 듀티) */
	float ff_slope_percent_per_mm_s;
} MotorConfig;

typedef struct{
	MotorConfig config;
	MotorState state;
} Motor;

HAL_StatusTypeDef Motor_Init(Motor *moter);

void Motor_SetTargetSpeed(Motor *motor, float target_speed_mm_s);

void Motor_Update(Motor *motor, uint32_t sample_dt_ms);

void Motor_Stop(Motor *motor);

void Motor_SetFeedforward(Motor *motor, uint8_t enable);

#endif /* INC_MOTOR_CONTROL_H_ */
