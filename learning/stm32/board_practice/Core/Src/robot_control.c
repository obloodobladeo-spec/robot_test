/*
 * robot_control.c
 *
 *  Created on: 2026. 10. 21.
 *      Author: heosi
 */


#include "robot_control.h"
#include <math.h>

#define ROBOT_PI 3.14159265f

void Robot_SetVelocity(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s)
{
	const float turn_speed =
			angular_speed_rad_s * robot->wheel_track_mm * 0.5f;

	float left_target = linear_speed_mm_s - turn_speed;
	float right_target = linear_speed_mm_s + turn_speed;

	if (!isfinite(left_target) || !isfinite(right_target)) {
		Motor_Stop(robot->left_motor);
		Motor_Stop(robot->right_motor);
		return;
	}

	const float max_target =
		fmaxf(fabsf(left_target), fabsf(right_target));

	if (max_target > MOTOR_SPEED_LIMIT_MM_S) {
		const float scale =
			MOTOR_SPEED_LIMIT_MM_S / max_target;

		left_target *= scale;
		right_target *= scale;
	}

    Motor_SetTargetSpeed(robot->left_motor, left_target);
    Motor_SetTargetSpeed(robot->right_motor, right_target);
}

void Robot_SetFeedforward(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s)
{
	/* 정지 명령이 아니면 모든 속도·회전에 적용한다. */
	const uint8_t enable =
		(linear_speed_mm_s != 0.0f || angular_speed_rad_s != 0.0f) ? 1U : 0U;

	Motor_SetFeedforward(robot->left_motor, enable);
	Motor_SetFeedforward(robot->right_motor, enable);
}

static float wrap_pi(float angle)
{
	while (angle > ROBOT_PI) {
		angle -= 2.0f * ROBOT_PI;
	}
	while (angle < -ROBOT_PI) {
		angle += 2.0f * ROBOT_PI;
	}
	return angle;
}

static void heading_hold_reset(HeadingHold *hold)
{
	hold->active = 0U;
	hold->error_rad = 0.0f;
	hold->slow_error_rad = 0.0f;
	hold->integral = 0.0f;
	hold->correction_rad_s = 0.0f;
}

float Robot_HeadingHold(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s,
	uint8_t imu_valid,
	float imu_yaw_rad,
	uint32_t sample_dt_ms)
{
	HeadingHold *hold = &robot->hold;

	/* 회전 명령이 있거나 정지 명령이면 개입하지 않는다. */
	if (angular_speed_rad_s != 0.0f || linear_speed_mm_s == 0.0f ||
	    sample_dt_ms == 0U) {
		heading_hold_reset(hold);
		return angular_speed_rad_s;
	}

	const uint8_t use_imu = (imu_valid != 0U && isfinite(imu_yaw_rad)) ? 1U : 0U;
	const float imu_yaw = HEADING_IMU_SIGN * imu_yaw_rad;
	const float encoder_yaw = robot->state.yaw_rad;
	const int8_t direction = (linear_speed_mm_s > 0.0f) ? 1 : -1;

	/* 직진 시작·방향 전환 시 두 센서의 현재 yaw를 기준으로 잡는다. */
	if (hold->active == 0U || hold->direction != direction) {
		heading_hold_reset(hold);
		hold->active = 1U;
		hold->use_imu = use_imu;
		hold->direction = direction;
		hold->reference_rad = encoder_yaw;
		hold->imu_reference_rad = imu_yaw;
		return 0.0f;
	}

	/* IMU가 끊겼다 돌아오면 IMU 기준만 현재 값으로 다시 잡고 적분은 유지한다.
	 * 그 사이 오차는 엔코더 적분으로 이어 간다. */
	if (use_imu != 0U && hold->use_imu == 0U) {
		hold->imu_reference_rad = imu_yaw
			- wrap_pi(hold->reference_rad - encoder_yaw);
	}
	hold->use_imu = use_imu;

	const float dt_s = (float)sample_dt_ms / 1000.0f;
	const float error = wrap_pi(hold->reference_rad - encoder_yaw);
	const float slow_error = (use_imu != 0U)
		? wrap_pi(hold->imu_reference_rad - imu_yaw)
		: error;

	/* 엔코더 P + (IMU가 엔코더보다 더 본 실제 틀어짐) P */
	const float p_output = HEADING_KP * error
		+ ((use_imu != 0U) ? HEADING_KP_IMU * wrap_pi(slow_error - error) : 0.0f);
	const float i_candidate = hold->integral + slow_error * dt_s;
	float output = p_output + HEADING_KI * i_candidate;

	/* 제한에 걸린 방향으로는 적분을 더 쌓지 않는다. */
	if (output > HEADING_MAX_CORR_RAD_S) {
		output = HEADING_MAX_CORR_RAD_S;
		if (slow_error < 0.0f) {
			hold->integral = i_candidate;
		}
	}
	else if (output < -HEADING_MAX_CORR_RAD_S) {
		output = -HEADING_MAX_CORR_RAD_S;
		if (slow_error > 0.0f) {
			hold->integral = i_candidate;
		}
	}
	else {
		hold->integral = i_candidate;
	}

	hold->error_rad = error;
	hold->slow_error_rad = slow_error;
	hold->correction_rad_s = output;
	return output;
}

void Robot_Update(Robot *robot, uint32_t sample_dt_ms){
	if (sample_dt_ms == 0U){
		return;
	}

	Motor_Update(robot->left_motor, sample_dt_ms);
	Motor_Update(robot->right_motor, sample_dt_ms);

	const float left_mm =
		robot->left_motor->state.delta_distance_mm;

	const float right_mm =
		robot->right_motor->state.delta_distance_mm;

	robot->state.delta_s_mm =
		(left_mm + right_mm) * 0.5f;

	robot->state.delta_yaw_rad =
		(right_mm - left_mm) / robot->wheel_track_mm;

	const float mid_yaw_rad =
		robot->state.yaw_rad
		+ robot->state.delta_yaw_rad * 0.5f;

    robot->state.x_mm +=
        robot->state.delta_s_mm * cosf(mid_yaw_rad);

    robot->state.y_mm +=
        robot->state.delta_s_mm * sinf(mid_yaw_rad);

    robot->state.yaw_rad += robot->state.delta_yaw_rad;
}
