/*
 * robot_control.c
 *
 *  Created on: 2026. 10. 21.
 *      Author: heosi
 */


#include "robot_control.h"
#include <math.h>

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
