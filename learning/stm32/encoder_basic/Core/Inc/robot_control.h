/*
 * robot_control.h
 *
 *  Created on: 2026. 10. 21.
 *      Author: heosi
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_

#include "motor_control.h"

#define WHEEL_TRACK_MM 230.0f

typedef struct {
	float delta_s_mm;
	float delta_yaw_rad;

	float x_mm;
	float y_mm;
	float yaw_rad;
} RobotState;

typedef struct {
	Motor *left_motor;
	Motor *right_motor;

	float wheel_track_mm;

	RobotState state;
} Robot;

void Robot_SetVelocity(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s
);

void Robot_Update(
	Robot *robot,
	uint32_t sample_dt_ms
);

#endif /* INC_ROBOT_CONTROL_H_ */
