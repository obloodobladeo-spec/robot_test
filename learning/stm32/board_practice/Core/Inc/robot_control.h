/*
 * robot_control.h
 *
 *  Created on: 2026. 10. 21.
 *      Author: heosi
 */

#ifndef INC_ROBOT_CONTROL_H_
#define INC_ROBOT_CONTROL_H_

#include "motor_control.h"

/* 실효 윤거. 실측 바퀴 중심 간격은 234 mm이지만, 제자리 360° 회전 5회(2026-11-21)에서
 * 엔코더 yaw / IMU yaw = 1.056 (방향별 차이 없음)이 나와 234 x 1.056 = 247 mm로 보정했다.
 * 2026-11-22 캐스터 2개 -> 앞 1개로 변경 후 재측정 5회: 247 x 0.994 = 245.5 mm (표준편차 0.5).
 * 2026-11-22 IMU yaw 배율(1.0053)을 바닥 테이프 실측으로 보정한 뒤, 볼 캐스터 1개·오른쪽 바퀴
 * 둘레 330.7 상태에서 재측정 5회: 243.1 mm (표준편차 0.6, 방향별 차이 없음). 실측 회전 기준 값.
 * 회전 시 바퀴 옆 미끄러짐·캐스터 저항으로 실제 회전이 기하학 계산보다 작기 때문이다.
 * ROS cmd_vel_receiver의 wheel_track_m과 같은 값을 유지한다. */
#define WHEEL_TRACK_MM 243.1f

/* 직진 방향 유지(heading hold): W=0, V!=0 명령일 때만 동작한다.
 * 보정 각속도 = KP * 엔코더 yaw 오차 + KI * IMU yaw 오차 적분 (IMU 없으면 엔코더 오차 적분).
 *  - P(엔코더): 지연 없이 좌우 바퀴 출발 차이·순간 편차를 바로 잡는다. 어느 쪽이 빠른지 가정하지 않는다.
 *  - I(IMU): 바퀴 둘레 차·미끄러짐처럼 엔코더로 안 보이는 실제 방향 차이를 천천히 없앤다.
 * 최대 HEADING_MAX_CORR_RAD_S로 제한. IMU yaw 부호가 반대면 HEADING_IMU_SIGN을 -1로 바꾼다. */
#define HEADING_KP              3.0f   /* (rad/s) / rad, 엔코더 오차 */
#define HEADING_KI              1.0f   /* (rad/s) / (rad*s), IMU 오차 적분 */
/* IMU와 엔코더 yaw 오차의 차이(엔코더가 못 보는 실제 틀어짐)에 거는 P 게인. 0이면 P는 엔코더만. */
#define HEADING_KP_IMU          2.0f   /* (rad/s) / rad */
#define HEADING_MAX_CORR_RAD_S  0.15f
#define HEADING_IMU_SIGN        1.0f

typedef struct {
	float delta_s_mm;
	float delta_yaw_rad;

	float x_mm;
	float y_mm;
	float yaw_rad;
} RobotState;

typedef struct {
	uint8_t active;      /* 1 = 직진 방향 유지 중 */
	uint8_t use_imu;     /* 1 = 적분에 IMU yaw 사용, 0 = 엔코더 yaw 사용 */
	int8_t direction;    /* 직진 방향 부호, 바뀌면 기준을 다시 잡는다 */
	float reference_rad; /* 직진 시작 시 엔코더 yaw */
	float imu_reference_rad; /* 직진 시작(또는 IMU 복귀) 시 IMU yaw */
	float error_rad;     /* 엔코더 yaw 오차 */
	float slow_error_rad; /* 적분에 쓰는 오차 (IMU 또는 엔코더) */
	float integral;      /* rad*s */
	float correction_rad_s;
} HeadingHold;

/* heading hold 게인 (RAM 설정값: 실행 중 SWD로 바꿔 가며 시험할 수 있다. 기본값은 위 HEADING_* 상수) */
typedef struct {
	float kp_encoder;       /* (rad/s)/rad, 엔코더 yaw 오차 */
	float kp_imu;           /* (rad/s)/rad, (IMU - 엔코더) 오차 */
	float ki;               /* (rad/s)/(rad*s), IMU 오차 적분 */
	float max_corr_rad_s;
} HeadingConfig;

typedef struct {
	Motor *left_motor;
	Motor *right_motor;

	float wheel_track_mm;
	HeadingConfig heading;

	RobotState state;
	HeadingHold hold;
} Robot;

void Robot_SetVelocity(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s
);

/* 직진 명령이면 방향 오차를 보정한 각속도를, 아니면 angular 그대로를 반환한다.
 * P는 엔코더 yaw, I는 imu_valid != 0이면 imu_yaw_rad, 아니면 엔코더 yaw 오차를 쓴다. */
float Robot_HeadingHold(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s,
	uint8_t imu_valid,
	float imu_yaw_rad,
	uint32_t sample_dt_ms
);

/* 피드포워드 적용 여부를 정한다. 방향 유지 보정 전의 원래 명령을 넘긴다. */
void Robot_SetFeedforward(
	Robot *robot,
	float linear_speed_mm_s,
	float angular_speed_rad_s
);

void Robot_Update(
	Robot *robot,
	uint32_t sample_dt_ms
);

#endif /* INC_ROBOT_CONTROL_H_ */
