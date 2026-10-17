#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

#define MOTOR_COUNT 2U
#define MOTOR_MAX_SPEED_MM_S 200
#define MOTOR_MAX_DUTY_PERCENT 50.0f

typedef struct {
    float target_mm_s;
    float speed_mm_s;
    float rpm;
    float duty_percent;
    float i_output;
    float kp;
    float ki;
    int32_t delta_count;
    uint32_t raw_count;
    int8_t direction;
    uint8_t stop_samples;
} MotorState;

extern MotorState motors[MOTOR_COUNT];
HAL_StatusTypeDef Motor_Init(void);
void Motor_SetTargets(int16_t motor1_mm_s, int16_t motor2_mm_s);
void Motor_Update(uint32_t now);
void Motor_StopAll(void);
void Motor_EmergencyStop(void);

#endif
