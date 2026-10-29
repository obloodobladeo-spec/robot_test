#ifndef MOTOR_SEQUENCE_H
#define MOTOR_SEQUENCE_H

#include "stm32f4xx_hal.h"

typedef enum {
    MOTOR_TEST_IDLE = 0,
    MOTOR_TEST_STARTUP_WAIT,
    MOTOR_TEST_M1_LOW_SETTLE,
    MOTOR_TEST_M1_LOW_RUN,
    MOTOR_TEST_STOP_AFTER_M1_LOW,
    MOTOR_TEST_M1_HIGH_SETTLE,
    MOTOR_TEST_M1_HIGH_RUN,
    MOTOR_TEST_STOP_AFTER_M1_HIGH,
    MOTOR_TEST_M2_LOW_SETTLE,
    MOTOR_TEST_M2_LOW_RUN,
    MOTOR_TEST_STOP_AFTER_M2_LOW,
    MOTOR_TEST_M2_HIGH_SETTLE,
    MOTOR_TEST_M2_HIGH_RUN,
    MOTOR_TEST_STOP_BEFORE_SPIN,
    MOTOR_TEST_SPIN_SETTLE,
    MOTOR_TEST_SPIN_RUN,
    MOTOR_TEST_DONE,
    MOTOR_TEST_FAILED
} MotorTestPhase;

/* 디버거에서 현재 시험 단계 확인. 애플리케이션에서는 값을 변경하지 않습니다. */
extern volatile MotorTestPhase motor_test_phase;

/* TIM1 초기화 후 main에서 한 번 호출. 실패하면 즉시 Error_Handler 호출 필요. */
HAL_StatusTypeDef MotorSequence_Init(TIM_HandleTypeDef *timer);
/* main 루프에서 계속 호출. HAL_Delay 없이 tick 경과에 따라 한 단계씩 진행. */
void MotorSequence_Task(uint32_t now);

#endif
