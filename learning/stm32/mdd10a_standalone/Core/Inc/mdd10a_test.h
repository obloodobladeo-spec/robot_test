#ifndef MDD10A_TEST_H
#define MDD10A_TEST_H

#include "stm32f4xx_hal.h"

typedef enum {
    MDD10A_TEST_OK = 0,
    MDD10A_TEST_ALREADY_ATTEMPTED,
    MDD10A_TEST_BAD_CONFIG,
    MDD10A_TEST_START_FAILED
} Mdd10aTestResult;

/* Call once from main after CubeMX GPIO/TIM1 initialization, with IRQs enabled.
 * Owns TIM1 CH1/CH2, PA8/PA9 and PC0/PC1. Blocks for about 25 seconds.
 * A second call does nothing, even if the first attempt failed.
 */
Mdd10aTestResult MDD10A_TestRunOnce(TIM_HandleTypeDef *htim1);

#endif
