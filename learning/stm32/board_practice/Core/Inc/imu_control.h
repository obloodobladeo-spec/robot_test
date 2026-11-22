/*
 * imu_control.h
 *
 *  Created on: 2026. 11. 12.
 *      Author: kim
 */

#ifndef INC_IMU_CONTROL_H_
#define INC_IMU_CONTROL_H_

#include "stm32f4xx_hal.h"

void IMU_RESET(void);
HAL_StatusTypeDef IMU_CheckConnection(I2C_HandleTypeDef *hi2c);
uint8_t IMU_DataReady(void);

HAL_StatusTypeDef IMU_ReadHeader(
		I2C_HandleTypeDef *hi2c,
		uint8_t *header
);

#endif /* INC_IMU_CONTROL_H_ */
