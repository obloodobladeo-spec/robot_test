/*
 * imu_control.c
 *
 *  Created on: 2026. 11. 12.
 *      Author: kim
 */

#include "imu_control.h"
#include "main.h"

void IMU_RESET(void)
{
	HAL_GPIO_WritePin(
        IMU_RST_GPIO_Port,
        IMU_RST_Pin,
        GPIO_PIN_RESET
    );

    HAL_Delay(10);

    HAL_GPIO_WritePin(
        IMU_RST_GPIO_Port,
        IMU_RST_Pin,
        GPIO_PIN_SET
    );

    HAL_Delay(150);
}

HAL_StatusTypeDef IMU_CheckConnection(I2C_HandleTypeDef *hi2c)
{
	return HAL_I2C_IsDeviceReady(hi2c, (0x4A << 1), 2, 20);
}


uint8_t IMU_DataReady(void)
{
	return HAL_GPIO_ReadPin(
		IMU_INT_GPIO_Port,
		IMU_INT_Pin
	) == GPIO_PIN_RESET;
}

HAL_StatusTypeDef IMU_ReadHeader(
    I2C_HandleTypeDef *hi2c,
    uint8_t *header
)
{
	if (IMU_DataReady() == 0){
		return HAL_BUSY;
	}

	return HAL_I2C_Master_Receive(
		hi2c,
		(0x4A <<1),
		header,
		4,
		20
	);
}
