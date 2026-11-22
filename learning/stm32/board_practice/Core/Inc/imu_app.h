/*
 * imu_app.h
 *
 * GY-BNO085 (BNO085) I2C 연결 계층. SHTP 패킷 해석은 CEVA SH-2 라이브러리가 맡는다.
 * 배선: I2C1 SCL=PB8 / SDA=PB9 (100 kHz), INT=PC6(Low=데이터 준비), RST=PC5, 주소 0x4A(AD0 Low).
 *
 * 모든 값은 센서 자체 축 기준이며 로봇 축으로 변환하지 않는다.
 */

#ifndef INC_IMU_APP_H_
#define INC_IMU_APP_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
	uint8_t address;          /* ACK 응답한 7비트 주소, 0이면 미발견 */
	uint8_t online;           /* SH-2 열기 + 보고서 설정 성공 */
	uint8_t int_level;        /* PC6 현재 레벨 (0 = 데이터 준비) */
	uint8_t rotation_status;  /* Game Rotation Vector 정확도 상태 0~3 */
	uint8_t gyro_status;      /* 보정 자이로 정확도 상태 0~3 */
	int32_t last_result;      /* 마지막 SH-2 결과 코드 (sh2_err.h) */

	uint32_t product_part;    /* 제품 소프트웨어 part number */
	uint8_t fw_major, fw_minor;
	uint16_t fw_patch;

	uint32_t rotation_count, gyro_count;  /* 보고서 수신 누적 개수 */
	uint32_t rotation_ms, gyro_ms;        /* 마지막 수신 시각 (HAL_GetTick) */
	uint32_t i2c_errors, decode_errors, reconnects;

	float qx, qy, qz, qw;     /* Game Rotation Vector 쿼터니언 (자기계 미사용) */
	float yaw_rad;            /* 위 쿼터니언에서 계산한 yaw, -pi ~ pi */
	float gx, gy, gz;         /* 보정 각속도 rad/s */
} ImuState;

extern volatile ImuState g_imu;

/* 부팅 시 1회 호출. 센서 리셋·연결 때문에 수백 ms 동안 대기할 수 있다. */
void ImuApp_Init(I2C_HandleTypeDef *i2c);

/* while(1)에서 매번 호출. INT가 Low일 때만 I2C를 읽는다.
 * allow_reconnect = 1일 때만 끊긴 센서를 다시 연결한다(재연결은 수백 ms 대기하므로
 * 로봇이 정지 명령 상태일 때만 허용한다). */
void ImuApp_Process(uint8_t allow_reconnect);

/* 회전 벡터와 자이로가 모두 max_age_ms 이내에 수신됐으면 1. */
uint8_t ImuApp_IsFresh(uint32_t max_age_ms);

#endif /* INC_IMU_APP_H_ */
