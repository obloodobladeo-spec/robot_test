/*
 * imu_app.c
 *
 * GY-BNO085 I2C 연결 계층 (board_practice 로봇용).
 * uart_project/Core/Src/imu_app.c(실물 수신 확인됨)를 바탕으로 다음을 바꿨다.
 *  - USART2는 ROS2 통신 전용이므로 콘솔 출력을 모두 제거했다.
 *  - 지상 로봇·모터 자기장 영향을 피하려고 Game Rotation Vector(자기계 미사용)와
 *    보정 자이로만 50 Hz로 요청한다.
 *  - 주행 중에는 수백 ms 대기하는 재연결을 하지 않는다(allow_reconnect).
 */

#include "imu_app.h"
#include "main.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include <string.h>
#include <math.h>

#define IMU_I2C_TIMEOUT_MS     50U    /* 100 kHz에서 최대 패킷도 충분히 읽는 시간 */
#define IMU_REPORT_INTERVAL_US 20000U /* 50 Hz */
#define IMU_RETRY_PERIOD_MS    3000U
#define IMU_STALE_MS           1000U  /* 이 시간 동안 보고서가 없으면 끊김으로 판단 */
/* yaw 회전량 배율 보정. 2026-11-22 반시계 360° x 5회를 바닥 테이프로 실측:
 * 실제 / IMU = 1.0053 ± 0.0020 (IMU가 회전을 약 0.5% 작게 잼).
 * BNO085는 자이로 게인 자체 보정 명령이 공식 매뉴얼(SH-2 v1.2)에 없어 여기서 보정한다.
 * yaw 변화량과 gz에만 적용하고 roll/pitch는 센서 값 그대로 둔다. */
#define IMU_YAW_SCALE          1.0053f

volatile ImuState g_imu;

static I2C_HandleTypeDef *bus;
static uint8_t opened;
static uint8_t reset_pending;
static uint32_t last_attempt;
static uint32_t last_data;
static uint8_t rx_packet[SH2_HAL_MAX_TRANSFER_IN];
static uint8_t yaw_scale_ready;  /* 0이면 다음 보고서에서 보정 yaw를 센서 yaw로 초기화 */
static float raw_yaw_last;
static float scaled_yaw;

static float wrap_pi_f(float a)
{
	while (a > 3.14159265f) {
		a -= 6.28318531f;
	}
	while (a < -3.14159265f) {
		a += 6.28318531f;
	}
	return a;
}

/* ---------- SH-2 HAL (I2C 전송) ---------- */

static void reset_sensor(void)
{
	HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_SET);
}

static uint32_t hal_time_us(sh2_Hal_t *self)
{
	(void)self;
	/* 1 ms 분해능. SH-2 내부 timeout용이며 ROS와 동기화된 측정 시각이 아니다. */
	return HAL_GetTick() * 1000U;
}

static int hal_open(sh2_Hal_t *self)
{
	(void)self;
	reset_sensor();
	HAL_Delay(100);
	return SH2_OK;
}

static void hal_close(sh2_Hal_t *self)
{
	(void)self;
	HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, GPIO_PIN_RESET);
}

static int hal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned capacity, uint32_t *stamp)
{
	(void)self;
	uint8_t header[4];

	/* INT가 High이면 읽을 데이터가 없다. 대기하지 않고 바로 돌아간다. */
	if (HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) != GPIO_PIN_RESET) {
		return 0;
	}

	*stamp = hal_time_us(NULL);
	const uint16_t address = (uint16_t)(g_imu.address << 1);

	/* 1) 4바이트 SHTP 헤더로 전체 길이를 알아낸다. */
	if (HAL_I2C_Master_Receive(bus, address, header, 4U, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
		g_imu.i2c_errors++;
		return 0;
	}

	/* 길이 = 하위 15비트 (bit15는 continuation 비트) */
	const unsigned length = ((unsigned)header[0] | ((unsigned)header[1] << 8)) & 0x7FFFU;

	if (length == 0U || length == 0x7FFFU) {
		return 0;
	}
	if (length < 4U || length > sizeof(rx_packet) || length > capacity) {
		g_imu.i2c_errors++;
		return 0;
	}

	/* 2) 새 I2C 읽기는 헤더부터 다시 보내므로 헤더+payload 전체를 읽는다.
	 *    참고: CEVA sh2-demo-nucleo app/i2c_hal.c */
	if (HAL_I2C_Master_Receive(bus, address, rx_packet, (uint16_t)length,
	                           IMU_I2C_TIMEOUT_MS) != HAL_OK) {
		g_imu.i2c_errors++;
		return 0;
	}

	memcpy(buffer, rx_packet, length);
	return (int)length;
}

static int hal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned length)
{
	(void)self;

	if (length > SH2_HAL_MAX_TRANSFER_OUT) {
		return SH2_ERR_BAD_PARAM;
	}
	if (HAL_I2C_Master_Transmit(bus, (uint16_t)(g_imu.address << 1), buffer,
	                            (uint16_t)length, IMU_I2C_TIMEOUT_MS) != HAL_OK) {
		g_imu.i2c_errors++;
		return SH2_ERR_IO;
	}
	return (int)length;
}

static sh2_Hal_t transport = {
	.open = hal_open,
	.close = hal_close,
	.read = hal_read,
	.write = hal_write,
	.getTimeUs = hal_time_us,
};

/* ---------- SH-2 콜백 ---------- */

static void event_callback(void *cookie, sh2_AsyncEvent_t *event)
{
	(void)cookie;
	if (event->eventId == SH2_RESET) {
		reset_pending = 1U;  /* 센서가 스스로 리셋되면 보고서 설정이 사라진다. */
	}
}

static void sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
	(void)cookie;
	sh2_SensorValue_t value;

	if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
		g_imu.decode_errors++;
		return;
	}

	const uint32_t now = HAL_GetTick();

	switch (value.sensorId) {
	case SH2_GAME_ROTATION_VECTOR: {
		const float x = value.un.gameRotationVector.i;
		const float y = value.un.gameRotationVector.j;
		const float z = value.un.gameRotationVector.k;
		const float w = value.un.gameRotationVector.real;

		const float raw_yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

		/* yaw 변화량에 배율을 곱해 누적한다. */
		if (yaw_scale_ready == 0U) {
			scaled_yaw = raw_yaw;
			yaw_scale_ready = 1U;
		}
		else {
			scaled_yaw = wrap_pi_f(scaled_yaw + IMU_YAW_SCALE * wrap_pi_f(raw_yaw - raw_yaw_last));
		}
		raw_yaw_last = raw_yaw;

		/* 쿼터니언에 world Z축 회전(보정 yaw - 센서 yaw)을 앞에서 곱해 yaw만 바꾼다. */
		const float half = 0.5f * wrap_pi_f(scaled_yaw - raw_yaw);
		const float c = cosf(half);
		const float s = sinf(half);
		g_imu.qw = c * w - s * z;
		g_imu.qx = c * x - s * y;
		g_imu.qy = c * y + s * x;
		g_imu.qz = c * z + s * w;
		g_imu.yaw_rad = scaled_yaw;
		g_imu.rotation_status = value.status & 3U;
		g_imu.rotation_ms = now;
		g_imu.rotation_count++;
		break;
	}
	case SH2_GYROSCOPE_CALIBRATED:
		g_imu.gx = value.un.gyroscope.x;
		g_imu.gy = value.un.gyroscope.y;
		g_imu.gz = IMU_YAW_SCALE * value.un.gyroscope.z;
		g_imu.gyro_status = value.status & 3U;
		g_imu.gyro_ms = now;
		g_imu.gyro_count++;
		break;
	default:
		return;
	}

	last_data = now;
}

/* ---------- 연결 관리 ---------- */

static int configure_reports(void)
{
	const sh2_SensorId_t reports[] = {
		SH2_GAME_ROTATION_VECTOR,
		SH2_GYROSCOPE_CALIBRATED,
	};
	sh2_SensorConfig_t config;

	memset(&config, 0, sizeof(config));
	config.reportInterval_us = IMU_REPORT_INTERVAL_US;

	for (unsigned i = 0; i < sizeof(reports) / sizeof(reports[0]); ++i) {
		const int result = sh2_setSensorConfig(reports[i], &config);
		if (result != SH2_OK) {
			return result;
		}
	}

	last_data = HAL_GetTick();
	return SH2_OK;
}

static void disconnect_sensor(int reason)
{
	g_imu.online = 0U;
	g_imu.last_result = reason;

	if (opened) {
		sh2_close();
		opened = 0U;
	}

	reset_pending = 0U;
	last_attempt = HAL_GetTick();
}

static void connect_sensor(void)
{
	last_attempt = HAL_GetTick();
	g_imu.reconnects++;
	yaw_scale_ready = 0U;

	reset_sensor();
	HAL_Delay(150);

	/* AD0 Low = 0x4A, High = 0x4B. 두 주소를 모두 확인한다. */
	g_imu.address = 0U;
	for (uint8_t address = 0x4AU; address <= 0x4BU; ++address) {
		if (HAL_I2C_IsDeviceReady(bus, (uint16_t)(address << 1), 2U, 30U) == HAL_OK) {
			g_imu.address = address;
			break;
		}
	}
	if (g_imu.address == 0U) {
		g_imu.last_result = SH2_ERR_IO;
		return;
	}

	int result = sh2_open(&transport, event_callback, NULL);
	opened = 1U;
	if (result != SH2_OK) {
		disconnect_sensor(result);
		return;
	}

	sh2_ProductIds_t ids;
	memset(&ids, 0, sizeof(ids));
	result = sh2_getProdIds(&ids);
	if (result != SH2_OK || ids.numEntries == 0U) {
		disconnect_sensor(result != SH2_OK ? result : SH2_ERR);
		return;
	}
	g_imu.product_part = ids.entry[0].swPartNumber;
	g_imu.fw_major = ids.entry[0].swVersionMajor;
	g_imu.fw_minor = ids.entry[0].swVersionMinor;
	g_imu.fw_patch = ids.entry[0].swVersionPatch;

	result = sh2_setSensorCallback(sensor_callback, NULL);
	if (result == SH2_OK) {
		result = configure_reports();
	}
	if (result != SH2_OK) {
		disconnect_sensor(result);
		return;
	}

	reset_pending = 0U;
	g_imu.online = 1U;
	g_imu.last_result = SH2_OK;
}

/* ---------- 공개 함수 ---------- */

void ImuApp_Init(I2C_HandleTypeDef *i2c)
{
	bus = i2c;
	connect_sensor();
}

void ImuApp_Process(uint8_t allow_reconnect)
{
	uint32_t now = HAL_GetTick();

	g_imu.int_level = (uint8_t)HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin);

	if (g_imu.online == 0U) {
		if (allow_reconnect != 0U &&
		    (uint32_t)(now - last_attempt) >= IMU_RETRY_PERIOD_MS) {
			connect_sensor();
		}
		return;
	}

	/* INT가 Low일 때만 실제 I2C 읽기가 일어난다. */
	sh2_service();

	if (reset_pending != 0U) {
		reset_pending = 0U;
		const int result = configure_reports();
		if (result != SH2_OK) {
			disconnect_sensor(result);
			return;
		}
	}

	now = HAL_GetTick();
	if ((uint32_t)(now - last_data) > IMU_STALE_MS) {
		disconnect_sensor(SH2_ERR_TIMEOUT);
	}
}

uint8_t ImuApp_IsFresh(uint32_t max_age_ms)
{
	if (g_imu.online == 0U || g_imu.rotation_count == 0U || g_imu.gyro_count == 0U) {
		return 0U;
	}

	const uint32_t now = HAL_GetTick();

	return ((uint32_t)(now - g_imu.rotation_ms) <= max_age_ms &&
	        (uint32_t)(now - g_imu.gyro_ms) <= max_age_ms) ? 1U : 0U;
}
