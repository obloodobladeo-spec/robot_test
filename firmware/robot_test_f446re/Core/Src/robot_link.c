#include "robot_link.h"
#include "motor_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RX_SIZE 256U
#define COMMAND_TIMEOUT_MS 500U
static UART_HandleTypeDef uart;
static uint8_t rx_byte;
/* UART ISR와 main이 공유하므로 이 인덱스와 오류 플래그에만 volatile 사용. */
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_error;
static uint8_t rx_ring[RX_SIZE];
static char line[96], tx_line[96];
static uint16_t line_size;
static uint8_t discard_line;
static uint32_t last_command, last_status;
static uint16_t last_sequence;
static uint8_t have_command;

static uint16_t crc16(const char *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)(uint8_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; ++b)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint8_t parse_command(uint32_t now)
{
    char *star = strchr(line, '*');
    if (star == NULL || strlen(star + 1) != 4U || line[0] != 'V' || line[1] != ',') return 0;
    char *end;
    const unsigned long received_crc = strtoul(star + 1, &end, 16);
    if (*end != '\0' || received_crc > 65535U || crc16(line, (size_t)(star - line)) != received_crc) return 0;
    *star = '\0';
    char *cursor = line + 2;
    const long sequence = strtol(cursor, &end, 10);
    if (end == cursor || *end != ',' || sequence < 0 || sequence > 65535) return 0;
    cursor = end + 1;
    const long m1 = strtol(cursor, &end, 10);
    if (end == cursor || *end != ',' || m1 < -200 || m1 > 200) return 0;
    cursor = end + 1;
    const long m2 = strtol(cursor, &end, 10);
    if (end == cursor || *end != '\0' || m2 < -200 || m2 > 200) return 0;
    Motor_SetTargets((int16_t)m1, (int16_t)m2);
    last_sequence = (uint16_t)sequence;
    last_command = now;
    have_command = 1;
    return 1;
}

HAL_StatusTypeDef RobotLink_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    uart.Instance = USART2;
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart) != HAL_OK) return HAL_ERROR;
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    return HAL_UART_Receive_IT(&uart, &rx_byte, 1);
}

void RobotLink_IrqHandler(void) { HAL_UART_IRQHandler(&uart); }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
    if (handle != &uart) return;
    /* ISR은 수신 바이트 저장과 다음 수신 요청만 수행합니다. */
    const uint16_t next = (uint16_t)((rx_head + 1U) % RX_SIZE);
    if (next == rx_tail) rx_error = 1;
    else {
        rx_ring[rx_head] = rx_byte;
        __DMB();
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(&uart, &rx_byte, 1) != HAL_OK) rx_error = 1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    if (handle == &uart) rx_error = 1;
}

void RobotLink_Task(uint32_t now)
{
    if (rx_error) {
        const uint32_t irq_state = __get_PRIMASK();
        __disable_irq();
        rx_tail = rx_head;
        rx_error = 0;
        __set_PRIMASK(irq_state);
        line_size = 0;
        discard_line = 1;
        have_command = 0;
        Motor_StopAll();
        HAL_UART_AbortReceive(&uart);
        if (HAL_UART_Receive_IT(&uart, &rx_byte, 1) != HAL_OK) rx_error = 1;
    }
    while (rx_tail != rx_head) {
        __DMB();
        const char c = (char)rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_SIZE);
        if (c == '\n') {
            if (!discard_line) {
                line[line_size] = '\0';
                (void)parse_command(now);
            }
            line_size = 0;
            discard_line = 0;
        } else if (c != '\r') {
            if ((uint8_t)c < 32U || (uint8_t)c > 126U || line_size >= sizeof(line) - 1U)
                discard_line = 1;
            if (!discard_line) line[line_size++] = c;
        }
    }
    const uint8_t timed_out = !have_command || (uint32_t)(now - last_command) >= COMMAND_TIMEOUT_MS;
    if (timed_out) Motor_StopAll();
    if ((uint32_t)(now - last_status) >= 100U && uart.gState == HAL_UART_STATE_READY) {
        last_status = now;
        const int n = snprintf(tx_line, sizeof(tx_line), "S,%u,%ld,%ld,%ld,%ld,%u",
            (unsigned)last_sequence, (long)motors[0].speed_mm_s, (long)motors[1].speed_mm_s,
            (long)(motors[0].duty_percent * 100.0f), (long)(motors[1].duty_percent * 100.0f),
            (unsigned)timed_out);
        if (n > 0 && (size_t)n + 6U < sizeof(tx_line)) {
            const uint16_t crc = crc16(tx_line, (size_t)n);
            const int suffix = snprintf(tx_line + n, sizeof(tx_line) - (size_t)n, "*%04X\n", crc);
            (void)HAL_UART_Transmit_IT(&uart, (uint8_t *)tx_line, (uint16_t)(n + suffix));
        }
    }
}
