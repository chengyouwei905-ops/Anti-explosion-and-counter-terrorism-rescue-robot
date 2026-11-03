#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "app_imu_uart_debug.h"
#include "bsp_uart.h"

static double yaw;
static double temperature;
static bool imu_valid = true;
static unsigned it_sends;
static HAL_StatusTypeDef it_result = HAL_OK;

bool app_imu_get_euler(double euler[3])
{
    euler[0] = 0.0;
    euler[1] = 0.0;
    euler[2] = yaw;
    return imu_valid;
}

double app_imu_get_temp(void) { return temperature; }

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data,
                                    uint16_t length, uint32_t timeout)
{
    (void)uart; (void)data; (void)length; (void)timeout;
    assert(!"debug logger must not use blocking UART transmit");
    return HAL_ERROR;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data,
                                       uint16_t length)
{
    if (it_result != HAL_OK) { return it_result; }
    uart->test_tx_ptr = data;
    uart->test_tx_len = length;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    ++it_sends;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data,
                                      uint16_t length)
{
    (void)uart; (void)data; (void)length;
    return HAL_OK;
}

static void expect_pending(const UART_HandleTypeDef *uart, const char *line)
{
    /* The real BSP must retain a copy after the formatter's local buffer expires. */
    assert(uart->test_tx_len == strlen(line));
    assert(memcmp(uart->test_tx_ptr, line, strlen(line)) == 0);
}

int main(void)
{
    UART_HandleTypeDef uart = { .gState = HAL_UART_STATE_READY };
    assert(app_imu_uart_debug_poll(0U) == RET_NOTREADY);
    assert(app_imu_uart_debug_init(NULL, 0U) == RET_PARAM);
    assert(app_imu_uart_debug_init(&uart, 100U) == RET_OK);
    expect_pending(&uart, "time_ms,yaw_rad,temp_c\r\n");
    uart.gState = HAL_UART_STATE_READY;

    yaw = -0.00125;
    temperature = 30.125;
    assert(app_imu_uart_debug_poll(1099U) == RET_IDLE);
    assert(app_imu_uart_debug_poll(1100U) == RET_OK);
    expect_pending(&uart, "1000,-0.001250000,30.125\r\n");
    assert(app_imu_uart_debug_poll(1101U) == RET_IDLE);
    assert(it_sends == 2U);

    /* Busy skips one report without overwriting the in-flight bytes or retry spinning. */
    yaw = 12.5;
    temperature = -5.125;
    assert(app_imu_uart_debug_poll(2100U) == RET_BUSY);
    expect_pending(&uart, "1000,-0.001250000,30.125\r\n");
    uart.gState = HAL_UART_STATE_READY;
    assert(app_imu_uart_debug_poll(2101U) == RET_IDLE);
    assert(app_imu_uart_debug_poll(3100U) == RET_OK);
    expect_pending(&uart, "3000,12.500000000,-5.125\r\n");
    assert(it_sends == 3U);

    uart.gState = HAL_UART_STATE_READY;
    imu_valid = false;
    assert(app_imu_uart_debug_poll(4100U) == RET_NOTREADY);
    imu_valid = true;
    yaw = NAN;
    assert(app_imu_uart_debug_poll(5100U) == RET_ERROR);
    yaw = 0.0;
    temperature = INFINITY;
    assert(app_imu_uart_debug_poll(6100U) == RET_ERROR);
    assert(it_sends == 3U);

    /* A delayed main loop emits one current report, with no burst of catch-up rows. */
    temperature = 25.0;
    assert(app_imu_uart_debug_poll(10100U) == RET_OK);
    expect_pending(&uart, "10000,0.000000000,25.000\r\n");
    assert(app_imu_uart_debug_poll(10101U) == RET_IDLE);
    uart.gState = HAL_UART_STATE_READY;

    /* Both cadence and relative timestamps remain correct across tick wraparound. */
    uint32_t start = UINT32_MAX - 400U;
    assert(app_imu_uart_debug_init(&uart, start) == RET_OK);
    uart.gState = HAL_UART_STATE_READY;
    assert(app_imu_uart_debug_poll(start + 999U) == RET_IDLE);
    assert(app_imu_uart_debug_poll(start + 1000U) == RET_OK);
    expect_pending(&uart, "1000,0.000000000,25.000\r\n");

    uart.gState = HAL_UART_STATE_READY;
    it_result = HAL_ERROR;
    assert(app_imu_uart_debug_init(&uart, 0U) == RET_ERROR);
    assert(app_imu_uart_debug_poll(1000U) == RET_NOTREADY);
    it_result = HAL_OK;
    assert(app_imu_uart_debug_init(&uart, 0U) == RET_OK);
    uart.gState = HAL_UART_STATE_READY;
    it_result = HAL_ERROR;
    assert(app_imu_uart_debug_poll(1000U) == RET_ERROR);
    it_result = HAL_OK;
    assert(app_imu_uart_debug_poll(1001U) == RET_IDLE);
    assert(app_imu_uart_debug_poll(2000U) == RET_OK);
    expect_pending(&uart, "2000,0.000000000,25.000\r\n");

    puts("PASS: IMU CSV formatting, nonblocking UART, timing, busy/failure recovery and tick rollover");
    return 0;
}
