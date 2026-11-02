#include "app_imu_uart_debug.h"
#include "app_imu.h"
#include "bsp_uart.h"
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#define IMU_UART_DEBUG_PERIOD_MS 1000U

static bool s_debug_ready;
static uint32_t s_debug_start_ms;
static uint32_t s_debug_last_ms;

int app_imu_uart_debug_init(UART_HandleTypeDef *huart, uint32_t now_ms)
{
    static const uint8_t header[] = "time_ms,yaw_rad,temp_c\r\n";
    int ret;

    s_debug_ready = false;
    ret = bsp_uart_attach(BSP_UART_ROBOARM, huart);
    if (ret != RET_OK) {
        return ret;
    }
    ret = bsp_uart_send_it(BSP_UART_ROBOARM, header, (uint16_t)(sizeof(header) - 1U));
    if (ret != RET_OK) {
        return ret;
    }

    s_debug_start_ms = now_ms;
    s_debug_last_ms = now_ms;
    s_debug_ready = true;
    return RET_OK;
}

int app_imu_uart_debug_poll(uint32_t now_ms)
{
    float euler[3];
    float temp_c;
    char line[BSP_UART_TX_BUFSIZE];
    int length;

    if (!s_debug_ready) {
        return RET_NOTREADY;
    }
    /* 无符号减法自然处理 HAL_GetTick() 回绕；忙时不在主循环里连续重试。 */
    if ((uint32_t)(now_ms - s_debug_last_ms) < IMU_UART_DEBUG_PERIOD_MS) {
        return RET_IDLE;
    }
    s_debug_last_ms = now_ms;

    if (!app_imu_get_euler(euler)) {
        return RET_NOTREADY;
    }
    temp_c = app_imu_get_temp();
    if (!isfinite(euler[AHRS_YAW]) || !isfinite(temp_c)) {
        return RET_ERROR;
    }

    /* %f 将浮点数转换为可读十进制文本；len 是字节数，不是 float 个数。
     * newlib-nano 的浮点格式化由 CMake 中 -u _printf_float 启用。
     * yaw 保留 9 位小数（rad），温度 3 位（℃）；结尾 CRLF 便于保存 CSV。 */
    length = snprintf(line, sizeof(line), "%lu,%.9f,%.3f\r\n",
                      (unsigned long)(uint32_t)(now_ms - s_debug_start_ms),
                      (double)euler[AHRS_YAW], (double)temp_c);
    if (length <= 0 || (size_t)length >= sizeof(line)) {
        return RET_PARAM;
    }

    /* BSP 会复制 line，故函数返回后局部缓冲区失效不影响中断发送。 */
    return bsp_uart_send_it(BSP_UART_ROBOARM, (const uint8_t *)line, (uint16_t)length);
}
