#ifndef APP_IMU_UART_DEBUG_H
#define APP_IMU_UART_DEBUG_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "common_def.h"

/* 临时借用 USART10 采集 IMU：1=文本日志并暂停机械臂/路线控制，0=恢复原业务。 */
#ifndef APP_IMU_UART_DEBUG_ENABLE
#define APP_IMU_UART_DEBUG_ENABLE 0
#endif

/* 占用 BSP_UART_ROBOARM；USART10 已由 CubeMX 初始化为 9600、8N1。
 * 输出表头 time_ms,yaw_rad,temp_c，时间从本函数调用时开始计数。
 * 只启动中断发送，不等待数据发完；启动失败时可重新调用。 */
int app_imu_uart_debug_init(UART_HandleTypeDef *huart, uint32_t now_ms);

/* 放在主循环：每隔至少 1000ms 尝试发送一次最新 yaw/温度。
 * RET_IDLE=未到发送时间；忙/失败跳过本帧，一秒后再试，不阻塞 IMU 轮询。
 * 同一上下文串行调用，与 app_imu_uart_debug_init() 不可重入。 */
int app_imu_uart_debug_poll(uint32_t now_ms);

#endif
