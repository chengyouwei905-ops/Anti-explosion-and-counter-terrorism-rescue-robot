/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   串口（UART）板级封装：与"协议/业务"完全解耦，只负责收发字节流
  * @note    设计要点
  *          1. 上层只认逻辑编号 bsp_uart_id_t，不直接碰 UART_HandleTypeDef；
  *          2. 接收采用"单字节中断 + 环形缓冲区"，中断里只做搬运，解析放主循环；
  *          3. 自动重装接收，发生 ORE/FE/NE/PE 错误后能自恢复，不会永久断流。
  ******************************************************************************
  */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "ring_buffer.h"

/** @brief 逻辑串口编号（新增串口时在这里加一项，并在 bsp_uart.c 里 attach） */
typedef enum {
    BSP_UART_VISION = 0,    /**< UART7：PE8=TX / PE7=RX，视觉链路 */
    BSP_UART_CHASSIS,       /**< UART1：PA9=TX / PA10=RX，底盘链路 */          
    BSP_UART_ID_MAX
} bsp_uart_id_t;

/** @brief 每个串口接收环形缓冲区容量，必须为 2 的幂 */
#define BSP_UART_RX_BUFSIZE     512U

/** @brief 收到 1 字节的回调（中断上下文，禁止 printf/延时等耗时操作） */
typedef void (*bsp_uart_rx_cb_t)(bsp_uart_id_t id, uint8_t byte);

/**
  * @brief  绑定逻辑串口与 HAL 句柄（HAL_UART_Init 之后调用）
  * @param  id    逻辑串口编号
  * @param  huart HAL 串口句柄，如 &huart7
  * @retval ret_code_t
  */
int      bsp_uart_attach(bsp_uart_id_t id, UART_HandleTypeDef *huart);

/**
  * @brief  启动中断接收（需先在 CubeMX 中使能该串口的 global interrupt）
  * @retval ret_code_t
  */
int      bsp_uart_start_rx(bsp_uart_id_t id);

/** @brief 注册收到字节的回调，传 NULL 表示取消注册 */
void     bsp_uart_set_rx_cb(bsp_uart_id_t id, bsp_uart_rx_cb_t cb);

/**
  * @brief  阻塞发送
  * @param  timeout_ms 超时时间（ms）
  * @retval ret_code_t
  */
int      bsp_uart_send(bsp_uart_id_t id, const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/** @brief 从接收缓冲区取出数据，返回实际取出字节数 */
uint16_t bsp_uart_read(bsp_uart_id_t id, uint8_t *data, uint16_t len);

/** @brief 接收缓冲区中待处理字节数 */
uint16_t bsp_uart_available(bsp_uart_id_t id);

/** @brief 丢弃接收缓冲区中所有未处理数据 */
void     bsp_uart_flush_rx(bsp_uart_id_t id);

/** @brief 累计接收缓冲溢出（丢字节）次数，用于排查"解析不上"的问题 */
uint32_t bsp_uart_get_rx_overflow(bsp_uart_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
