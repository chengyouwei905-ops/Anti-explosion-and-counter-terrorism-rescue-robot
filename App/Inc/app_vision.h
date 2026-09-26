/**
  ******************************************************************************
  * @file    app_vision.h
  * @brief   视觉链路应用层：串口收发的统一入口
  * @note    层次关系
  *            bsp_uart      （字节流 + 环形缓冲）
  *              -> proto_vision（数据包：帧头/长度/CRC）
  *                -> app_vision （搬运 + 组包，业务只需注册回调）   <-- 本文件
  *
  *          本层不做任何业务解释：payload 里装的是什么，由调用方自己决定。
  ******************************************************************************
  */

#ifndef __APP_VISION_H
#define __APP_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "common_def.h"
#include "proto_vision.h"

/** @brief 一次从环形缓冲搬运的最大字节数 */
#define APP_VISION_CHUNK_SIZE       64U

/** @brief 发送超时时间（ms） */
#define APP_VISION_TX_TIMEOUT_MS    50U

/**
  * @brief  初始化视觉链路（绑定串口 -> 复位协议状态机 -> 启动中断接收）
  * @param  huart 串口句柄（本工程为 &huart7）
  * @retval ret_code_t
  */
int  app_vision_init(UART_HandleTypeDef *huart);

/** @brief 主循环周期调用：搬运串口数据并驱动协议解析（不调用会丢数据） */
void app_vision_poll(void);

/**
  * @brief  注册"收到一整帧"的回调
  * @param  cb 回调；传 NULL 取消注册
  * @note   回调在主循环上下文执行（app_vision_poll() 调用栈内），可做业务处理
  */
void app_vision_set_rx_cb(proto_vision_frame_cb_t cb);

/**
  * @brief  发送一帧（自动填充帧头 / 序号 / 长度 / CRC）
  * @param  cmd     命令字
  * @param  payload 载荷，len 为 0 时可传 NULL
  * @param  len     载荷字节数（<= PROTO_VISION_MAX_PAYLOAD）
  * @retval ret_code_t
  */
int  app_vision_send(uint8_t cmd, const uint8_t *payload, uint8_t len);

/** @brief 协议层统计（收字节数 / 正确帧数 / CRC 错误数） */
const proto_vision_stat_t *app_vision_get_stat(void);

/** @brief 串口接收缓冲溢出（丢字节）次数，正常应为 0 */
uint32_t app_vision_get_rx_overflow(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_VISION_H */
