/**
  ******************************************************************************
  * @file    app_vision.h
  * @brief   视觉模块应用层：把协议帧翻译成"业务可用的目标位姿"
  * @note    层次关系
  *            bsp_uart  (字节流)
  *              -> proto_vision (帧)
  *                -> app_vision  (目标位姿 + 超时/丢帧判断)   <-- 本文件
  *        底盘/任务状态机只调用 app_vision_get_target()，不关心串口和协议细节。
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

/** @brief 超过该时间未收到有效帧则认为视觉数据失效（ms） */
#define VISION_TIMEOUT_MS           200U

/** @brief PROTO_VISION_CMD_TARGET 的载荷布局（小端）：
  *        [0..1] : int16  x_mm      目标相对 X（mm）
  *        [2..3] : int16  y_mm      目标相对 Y（mm）
  *        [4..5] : int16  yaw_cdeg  目标偏航角（0.01°）
  *        [6]    : uint8  detect    检测有效标志
  *        需与视觉端约定一致
  */
#define VISION_TARGET_PAYLOAD_LEN   7U

/** @brief 视觉目标（业务视角的数据结构） */
typedef struct {
    bool     valid;         /**< 数据是否可用；超时未更新会自动置 false */
    uint16_t seq;           /**< 最近一帧的帧序号 */
    int16_t  x_mm;          /**< 目标相对 X（mm） */
    int16_t  y_mm;          /**< 目标相对 Y（mm） */
    int16_t  yaw_cdeg;      /**< 目标偏航角（0.01°） */
    uint8_t  detect;        /**< 视觉检测有效标志 */
    uint32_t rx_tick;       /**< 最近一次有效帧到达时刻（HAL_GetTick） */
    uint32_t lost_frames;   /**< 累计丢帧数（按 SEQ 跳变统计） */
} vision_target_t;

/**
  * @brief  初始化视觉通信链路（绑定串口 -> 启动中断接收 -> 注册协议回调）
  * @param  huart 视觉链路串口句柄（本工程为 &huart7）
  * @retval ret_code_t
  */
int  app_vision_init(UART_HandleTypeDef *huart);

/** @brief 主循环周期调用：搬运串口数据、驱动协议解析、做超时判断 */
void app_vision_poll(void);

/** @brief 获取视觉目标（只读，永不返回 NULL） */
const vision_target_t *app_vision_get_target(void);

/** @brief 获取协议层统计（CRC 错误 / 丢帧等，联调必备） */
const proto_vision_stat_t *app_vision_get_stat(void);

/** @brief 获取串口接收缓冲溢出次数 */
uint32_t app_vision_get_rx_overflow(void);

/**
  * @brief  调试用：把当前视觉目标格式化后发回串口
  * @note   仅在联调阶段使用；正式跑车时若该串口就是视觉链路，
  *         回发数据会干扰对方，请改成独立的调试串口或直接注掉。
  */
void app_vision_debug_echo(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __APP_VISION_H */
