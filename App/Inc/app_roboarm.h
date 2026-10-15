/**
  ******************************************************************************
  * @file    app_roboarm.h
  * @brief   机械臂（总线舵机控制板）应用层：组帧 + 下发，目前只做"用户主动发送"
  * @note    层次关系
  *            bsp_uart        （字节流 + 环形缓冲）
  *              -> proto_roboarm（数据包：55 55 | Length | Cmd | Prm）
  *                -> app_roboarm （组帧 + 串口发送，业务只调下面的函数）  <-- 本文件
  *
  *          ⚠️ 控制板固定 **9600-8-N-1**：CubeMX 里 USART10 的波特率必须改成 9600
  *             （默认 115200），否则灯不闪、舵机不动，而且不报任何错。
  *
  *          目前只实现手册"一、用户主动给控制板发送数据部分"：
  *          每调一次函数 = 组一帧发出去，不缓存、不排队、不需要 poll()。
  *          控制板主动上报（动作组结束等）和读指令的回包还没解析 ——
  *          等真正要用时再加 app_roboarm_poll() + 回调。
  *
  *          ⚠️ 两个使用上的坑：
  *          1. `time_ms` 是"这条指令用多久走完"。周期下发（视觉/手柄每 20~50ms 一条）
  *             时它应该 ≈ 发送周期；要是每条都给 500ms，控制板每条都会重新开始插值，
  *             机械臂永远追不上目标，看起来又慢又抖。只有单次动作（归位、抓取）才给大时间。
  *          2. 发送是**阻塞**的（bsp_uart_send → HAL_UART_Transmit 忙等）：
  *             9600 下 3 关节 ≈17ms、8 关节 ≈32ms。别放进现在这个 1kHz 的
  *             IMU/底盘循环里（会把 app_imu_poll() 的 dt 拉大、姿态变差），
  *             放到 20~50ms 的慢速任务里发。
  ******************************************************************************
  */

#ifndef __APP_ROBOARM_H
#define __APP_ROBOARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "common_def.h"
#include "proto_roboarm.h"

/** @brief 发送超时时间（ms）。
  *        9600bps 下 1 个字节 ≈ 1.04ms，最长的一帧（8 舵机）31 字节 ≈ 32ms，
  *        给 100ms 留足余量。 */
#define APP_ROBOARM_TX_TIMEOUT_MS   100U

/**
  * @brief  初始化机械臂链路（绑定串口 -> 复位协议层）
  * @param  huart 串口句柄（本工程为 &huart10，波特率必须是 9600）
  * @retval ret_code_t
  * @note   只发不收，所以这里不启动中断接收；以后要收控制板上报的数据再调
  *         bsp_uart_start_rx(BSP_UART_ROBOARM)。
  *         ⚠️ 本函数**不会让机械臂做任何动作**：上电后控制板上的舵机是保持力矩
  *         僵在断电那一刻的姿势的。需要安全姿态或卸力，请在 init 之后显式调
  *         app_roboarm_servo_move() / app_roboarm_action_group_run() / app_roboarm_servo_unload()。
  */
int  app_roboarm_init(UART_HandleTypeDef *huart);

/**
  * @brief  控制任意个舵机在 time_ms 内转到目标位置
  * @param  servos  舵机数组（ID + 位置），一个元素 = 一个舵机
  * @param  count   舵机个数，1 ~ PROTO_ROBOARM_MAX_SERVO
  * @param  time_ms 转动时间（ms），0 = 立刻到位
  * @retval ret_code_t
  * @note   所有舵机共用同一个时间。例：1 号 1000ms 转到 800
  *             app_roboarm_servo_move(&(proto_roboarm_servo_t){1U, 800U}, 1U, 1000U);
  *         ⚠️ 机械臂是刚体，多关节同时动才是一段轨迹；分开一条条发会互相干涉。
  */
int  app_roboarm_servo_move(const proto_roboarm_servo_t *servos, uint8_t count,
                            uint16_t time_ms);

/**
  * @brief  运行已经下载到控制板里的动作组
  * @param  group 动作组编号
  * @param  times 运行次数，0 = 一直循环
  * @retval ret_code_t
  */
int  app_roboarm_action_group_run(uint8_t group, uint16_t times);

/** @brief 停止正在运行的动作组（本来没在跑也不受影响） */
int  app_roboarm_action_group_stop(void);

/**
  * @brief  调整动作组运行速度
  * @param  group   动作组编号，PROTO_ROBOARM_GROUP_ALL(0xFF) = 全部动作组
  * @param  percent 百分比，100 = 原速
  * @retval ret_code_t
  * @note   掉电不保存，每次上电都要重发。
  */
int  app_roboarm_action_group_speed(uint8_t group, uint16_t percent);

/**
  * @brief  多个舵机掉电卸力（卸力后可用手随意扳动）
  * @param  ids   舵机 ID 数组
  * @param  count 舵机个数，1 ~ PROTO_ROBOARM_MAX_SERVO
  * @retval ret_code_t
  */
int  app_roboarm_servo_unload(const uint8_t *ids, uint8_t count);

/**
  * @brief  请求控制板回传电池电压（mV）
  * @retval ret_code_t
  * @note   ⚠️ 只发请求，**回包还没解析**（App 真正要用时再加接收侧）。
  */
int  app_roboarm_read_battery_voltage(void);

/**
  * @brief  请求控制板回传这些舵机的角度位置
  * @param  ids   舵机 ID 数组
  * @param  count 舵机个数，1 ~ PROTO_ROBOARM_MAX_SERVO
  * @retval ret_code_t
  * @note   ⚠️ 同上，只发请求，回包还没解析。
  */
int  app_roboarm_read_servo_pos(const uint8_t *ids, uint8_t count);

/** @brief 组帧统计：ok_frames 一直是 0 → 参数非法或串口没通；pack_errors 涨 → 参数超范围 */
const proto_roboarm_stat_t *app_roboarm_get_stat(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ROBOARM_H */
