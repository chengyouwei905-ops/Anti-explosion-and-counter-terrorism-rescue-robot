/**
  ******************************************************************************
  * @file    app_chassis.h
  * @brief   底盘链路应用层：串口收发的统一入口 + 里程（编码器）解算
  * @note    层次关系
  *            bsp_uart       （字节流 + 环形缓冲）
  *              -> proto_chassis（文本帧 $<命令>[:<参数>]#）
  *                -> app_chassis（搬运 + 里程解算）      <-- 本文件
  *
  *          典型用法：
  *            app_chassis_init(&huart1);              // 只调一次（内部会下发配置并请求编码器上报）
  *            while (1) { app_chassis_poll(); }       // 主循环周期调用，不调用里程不动
  *            float d = app_chassis_get_distance();   // 随时取累计行驶距离(mm)
  ******************************************************************************
  */

#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"      /* UART_HandleTypeDef 定义在这里 */
#include "proto_chassis.h"
#include "alg_chassis_kinematics.h"

/** @brief 一次从环形缓冲搬运的最大字节数 */
#define APP_CHASSIS_CHUNK_SIZE      64U

/* ==========================================================================
 * 里程解算参数（必须与实际电机 / 轮子一致，换硬件时改这里）
 * ========================================================================== */

/** @brief 轮子转一圈驱动板上报的编码器计数增量（500 线 × 4 倍频 × 30 减速比） */
#define APP_CHASSIS_ENCODER_CNT_PER_REV     (60000.0f)

/** @brief 轮径(mm)，与初始化时下发的 $wdiameter 保持一致 */
#define APP_CHASSIS_WHEEL_DIAMETER_MM       (77.0f)

/** @brief 编码器计数 -> 轮子行程(mm)：行程 = 计数增量 × 本系数
  * @note  标定方法：app_chassis_reset_distance() 清零后直线推车整 1 m，
  *        读 app_chassis_get_distance() 得 d(mm)，新系数 = 旧系数 × 1000 / d */
#define APP_CHASSIS_MM_PER_COUNT \
    (3.14159265f * APP_CHASSIS_WHEEL_DIAMETER_MM / APP_CHASSIS_ENCODER_CNT_PER_REV)

/** @brief 里程死区(mm)：一次上报解算出的中心位移小于它就当编码器抖动
  *        （驱动板静止时 total 也会来回跳几个计数，不加死区里程会缓慢"爬升"）
  * @note   死区只"暂缓结算"，不丢计数：位移先攒着，顶出死区后一次性计入，
  *         停车时最多有不到一个死区的位移还挂在账上
  * @note   静止时 distance 仍缓慢变大 → 加大；想让极慢速挪动也记上 → 减小 */
#define APP_CHASSIS_ODOM_DEADBAND_MM        (0.2f)

/**
  * @brief  初始化底盘链路（绑定串口 -> 复位协议状态机 -> 启动中断接收 -> 下发配置并请求编码器上报）
  * @param  huart 串口句柄（本工程为 &huart1）
  * @retval ret_code_t
  */
int  app_chassis_init(UART_HandleTypeDef *huart);

/** @brief 主循环周期调用：搬运串口数据并解析驱动板上报（不调用里程不动） */
void app_chassis_poll(void);

/**
  * @brief  底盘速度控制（内部做麦轮逆解 + 限幅 + 轮序/方向映射）
  * @param  speed_x 右向速度(mm/s)
  * @param  speed_y 前向速度(mm/s)
  * @param  omega   自转角速度(rad/s，俯视逆时针为正)
  * @retval RET_OK 已启动中断发送；RET_BUSY 串口上一帧未完成，本帧未发送。
  *         其余为参数/初始化错误。主循环调用；源帧已复制，不等待发送完成。
  */
int  app_chassis_move(float speed_x, float speed_y, float omega);

/**
  * @brief  取小车累计行驶距离（底盘中心点走过的路程，单位 mm）
  * @note   任意时刻可调；上电（或 app_chassis_reset_distance()）后从 0 开始
  */
float app_chassis_get_distance(void);

/** @brief 里程清零（重新设起点，不影响底盘控制） */
void app_chassis_reset_distance(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CHASSIS_H */
