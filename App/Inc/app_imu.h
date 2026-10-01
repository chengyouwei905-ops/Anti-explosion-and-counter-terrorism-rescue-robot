/**
  ******************************************************************************
  * @file    app_imu.h
  * @brief   IMU 应用层：采样节拍 + 姿态解算 + 对外发布姿态快照
  * @note    层次关系
  *            bsp_imu   （SPI2 + DMA 收发、BMI088 寄存器、输出 m/s^2、rad/s）
  *              -> alg_ahrs（Mahony 四元数解算，纯数学）
  *                -> app_imu（采样节拍 + 陀螺零偏标定 + 发布快照）  <-- 本文件
  *
  *          **本层把"节拍"和"读取"彻底分开**：
  *
  *            节拍侧  app_imu_poll()     —— 谁来调、多久调一次都不会错
  *            读取侧  app_imu_get_xxx()  —— 任意时刻调，只读最近一次快照，零时序要求
  *
  *          典型用法（两种放法效果一样，A 的节拍更稳，推荐）：
  *            // A. 放在 1ms 定时中断里
  *            void TIMx_IRQHandler(void) { app_imu_poll(); }
  *            // B. 放在主循环里空转（内部按 DWT 实测 dt 自节流到约 1kHz）
  *            while (1) { (void)app_imu_poll(); }
  *
  *            // 业务代码在任何地方、任何时候取姿态
  *            float e[3];
  *            if (app_imu_get_euler(e)) { 此时 e = [roll, pitch, yaw] }
  *
  *          ⚠️ 欧拉角单位是**弧度**，转角度制乘 57.29578f
  *          ⚠️ yaw 没有绝对参考，上电瞬间为 0，之后靠陀螺积分会缓慢漂移
  ******************************************************************************
  */

#ifndef __APP_IMU_H
#define __APP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "common_def.h"
#include "alg_ahrs.h"

/**
  * @brief  初始化 IMU：跑 BSP 驱动 + 开机陀螺零偏标定 + 复位姿态解算
  * @param  hspi SPI 句柄（本工程为 &hspi2），需已由 MX_SPI2_Init() 初始化
  * @retval RET_OK 成功；其它见 ret_code_t
  * @note   本函数会阻塞约 1 秒做零偏标定，期间板子必须**静止**
  *         （标定失败不致命：零偏按 0 处理，可用 app_imu_is_calibrated() 查询）
  */
int app_imu_init(SPI_HandleTypeDef *hspi);

/**
  * @brief  采样一拍：读传感器 -> 减零偏 -> 姿态解算 -> 发布快照
  * @retval RET_OK       本轮完成了一次解算
  *         RET_IDLE     距上次不足 APP_IMU_MIN_DT_S（陀螺 ODR 1kHz）本轮跳过，
  *                      **属于正常情况，不是错误**
  *         RET_NOTREADY 还没执行 app_imu_init()
  *         其它         传感器读写失败（累计在 app_imu_get_error_count()），
  *                      姿态保持上一次的值不变
  *
  * @note   **调用频率不敏感**（这是本文件的设计目标）：
  *         - 调太快：静默跳过并返回 RET_IDLE，不丢数据、不算错误
  *         - 调太慢：dt 由 DWT 实测，积分本身照样正确；只有间隔超过 1s
  *           （断点恢复 / 长时间阻塞那种）才按 1s 夹住，防止单拍积分把姿态带飞
  *         - 姿态只在"传感器读失败"时才停止更新
  * @note   推荐放进 1ms 定时中断里调用（节拍最稳）；放主循环里空转也同样正确。
  * @note   本函数**不可重入**：同一时刻只能有一个调用者（中断里调了就别再在主循环调）。
  */
int app_imu_poll(void);

/**
  * @brief  取欧拉角（读最近一次解算结果，任意时刻、任意频率可调）
  * @param  euler_rad 输出 [roll, pitch, yaw]，**弧度**，用 AHRS_ROLL 等枚举当下标
  * @retval true  数据有效
  *         false 还没有有效姿态（此时 euler_rad 被清零）
  */
bool app_imu_get_euler(float euler_rad[3]);

/** @brief 取四元数 [w, x, y, z]；返回 false 时输出清零 */
bool app_imu_get_quat(float quat[4]);

/** @brief 取角速度（已减掉标定出的零偏），单位 rad/s；返回 false 时输出清零 */
bool app_imu_get_gyro(float gyro_radps[3]);

/** @brief 取加速度，单位 m/s^2（含重力）；返回 false 时输出清零 */
bool app_imu_get_accel(float accel_mps2[3]);

/** @brief 取芯片温度，单位 ℃；暂无有效数据时返回 0 */
float app_imu_get_temp(void);

/** @brief 最近一次解算的等效采样率 Hz；暂无有效数据时返回 0。正常应接近 1000 */
float app_imu_get_rate_hz(void);

/** @brief 姿态解算是否已就绪（app_imu_init 成功后为 true） */
bool app_imu_is_ready(void);

/** @brief 开机陀螺零偏标定是否有效（标定时板子被晃动则为 false，此时零偏按 0 处理） */
bool app_imu_is_calibrated(void);

/** @brief 传感器读取失败累计次数，正常应为 0 */
uint32_t app_imu_get_error_count(void);

/**
  * @brief  取初始化失败的现场（bsp 层记录的），仅用于排查 ``app_imu_init()`` 失败
  * @param  dev    0 = 加速度计，1 = 陀螺仪，0xFF = 没失败过
  * @param  reg    出问题的寄存器地址（0x00 = WHO_AM_I 校验没过，说明器件没应答）
  * @param  expect 期望写入/读回的值
  * @param  actual 实际读回的值（`reg == 0x00` 时它就是读到的 ID）
  */
void app_imu_get_init_fail(uint8_t *dev, uint8_t *reg, uint8_t *expect, uint8_t *actual);

#ifdef __cplusplus
}
#endif

#endif /* __APP_IMU_H */
