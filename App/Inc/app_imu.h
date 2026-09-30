/**
  ******************************************************************************
  * @file    app_imu.h
  * @brief   IMU 应用层：把"读传感器"和"姿态解算"串成一条线，对外只给欧拉角
  * @note    层次关系
  *            bsp_imu   （SPI2 + DMA 收发、BMI088 寄存器、输出 m/s^2、rad/s）
  *              -> alg_ahrs（Mahony 四元数解算，纯数学）
  *                -> app_imu（采样节拍 + 陀螺零偏标定 + 对外欧拉角）  <-- 本文件
  *
  *          典型用法（主循环至少要跑到 100Hz 以上，否则姿态会停更）：
  *            app_imu_init(&hspi2);          // 放在 while(1) 之前
  *            while (1) {
  *                app_imu_poll();            // 每轮调一次
  *                float e[3];
  *                app_imu_get_euler(e);      // e[AHRS_ROLL] / [AHRS_PITCH] / [AHRS_YAW]
  *            }
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
  * @brief  主循环周期调用：读一次传感器 -> 减零偏 -> 姿态解算
  * @retval RET_OK    本次完成了一次解算
  *         RET_BUSY  距上次不足 1ms（陀螺 ODR 是 1kHz，更密地读只会拿到重复样本），本轮跳过
  *         其它      传感器读写失败，姿态保持上一次的值不变
  *
  * @note   **不需要固定周期**：dt 由 DWT 周期计数器实测（分辨率约 15.6ns），
  *         主循环节奏不齐也不影响陀螺积分的正确性。
  * @note   上下限：太快（< 1ms）会被跳过；解算层会丢弃 dt > 0.1s 的调用，
  *         所以调用率**必须大于 10Hz**，否则姿态不更新。
  * @note   想把解算频率提到 1kHz 以上：把 app_imu.c 里的 APP_IMU_MIN_DT_S 调小，
  *         **同时**改 bsp_imu.c 里陀螺的 ODR（GYRO_BANDWIDTH），否则只是白读重复样本。
  */
int app_imu_poll(void);

/**
  * @brief  取欧拉角
  * @param  euler_rad 输出 [roll, pitch, yaw]，**弧度**，用 AHRS_ROLL 等枚举当下标
  */
void app_imu_get_euler(float euler_rad[3]);

/** @brief 取角速度（已减掉标定出的零偏），单位 rad/s */
void app_imu_get_gyro(float gyro_radps[3]);

/** @brief 取加速度，单位 m/s^2（含重力） */
void app_imu_get_accel(float accel_mps2[3]);

/** @brief 取芯片温度，单位 ℃（参考例程用它做温控，这里先留着） */
float app_imu_get_temp(void);

/** @brief 姿态解算是否已就绪（app_imu_init 成功后为 true） */
bool app_imu_is_ready(void);

/** @brief 开机陀螺零偏标定是否有效（标定时板子被晃动则为 false，此时零偏按 0 处理） */
bool app_imu_is_calibrated(void);

/** @brief 传感器读取失败累计次数，正常应为 0 */
uint32_t app_imu_get_error_count(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_IMU_H */
