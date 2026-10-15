/**
  ******************************************************************************
  * @file    bsp_imu.h
  * @brief   BMI088 六轴 IMU 驱动（硬件 SPI2，轮询收发）
  *
  * 硬件连接（与 CubeMX 配置一致）：
  *   SPI2 : PB13 = SCK, PC1 = MOSI, PC2_C = MISO, Mode3, NSS 软件
  *   PC0  : 加速度计片选 ACC_CS  （低有效）
  *   PC3  : 陀螺仪片选   GYRO_CS （低有效）
  *
  * 约定：
  *  - SPI2 用 HAL_SPI_TransmitReceive() 轮询收发（和参考例程一致），
  *    不使用 DMA / 中断，因此本文件**不实现**任何 HAL SPI 回调。
  *  - 拿到的数据已经换算好物理量：加速度 m/s^2、角速度 rad/s、温度 ℃。
  *  - ⚠️ SPI2 复用脚（PC1/PC2_C/PB13）速度必须是 VERY_HIGH：CubeMX 默认给的是 LOW，
  *    8MHz SCK 下驱动能力不足，会读回全 0。已在 HAL_SPI_MspInit() 的 USER CODE 里纠正。
  ******************************************************************************
  */

#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "common_def.h"

/** @brief 一次 IMU 采样结果 */
typedef struct
{
    float accel_mps2[3];   /**< 加速度 x/y/z，单位 m/s^2 */
    float gyro_radps[3];   /**< 角速度 x/y/z，单位 rad/s */
    float temp_c;          /**< 芯片温度，单位 ℃ */
} bsp_imu_data_t;

/**
  * @brief  初始化 BMI088（检查 WHO_AM_I、软复位、写配置寄存器并回读校验）
  * @param  hspi 已由 CubeMX 初始化好的 SPI 句柄，传 &hspi2
  * @retval RET_OK 成功；其它见 ret_code_t
  * @note   调用前必须已执行 MX_SPI2_Init() / MX_GPIO_Init()
  *         （不需要 MX_DMA_Init()：本驱动不用 DMA）
  */
ret_code_t bsp_imu_init(SPI_HandleTypeDef *hspi);

/** @brief 初始化是否成功（失败后不要调用 bsp_imu_read） */
bool bsp_imu_is_ready(void);

/**
  * @brief  读取一次加速度、角速度、温度
  * @param  out 结果输出，不可为 NULL
  * @retval RET_OK / RET_NOTREADY / RET_TIMEOUT ...
  */
ret_code_t bsp_imu_read(bsp_imu_data_t *out);

/* ---- 以下三个只用于调试排查，正常运行不需要调用 ---- */

/** @brief 初始化时读到的加速度计 WHO_AM_I，正常应为 0x1E */
uint8_t bsp_imu_get_accel_chip_id(void);

/** @brief 初始化时读到的陀螺仪 WHO_AM_I，正常应为 0x0F */
uint8_t bsp_imu_get_gyro_chip_id(void);

/**
  * @brief  初始化失败时的现场（在哪颗器件、哪个寄存器、期望值与实际值）
  * @param  dev    0 = 加速度计，1 = 陀螺仪，0xFF = 未失败
  * @param  reg    失败寄存器地址
  * @param  expect 期望写入/读回的值
  * @param  actual 实际读回的值
  */
void bsp_imu_get_init_fail(uint8_t *dev, uint8_t *reg, uint8_t *expect, uint8_t *actual);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IMU_H */
