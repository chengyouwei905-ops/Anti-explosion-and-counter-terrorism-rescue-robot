/**
  ******************************************************************************
  * @file    bsp_dwt.h
  * @brief   DWT 周期计数器：给上层提供高精度时间基准
  *
  * 为什么不用 HAL_GetTick()：
  *   它只有 1ms 的**整数**分辨率。1kHz 采样时 dt_ms 大多为 1、偶尔为 2，
  *   单拍的 dt 量化误差可达 ±100%（长期平均是对的，但每拍都在抖）。
  *   DWT->CYCCNT 由内核按 HCLK 自增，本工程 HCLK = 64MHz → 分辨率 15.6ns。
  *
  * 用法（测两个时刻之间的间隔）：
  *     uint32_t t0 = bsp_dwt_get_cycles();
  *     ... 干点事 ...
  *     float us = bsp_dwt_cycles_to_us(bsp_dwt_get_cycles() - t0);
  *
  * ⚠️ 32 位计数器会在 2^32 个周期后回绕（64MHz 下约 67 秒）。上面那种
  *    "无符号减法取差值"的写法能自然处理回绕，只要**单次间隔 < 67 秒**即可。
  * ⚠️ 调试器暂停 CPU 时 CYCCNT 仍在走，所以断点恢复后测出来的 dt 会很大。
  *    解算层有 dt > 0.1s 的保护，会把那一拍丢掉。
  ******************************************************************************
  */

#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "common_def.h"

/**
  * @brief  初始化 DWT 周期计数器
  * @note   必须在 SystemClock_Config() 之后调用（要读 SystemCoreClock 换算比例）
  * @note   可安全重复调用：已经初始化过就直接返回，不会再次清零计数器
  *         （清零会让别人手里拿着的时间戳失效）
  */
void bsp_dwt_init(void);

/** @brief 计数器是否已就绪（bsp_dwt_init 成功后为 true） */
bool bsp_dwt_is_ready(void);

/** @brief 取当前 CPU 周期计数（HCLK 频率自增，会回绕） */
uint32_t bsp_dwt_get_cycles(void);

/**
  * @brief  周期数 → 微秒
  * @param  cycles 周期数（两个时刻相减得到，允许回绕）
  */
float bsp_dwt_cycles_to_us(uint32_t cycles);

/**
  * @brief  周期数 → 秒
  * @param  cycles 周期数（同上）
  */
float bsp_dwt_cycles_to_s(uint32_t cycles);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DWT_H */
