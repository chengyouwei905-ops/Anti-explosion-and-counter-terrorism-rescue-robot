/**
  ******************************************************************************
  * @file    bsp_dwt.c
  * @brief   DWT 周期计数器实现
  *
  * 三个寄存器：
  *   CoreDebug->DEMCR  bit24 TRCENA  —— 使能整个跟踪单元，不开它 DWT 不工作
  *   DWT->LAR          解锁 CYCCNT（部分内核用锁保护，LSR.LOCKIMP 说明锁是否实现）
  *   DWT->CYCCNT       周期计数器本身，写 0 清零
  *   DWT->CTRL         bit0 CYCCNTENA —— 开始计数
  ******************************************************************************
  */

#include "bsp_dwt.h"

/** @brief DWT 解锁魔数（写 LAR 用） */
#define DWT_LAR_UNLOCK_KEY   0xC5ACCE55U
/** @brief LSR.LOCKIMP：为 1 表示该内核实现了寄存器锁，需要先解锁 */
#define DWT_LSR_LOCKIMP_Msk  0x00000001U

static bool  s_ready       = false;
static float s_us_per_cycle = 1.0f;

void bsp_dwt_init(void)
{
    /* 已经初始化过就直接返回：再写一次 CYCCNT=0 会让别人手里拿着的时间戳失效 */
    if (s_ready)
    {
        return;
    }

    /* 1) 使能跟踪单元 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 2) 若内核实现了寄存器锁，先解锁再访问 CYCCNT */
    if ((DWT->LSR & DWT_LSR_LOCKIMP_Msk) != 0U)
    {
        DWT->LAR = DWT_LAR_UNLOCK_KEY;
    }

    /* 3) 清零并启动计数 */
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* 4) 换算比例。
     *    DWT 按 CM7 的内核时钟计数，而本工程 HAL 里的 SystemCoreClock 正是
     *    "CM7 CPU frequency"（= SYSCLK >> D1CPRE，这里是 64MHz / 1）——
     *    注意它不是 SystemD2Clock(AXI/AHB)，两者在 D1CPRE ≠ HPRE 时会不一样。
     *    本工程 64MHz 下 1000000/64000000 = 0.015625 是精确的 2 的幂，毫无舍入误差 */
    s_us_per_cycle = 1000000.0f / (float)SystemCoreClock;

    s_ready = true;
}

bool bsp_dwt_is_ready(void)
{
    return s_ready;
}

uint32_t bsp_dwt_get_cycles(void)
{
    return DWT->CYCCNT;
}

float bsp_dwt_cycles_to_us(uint32_t cycles)
{
    /* dt 正常都远小于 2^23（0.1s @64MHz 也才 6.4e6），float 能精确表示，
     * 所以这里不会有精度问题；只有断点恢复那种超大 interval 才会粗化，
     * 而那种值本来就会被解算层丢掉 */
    return (float)cycles * s_us_per_cycle;
}

float bsp_dwt_cycles_to_s(uint32_t cycles)
{
    return ((float)cycles * s_us_per_cycle) * 0.000001f;
}
