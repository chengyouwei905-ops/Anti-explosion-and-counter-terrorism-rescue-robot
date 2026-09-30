/**
  ******************************************************************************
  * @file    bsp_rgb.c
  * @brief   单颗地址灯（WS2812 / RS2812 类）驱动实现
  *
  * 为什么等待循环要手写汇编：
  *   本工程 Debug 是 -O0，纯 C 写的"读 DWT 轮询"循环体大约 15 个周期（≈230ns），
  *   粒度比 WS2812 的 ±150ns 容差还粗，灯会乱色、闪烁。这里把等待写成 3 条指令
  *   （LDR / SUBS / BLO，约 4~6 周期 ≈ 60~90ns），并且卡的是**绝对时刻**
  *   （等到 CYCCNT >= t0 + N 才走），所以单拍误差不累积：每一拍的高电平宽度
  *   都独立地落在 [目标, 目标 + 一个循环粒度] 之内。
  *
  * 每拍的时间线（t 是"拉高瞬间"读到的周期数）：
  *     拉高 ──t──> 等到 t + T1H/T0H ──> 拉低 ──> 等到 t + 1.25us ──> 下一拍
  *   高电平宽度 = T1H / T0H（决定 0 还是 1，必须准）；
  *   低电平只是补足整拍，多出来的循环开销落在低电平里 —— 低电平长一点无害，
  *   只要不超过复位门限（300us）就不会被灯珠当成帧结束。
  ******************************************************************************
  */

#include "bsp_rgb.h"
#include "bsp_dwt.h"
#include "main.h"   /* CubeMX 生成的 RGB_GPIO_Port / RGB_Pin */

/* ---- 引脚操作：写 BSRR，不用 HAL_GPIO_WritePin（后者是函数调用，-O0 下太慢） ---- */
#define RGB_PIN_MASK      ((uint32_t)RGB_Pin)
#define RGB_PIN_HIGH()    (RGB_GPIO_Port->BSRR = RGB_PIN_MASK)
#define RGB_PIN_LOW()     (RGB_GPIO_Port->BSRR = (RGB_PIN_MASK << 16U))

/**
  * @brief 等到 DWT 周期计数器 >= target（绝对时刻，可自然处理回绕）
  * @note  用宏而不是函数：-O0 下一次函数调用的进出开销就有十几个周期，
  *        会把最短的 T0H（只有 22 个周期）吃掉一半
  * @note  带 "memory" 屏障，避免编译器把 BSRR 的写操作挪到等待之后（Release 下会踩）
  */
#define RGB_WAIT_UNTIL(target)                                          \
    do {                                                                \
        __asm volatile ("1:  LDR  r3, [%[p]]    \n\t"                   \
                        "    SUBS r2, r3, %[t]  \n\t"                   \
                        "    BLO  1b            \n\t"                   \
                        :                                               \
                        : [p] "r" (&DWT->CYCCNT), [t] "r" (target)      \
                        : "r2", "r3", "cc", "memory");                  \
    } while (0)

static bool     s_ready;        /**< 是否已 init */
static bool     s_sent;         /**< 是否发过帧（决定帧前要不要补复位间隔） */
static uint8_t  s_r, s_g, s_b;  /**< 最后一次 set 的颜色 */
static uint32_t s_frame_end;    /**< 上一帧结束时（引脚刚拉低）的 DWT 周期数 */
static uint32_t s_t0h_cyc;      /**< 0 码高电平周期数 */
static uint32_t s_t1h_cyc;      /**< 1 码高电平周期数 */
static uint32_t s_bit_cyc;      /**< 每拍总长周期数 */
static uint32_t s_reset_cyc;    /**< 帧间复位周期数 */

/**
  * @brief 纳秒 → DWT 周期数（四舍五入）
  * @note  用 64 位算避免 SystemCoreClock * ns 溢出；只在 init 里调，慢无所谓
  *        DWT 按 CM7 内核时钟计数，而本工程 SystemCoreClock 正是这个频率
  */
static uint32_t rgb_ns_to_cycles(uint32_t ns)
{
    uint64_t c = ((uint64_t)SystemCoreClock * (uint64_t)ns + 500000000ULL) / 1000000000ULL;

    return (uint32_t)c;
}

/**
  * @brief 发一帧：3 个字节（已经是灯珠要的字节顺序）按 MSB 先出，共 24 拍
  * @note  调用方负责保证帧前已经低电平了足够长的时间
  */
static void rgb_send_frame(uint8_t b0, uint8_t b1, uint8_t b2)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t bits;
    uint32_t t;
    uint32_t high;
    uint8_t  i;

    /* 24 位打包进一个 uint32 的高 24 位，发送时左移取下一位最高位 */
    bits = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8);

    /* 一帧约 40us，期间关中断：最坏只是让串口字节收晚一点，不会丢 */
    __disable_irq();

    for (i = 0U; i < BSP_RGB_BITS; i++)
    {
        /* 本拍要等多久先算好（这部分开销落在上一拍的低电平里，不影响高电平宽度） */
        high = ((bits & 0x80000000U) != 0U) ? s_t1h_cyc : s_t0h_cyc;

        RGB_PIN_HIGH();
        t = DWT->CYCCNT;               /* 本拍时间基准：紧跟引脚拉高 */
        RGB_WAIT_UNTIL(t + high);      /* 高电平宽度 = 0/1 判据，必须准 */
        RGB_PIN_LOW();
        RGB_WAIT_UNTIL(t + s_bit_cyc); /* 补足整拍，多出的开销落在低电平里 */

        bits <<= 1;
    }

    s_frame_end = DWT->CYCCNT;         /* 帧尾，引脚此后保持低电平 */
    s_sent      = true;

    __set_PRIMASK(primask);
}

ret_code_t bsp_rgb_init(void)
{
    GPIO_InitTypeDef gpio = { 0 };

    /* 位时序的基准是 DWT；bsp_dwt_init() 幂等，重复调用不会把计数器清零 */
    bsp_dwt_init();
    if (!bsp_dwt_is_ready())
    {
        return RET_ERROR;
    }

    /* PA7 配成推挽输出、Very High 速度、初始低。
       与 CubeMX 生成的配置一致，这里再写一遍是为了本驱动能独立使用：
       万一别的代码动过这个引脚，调一次 init 就能复位 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, GPIO_PIN_RESET);
    gpio.Pin   = RGB_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(RGB_GPIO_Port, &gpio);

    s_t0h_cyc   = rgb_ns_to_cycles(BSP_RGB_T0H_NS);
    s_t1h_cyc   = rgb_ns_to_cycles(BSP_RGB_T1H_NS);
    s_bit_cyc   = rgb_ns_to_cycles(BSP_RGB_BIT_NS);
    s_reset_cyc = rgb_ns_to_cycles(BSP_RGB_RESET_NS);

    s_r = 0U;
    s_g = 0U;
    s_b = 0U;
    s_sent = false;
    s_ready = true;

    return RET_OK;
}

ret_code_t bsp_rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_ready)
    {
        return RET_NOTREADY;
    }

    /* 帧间复位：这里中断是开着的（复位时间最长 300us，比一帧本身长得多，
       没必要也关中断），用绝对时刻等到最后一帧 + 复位时间之后 */
    if (s_sent && ((DWT->CYCCNT - s_frame_end) < s_reset_cyc))
    {
        RGB_WAIT_UNTIL(s_frame_end + s_reset_cyc);
    }

#if BSP_RGB_COLOR_ORDER == 1
    rgb_send_frame(r, g, b);           /* RGB 顺序的灯珠 */
#else
    rgb_send_frame(g, r, b);           /* WS2812 / RS2812 系默认 GRB */
#endif

    s_r = r;
    s_g = g;
    s_b = b;

    return RET_OK;
}

ret_code_t bsp_rgb_off(void)
{
    return bsp_rgb_set(0U, 0U, 0U);
}

void bsp_rgb_get(uint8_t *r, uint8_t *g, uint8_t *b)
{
    if ((r == NULL) || (g == NULL) || (b == NULL))
    {
        return;
    }

    *r = s_r;
    *g = s_g;
    *b = s_b;
}
