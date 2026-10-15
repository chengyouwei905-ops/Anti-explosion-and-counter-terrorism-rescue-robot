/**
  ******************************************************************************
  * @file    bsp_rgb.c
  * @brief   单颗地址灯（WS2812 / RS2812 类）驱动实现 —— 阻塞式延时版
  *
  * 时间基准：直接数 CPU 周期。本工程 HCLK = 64MHz（HSI 64MHz / DIV1），
  *   1 个周期 = 15.6ns；__NOP() 是 volatile，编译到哪一档都是一条 1 周期指令，
  *   把高电平写成固定条数的 __NOP() 序列，脉宽就是确定的 ——
  *   不用 DWT、不用定时器，不会死等，也不会因为计数器没起来就卡住。
  *
  * 每拍的时间线（高电平是 0/1 判据，必须准；低电平只补足整拍，多几拍无害）：
  *     0 码：拉高 ─24 条 NOP─ 拉低 ─54 条 NOP─ 下一拍    高 ≈ 390ns
  *     1 码：拉高 ─50 条 NOP─ 拉低 ─28 条 NOP─ 下一拍    高 ≈ 800ns
  *   一拍 ≈ 87 个周期（≈1.36us），一帧 24 拍 ≈ 33us。
  *   低电平长一点没关系，只要不超过复位门限（>50us）就不会被灯珠当成帧结束。
  ******************************************************************************
  */

#include "bsp_rgb.h"
#include "main.h"   /* CubeMX 生成的 RGB_GPIO_Port / RGB_Pin */

/* ---- 引脚操作：写 BSRR，不用 HAL_GPIO_WritePin（后者是函数调用，太慢） ---- */
#define RGB_PIN_MASK      ((uint32_t)RGB_Pin)
#define RGB_PIN_HIGH()    (RGB_GPIO_Port->BSRR = RGB_PIN_MASK)
#define RGB_PIN_LOW()     (RGB_GPIO_Port->BSRR = (RGB_PIN_MASK << 16U))

/* ---- 阻塞延时：数 __NOP() 条数 ----
   __NOP() 是 volatile，既不会被优化掉，也不会被编译器挪到 IO 操作的前后。
   本工程 HCLK = 64MHz，1 条 NOP = 1 个周期 = 15.6ns */
#define RGB_NOP_2()       do { __NOP(); __NOP(); } while (0)
#define RGB_NOP_4()       do { RGB_NOP_2(); RGB_NOP_2(); } while (0)
#define RGB_NOP_8()       do { RGB_NOP_4(); RGB_NOP_4(); } while (0)
#define RGB_NOP_16()      do { RGB_NOP_8(); RGB_NOP_8(); } while (0)
#define RGB_NOP_32()      do { RGB_NOP_16(); RGB_NOP_16(); } while (0)

/* 0 码：高 = 1（BSRR 写）+ 24 条 NOP = 25 周期 ≈ 390ns；低 54 周期补足整拍 */
#define RGB_BIT_0()                                                            \
    do {                                                                       \
        RGB_PIN_HIGH(); RGB_NOP_16(); RGB_NOP_8();                             \
        RGB_PIN_LOW();  RGB_NOP_32(); RGB_NOP_16(); RGB_NOP_4(); RGB_NOP_2();  \
    } while (0)

/* 1 码：高 = 1（BSRR 写）+ 50 条 NOP = 51 周期 ≈ 800ns；低 28 周期补足整拍 */
#define RGB_BIT_1()                                                            \
    do {                                                                       \
        RGB_PIN_HIGH(); RGB_NOP_32(); RGB_NOP_16(); RGB_NOP_2();               \
        RGB_PIN_LOW();  RGB_NOP_16(); RGB_NOP_8();  RGB_NOP_4();               \
    } while (0)

static bool     s_ready;        /**< 是否已 init */
static uint8_t  s_r, s_g, s_b;  /**< 最后一次 set 的颜色 */

/**
  * @brief 帧间复位：数据线保持低电平 BSP_RGB_RESET_US 微秒左右
  * @note  空循环一轮（volatile 变量的读改写）大约 7 个周期 ≈ 110ns，
  *        所以循环次数取"微秒数 × 9"。这里只要求够长（>50us），不要求准确
  */
static void rgb_reset_delay(void)
{
    for (volatile uint32_t i = 0U; i < (BSP_RGB_RESET_US * 9U); i++)
    {
    }
}

/**
  * @brief 发一帧：3 个字节（已经是灯珠要的字节顺序）按 MSB 先出，共 24 拍
  * @note  调用方负责保证帧前已经低电平了足够长的时间
  */
static void rgb_send_frame(uint8_t b0, uint8_t b1, uint8_t b2)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t bits;
    uint8_t  i;

    /* 24 位打包进一个 uint32 的高 24 位，发送时左移取下一位最高位 */
    bits = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8);

    /* 一帧约 33us，期间关中断：免得中断把某一拍的低电平拉长，让灯珠解错码 */
    __disable_irq();

    for (i = 0U; i < BSP_RGB_BITS; i++)
    {
        if ((bits & 0x80000000U) != 0U)
        {
            RGB_BIT_1();
        }
        else
        {
            RGB_BIT_0();
        }

        bits <<= 1;
    }

    RGB_PIN_LOW();                     /* 帧尾保持低电平 */

    __set_PRIMASK(primask);
}

ret_code_t bsp_rgb_init(void)
{
    GPIO_InitTypeDef gpio = { 0 };

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

    s_r = 0U;
    s_g = 0U;
    s_b = 0U;
    s_ready = true;

    /* 上电先给一段复位低电平，把灯珠从"上电默认（白灯）"状态里拉出来 */
    rgb_reset_delay();

    return RET_OK;
}

ret_code_t bsp_rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_ready)
    {
        return RET_NOTREADY;
    }

    /* 帧间复位：灯珠要看到 50us 以上的低电平，才会把新一帧锁存到输出上 */
    rgb_reset_delay();

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
