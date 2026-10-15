/**
  ******************************************************************************
  * @file    bsp_rgb.h
  * @brief   单颗地址灯（WS2812 / RS2812 类）驱动 —— PA7 数据线
  *
  * 硬件（与 CubeMX 配置一致）：
  *   PA7 : RGB 数据线，GPIO 推挽输出 / Very High 速度 / 初始低
  *         CubeMX 生成的标号就是 RGB_Pin / RGB_GPIO_Port
  *   供电：灯珠电源由 Power_OUT1_EN / Power_OUT2_EN 控制，本驱动只管数据线，
  *         上电后要自己把供电使能拉高，否则数据照发但灯不亮。
  *
  * 协议（单线归零码，MSB 先出）：一帧 = 24 位 = 1 颗灯，默认 GRB 顺序（WS2812 系）
  *   0 码 = 高 390ns + 低 970ns
  *   1 码 = 高 800ns + 低 560ns
  *   每拍约 1.36us（一帧约 33us）；帧间低电平超过 300us 才被灯珠当成复位
  *
  * 为什么用软件翻转 + 空指令延时，而不是 TIM-PWM + DMA / SPI / DWT：
  *   PA7 在 .ioc 里只配成了普通 GPIO，没落到任何定时器通道上；换 PWM+DMA 或 SPI
  *   都得动 .ioc 并重新生成代码（本工程踩过坑）。位时序用"数 __NOP() 条数"卡
  *   （本工程 HCLK = 64MHz，1 条 NOP = 1 个周期 = 15.6ns），不依赖 DWT 等任何
  *   计数器，不会死等，单颗灯足够稳。
  *
  * 注意事项：
  *  1. 脉宽是按 64MHz 数指令数写死的（见 bsp_rgb.c 里的 RGB_BIT_0/1 宏），
  *     以后改主频、或发现时序对不上时，同步改那几条 NOP 的数量即可。
  *  2. 刷新一帧期间会关全局中断约 33us：115200 波特率一个字节要 87us，
  *     最多把一个字节的接收推迟到发完之后，不会丢数据。
  *  3. 只给主循环调用，不要在中断里调。
  ******************************************************************************
  */

#ifndef __BSP_RGB_H
#define __BSP_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "common_def.h"

/* ---- 可调项（都有默认值，需要时用 -D 覆盖编译，见下方各自说明）---- */

/** @brief 字节顺序：0 = GRB（WS2812 / RS2812 系默认），1 = RGB（少数灯珠） */
#ifndef BSP_RGB_COLOR_ORDER
#define BSP_RGB_COLOR_ORDER   0
#endif

/**
  * @brief 帧间复位用的低电平时间（us）
  * @note  WS2812B 手册只要 >50us，但老一点的 WS2812 / 部分 RS 系列要 >280us，
  *        这里取 300us 一次到位。这段延时是普通空循环，不精确，但复位对时间
  *        不敏感（50us~10ms 都行），够用。
  */
#ifndef BSP_RGB_RESET_US
#define BSP_RGB_RESET_US      300U
#endif

/** @brief 一颗灯一帧的位数（24 位：GRB 各 8 位） */
#define BSP_RGB_BITS          24U

/**
  * @brief  初始化 RGB 灯数据线（配 PA7）
  * @retval RET_OK 成功
  * @note   可重复调用（幂等）。若别的代码把 PA7 改成过别的模式，调一次即可复位。
  */
ret_code_t bsp_rgb_init(void);

/**
  * @brief  设置颜色并立即刷新一位灯珠（阻塞约 33us + 帧间复位约 300us）
  * @param  r/g/b 各通道亮度，0~255（0 灭，255 最亮）
  * @retval RET_OK 成功；RET_NOTREADY 还没 init
  * @note   调这个函数会先等满复位时间再发帧，所以连刷也不会串色。
  *         需要整体调暗（呼吸灯）时在调用方按比例缩 r/g/b 即可。
  */
ret_code_t bsp_rgb_set(uint8_t r, uint8_t g, uint8_t b);

/**
  * @brief  灭灯（等价于 bsp_rgb_set(0, 0, 0)）
  * @retval 同 bsp_rgb_set
  */
ret_code_t bsp_rgb_off(void);

/**
  * @brief  取当前颜色（最后一次 set 的值）
  * @param  r/g/b 结果输出，均不可为 NULL
  * @note   只是软件记录的颜色，不代表灯珠现在的实际状态
  */
void bsp_rgb_get(uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_RGB_H */
