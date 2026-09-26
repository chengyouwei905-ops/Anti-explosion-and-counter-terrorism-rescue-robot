/**
  ******************************************************************************
  * @file    app_vision.c
  * @brief   视觉模块应用层实现
  ******************************************************************************
  */

#include "app_vision.h"
#include "bsp_uart.h"
#include "common_def.h"
#include <stdio.h>
#include <string.h>

/** @brief 一次从环形缓冲取出的最大字节数 */
#define VISION_DRAIN_CHUNK      64U

static vision_target_t s_target;
static bool            s_inited = false;

/* -------------------------------------------------------------------------- */

/** @brief 协议层整帧回调：把 payload 翻译成 vision_target_t */
static void app_vision_on_frame(const proto_vision_frame_t *frame)
{
    if (frame == NULL) {
        return;
    }
    if (frame->cmd != (uint8_t)PROTO_VISION_CMD_TARGET) {
        return;                         /* 其它命令字暂不处理 */
    }
    if (frame->len < VISION_TARGET_PAYLOAD_LEN) {
        return;                         /* 载荷长度不足，丢弃 */
    }

    /* 丢帧统计：仅在上一帧仍有效时判断，避免刚上电时的误计数 */
    if (s_target.valid) {
        uint8_t expect = (uint8_t)(s_target.seq + 1U);
        if (frame->seq != expect) {
            s_target.lost_frames += (uint32_t)(uint8_t)(frame->seq - expect);
        }
    }

    const uint8_t *p = frame->payload;
    s_target.x_mm     = le_i16_get(&p[0]);
    s_target.y_mm     = le_i16_get(&p[2]);
    s_target.yaw_cdeg = le_i16_get(&p[4]);
    s_target.detect   = p[6];
    s_target.seq      = frame->seq;
    s_target.rx_tick  = HAL_GetTick();
    s_target.valid    = true;
}

int app_vision_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return RET_PARAM;
    }

    memset(&s_target, 0, sizeof(s_target));

    proto_vision_init();
    proto_vision_set_cb(app_vision_on_frame);

    if (bsp_uart_attach(BSP_UART_VISION, huart) != RET_OK) {
        return RET_ERROR;
    }
    if (bsp_uart_start_rx(BSP_UART_VISION) != RET_OK) {
        return RET_ERROR;
    }

    s_inited = true;
    return RET_OK;
}

void app_vision_poll(void)
{
    uint8_t  buf[VISION_DRAIN_CHUNK];
    uint16_t got;

    if (!s_inited) {
        return;
    }

    /* 一次性把缓冲区里的数据搬空，避免逐字节调用带来的开销 */
    while ((got = bsp_uart_read(BSP_UART_VISION, buf, (uint16_t)sizeof(buf))) > 0U) {
        proto_vision_feed(buf, got);
    }

    /* 统计串口溢出，方便定位"数据解码失败"的原因 */
    static uint32_t reported = 0U;
    uint32_t overflow = bsp_uart_get_rx_overflow(BSP_UART_VISION);
    while (reported < overflow) {
        proto_vision_notify_buf_error();
        reported++;
    }

    /* 超时保护：视觉掉线时不能继续用旧数据控制底盘 */
    if (s_target.valid && ((HAL_GetTick() - s_target.rx_tick) > VISION_TIMEOUT_MS)) {
        s_target.valid = false;
    }
}

const vision_target_t *app_vision_get_target(void)
{
    return &s_target;
}

const proto_vision_stat_t *app_vision_get_stat(void)
{
    return proto_vision_get_stat();
}

uint32_t app_vision_get_rx_overflow(void)
{
    return bsp_uart_get_rx_overflow(BSP_UART_VISION);
}

void app_vision_debug_echo(UART_HandleTypeDef *huart)
{
    char msg[96];
    int  n;
    const vision_target_t     *t = &s_target;
    const proto_vision_stat_t *s = proto_vision_get_stat();

    if (huart == NULL) {
        return;
    }

    n = snprintf(msg, sizeof(msg),
                 "vis v=%d x=%d y=%d yaw=%d det=%d seq=%u lost=%lu ok=%lu crc=%lu ovf=%lu\r\n",
                 (int)t->valid, (int)t->x_mm, (int)t->y_mm, (int)t->yaw_cdeg,
                 (int)t->detect, (unsigned)t->seq,
                 (unsigned long)t->lost_frames, (unsigned long)s->ok_frames,
                 (unsigned long)s->crc_errors,
                 (unsigned long)bsp_uart_get_rx_overflow(BSP_UART_VISION));

    if (n > 0) {
        (void)HAL_UART_Transmit(huart, (uint8_t *)msg, (uint16_t)n, 50U);
    }
}
