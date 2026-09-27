/**
  ******************************************************************************
  * @file    proto_vision.c
  * @brief   视觉链路帧协议实现（流式状态机）
  ******************************************************************************
  */

#include "proto_vision.h"
#include "common_def.h"
#include <stddef.h>
#include <string.h>

/** @brief 解包状态机状态 */
typedef enum {
    PS_SOF1 = 0,    /**< 等待帧头第 1 字节 */
    PS_SOF2,        /**< 等待帧头第 2 字节 */
    PS_LEN,         /**< 长度 */
    PS_SEQ,         /**< 序号 */
    PS_CMD,         /**< 命令字 */
    PS_PAYLOAD,     /**< 载荷 */
    PS_CRC,         /**< 校验 */
} proto_vision_state_t;

static struct {
    proto_vision_state_t           state;
    proto_vision_frame_t    frame;      /**< 正在组装的帧 */
    uint8_t                 idx;        /**< 载荷写入下标 */
    uint8_t                 crc;        /**< 增量 CRC */
    uint8_t                 tx_seq;     /**< 发送序号 */
    proto_vision_frame_cb_t cb;
    proto_vision_stat_t     stat;
} s_ctx;

/* -------------------------------------------------------------------------- */

static uint8_t crc8_update(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t i = 0U; i < 8U; i++) {
        crc = (uint8_t)((crc & 0x01U) ? ((crc >> 1) ^ 0x8CU) : (crc >> 1));
    }
    return crc;
}

uint8_t proto_vision_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;

    if (data == NULL) {
        return crc;
    }
    while (len > 0U) {
        crc = crc8_update(crc, *data);
        data++;
        len--;
    }
    return crc;
}

void proto_vision_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = PS_SOF1;
}

void proto_vision_set_cb(proto_vision_frame_cb_t cb)
{
    s_ctx.cb = cb;
}

const proto_vision_stat_t *proto_vision_get_stat(void)
{
    return &s_ctx.stat;
}

uint8_t proto_vision_pack(uint8_t cmd, const uint8_t *payload, uint8_t len,
                          uint8_t *out, uint8_t out_size)
{
    uint8_t idx = 0U;
    uint8_t total;

    if ((out == NULL) || (len > PROTO_VISION_MAX_PAYLOAD)) {
        return 0U;
    }
    if ((len > 0U) && (payload == NULL)) {
        return 0U;
    }

    total = (uint8_t)(PROTO_VISION_OVERHEAD + len);
    if (out_size < total) {
        return 0U;
    }

    out[idx++] = PROTO_VISION_SOF1;
    out[idx++] = PROTO_VISION_SOF2;
    out[idx++] = len;
    out[idx++] = s_ctx.tx_seq++;
    out[idx++] = cmd;

    if (len > 0U) {
        memcpy(&out[idx], payload, len);
        idx = (uint8_t)(idx + len);
    }

    /* CRC 覆盖 LEN / SEQ / CMD / PAYLOAD，即从 out[2] 开始的 (3 + len) 字节 */
    out[idx++] = proto_vision_crc8(&out[2], (uint16_t)(3U + len));

    return idx;
}

void proto_vision_feed(const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return;
    }

    for (uint16_t i = 0U; i < len; i++) {
        uint8_t byte = data[i];

        s_ctx.stat.rx_bytes++;

        switch (s_ctx.state) {
        case PS_SOF1:
            if (byte == PROTO_VISION_SOF1) {
                s_ctx.state = PS_SOF2;
            }
            break;

        case PS_SOF2:
            if (byte == PROTO_VISION_SOF2) {
                s_ctx.state = PS_LEN;
            } else if (byte != PROTO_VISION_SOF1) {
                s_ctx.state = PS_SOF1;
            }
            /* byte == 0xAA：可能是新帧的帧头，保持在 PS_SOF2 继续等 0x55 */
            break;

        case PS_LEN:
            if (byte > PROTO_VISION_MAX_PAYLOAD) {
                s_ctx.stat.len_errors++;
                s_ctx.state = PS_SOF1;
                break;
            }
            s_ctx.frame.len = byte;
            s_ctx.crc       = crc8_update(0x00U, byte);   /* CRC 从 LEN 开始 */
            s_ctx.state     = PS_SEQ;
            break;

        case PS_SEQ:
            s_ctx.frame.seq = byte;
            s_ctx.crc       = crc8_update(s_ctx.crc, byte);
            s_ctx.state     = PS_CMD;
            break;

        case PS_CMD:
            s_ctx.frame.cmd = byte;
            s_ctx.crc       = crc8_update(s_ctx.crc, byte);
            s_ctx.idx       = 0U;
            s_ctx.state     = (s_ctx.frame.len > 0U) ? PS_PAYLOAD : PS_CRC;
            break;

        case PS_PAYLOAD:
            s_ctx.frame.payload[s_ctx.idx] = byte;
            s_ctx.idx++;
            s_ctx.crc = crc8_update(s_ctx.crc, byte);
            if (s_ctx.idx >= s_ctx.frame.len) {
                s_ctx.state = PS_CRC;
            }
            break;

        case PS_CRC:
            if (byte == s_ctx.crc) {
                s_ctx.stat.ok_frames++;
                if (s_ctx.cb != NULL) {
                    s_ctx.cb(&s_ctx.frame);
                }
            } else {
                s_ctx.stat.crc_errors++;
            }
            s_ctx.state = PS_SOF1;
            break;

        default:
            s_ctx.state = PS_SOF1;
            break;
        }
    }
}
