/**
  ******************************************************************************
  * @file    app_vision.c
  * @brief   视觉链路应用层实现：只管搬运和组包，不做业务解释
  ******************************************************************************
  */

#include "app_vision.h"
#include "bsp_uart.h"
#include <stddef.h>

static bool s_inited = false;

int app_vision_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return RET_PARAM;
    }

    /* 1) 绑定串口（HAL_UART_Init 已由 CubeMX 生成的 MX_UART7_Init 完成） */
    if (bsp_uart_attach(BSP_UART_VISION, huart) != RET_OK) {
        return RET_ERROR;
    }

    /* 2) 复位协议解析状态机 */
    proto_vision_init();

    /* 3) 启动中断接收 */
    if (bsp_uart_start_rx(BSP_UART_VISION) != RET_OK) {
        return RET_ERROR;
    }

    s_inited = true;
    return RET_OK;
}

void app_vision_set_rx_cb(proto_vision_frame_cb_t cb)
{
    proto_vision_set_cb(cb);
}

void app_vision_poll(void)
{
    uint8_t  buf[APP_VISION_CHUNK_SIZE];
    uint16_t got;

    if (!s_inited) {
        return;
    }

    /* 一次性把缓冲里的数据搬空，避免逐字节调用的开销。
       每次读到的可能只有半帧，proto_vision 内部有状态机会自己拼接。 */
    while ((got = bsp_uart_read(BSP_UART_VISION, buf, (uint16_t)sizeof(buf))) > 0U) {
        proto_vision_feed(buf, got);
    }
}

int app_vision_send(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[PROTO_VISION_OVERHEAD + PROTO_VISION_MAX_PAYLOAD];
    uint8_t frame_len;

    if (!s_inited) {
        return RET_NOTREADY;
    }

    /* 组帧：帧头/序号/长度/CRC 全部由协议层负责 */
    frame_len = proto_vision_pack(cmd, payload, len, frame, (uint8_t)sizeof(frame));
    if (frame_len == 0U) {
        return RET_PARAM;
    }

    return bsp_uart_send(BSP_UART_VISION, frame, frame_len, APP_VISION_TX_TIMEOUT_MS);
}

const proto_vision_stat_t *app_vision_get_stat(void)
{
    return proto_vision_get_stat();
}

uint32_t app_vision_get_rx_overflow(void)
{
    return bsp_uart_get_rx_overflow(BSP_UART_VISION);
}
