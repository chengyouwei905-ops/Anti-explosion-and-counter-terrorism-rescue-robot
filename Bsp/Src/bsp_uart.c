/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   串口板级封装实现
  * @note    ⚠️ 本文件独占实现 HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback。
  *          其它模块不要再实现这两个回调，否则会重复定义；
  *          需要感知数据请用 bsp_uart_set_rx_cb() 或轮询 bsp_uart_read()。
  ******************************************************************************
  */

#include "bsp_uart.h"
#include "common_def.h"
#include <stddef.h>

typedef struct {
    UART_HandleTypeDef *huart;                          /**< HAL 句柄 */
    ring_buffer_t       rx_rb;                          /**< 接收环形缓冲 */
    uint8_t             rx_storage[BSP_UART_RX_BUFSIZE];/**< 环形缓冲存储区 */
    uint8_t             rx_byte;                        /**< 中断单字节暂存 */
    bsp_uart_rx_cb_t    rx_cb;                          /**< 接收回调 */
    volatile uint32_t   rx_overflow;                    /**< 缓冲溢出丢字节计数 */
    bool                attached;                       /**< 是否已绑定句柄 */
} bsp_uart_dev_t;

static bsp_uart_dev_t s_uart[BSP_UART_ID_MAX];

static bsp_uart_dev_t *bsp_uart_dev(bsp_uart_id_t id)
{
    if ((uint32_t)id >= (uint32_t)BSP_UART_ID_MAX) {
        return NULL;
    }
    return &s_uart[id];
}

static bsp_uart_id_t bsp_uart_id_from_handle(UART_HandleTypeDef *huart)
{
    for (uint32_t i = 0U; i < (uint32_t)BSP_UART_ID_MAX; i++) {
        if (s_uart[i].attached && (s_uart[i].huart == huart)) {
            return (bsp_uart_id_t)i;
        }
    }
    return BSP_UART_ID_MAX;
}

int bsp_uart_attach(bsp_uart_id_t id, UART_HandleTypeDef *huart)
{
    bsp_uart_dev_t *dev = bsp_uart_dev(id);

    if ((dev == NULL) || (huart == NULL)) {
        return RET_PARAM;
    }

    dev->huart    = huart;
    dev->rx_cb    = NULL;
    dev->rx_byte  = 0U;
    dev->rx_overflow = 0U;
    dev->attached = true;
    rb_init(&dev->rx_rb, dev->rx_storage, BSP_UART_RX_BUFSIZE);

    return RET_OK;
}

int bsp_uart_start_rx(bsp_uart_id_t id)
{
    bsp_uart_dev_t *dev = bsp_uart_dev(id);

    if ((dev == NULL) || (!dev->attached)) {
        return RET_NOTREADY;
    }

    rb_reset(&dev->rx_rb);
    if (HAL_UART_Receive_IT(dev->huart, &dev->rx_byte, 1U) != HAL_OK) {
        return RET_ERROR;
    }

    return RET_OK;
}

void bsp_uart_set_rx_cb(bsp_uart_id_t id, bsp_uart_rx_cb_t cb)
{
    bsp_uart_dev_t *dev = bsp_uart_dev(id);

    if (dev != NULL) {
        dev->rx_cb = cb;
    }
}

int bsp_uart_send(bsp_uart_id_t id, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    bsp_uart_dev_t *dev = bsp_uart_dev(id);

    if ((dev == NULL) || (!dev->attached) || (data == NULL) || (len == 0U)) {
        return RET_PARAM;
    }
    if (HAL_UART_Transmit(dev->huart, (uint8_t *)(uintptr_t)data, len, timeout_ms) != HAL_OK) {
        return RET_TIMEOUT;
    }

    return RET_OK;
}

uint16_t bsp_uart_read(bsp_uart_id_t id, uint8_t *data, uint16_t len)
{
    bsp_uart_dev_t *dev = bsp_uart_dev(id);

    if (dev == NULL) {
        return 0U;
    }
    return rb_read(&dev->rx_rb, data, len);
}

uint16_t bsp_uart_available(bsp_uart_id_t id)
{
    bsp_uart_dev_t *dev = bsp_uart_dev(id);

    if (dev == NULL) {
        return 0U;
    }
    return rb_used(&dev->rx_rb);
}

void bsp_uart_flush_rx(bsp_uart_id_t id)
{
    bsp_uart_dev_t *dev = bsp_uart_dev(id);

    if (dev != NULL) {
        rb_reset(&dev->rx_rb);
    }
}

uint32_t bsp_uart_get_rx_overflow(bsp_uart_id_t id)
{
    bsp_uart_dev_t *dev = bsp_uart_dev(id);

    return (dev != NULL) ? dev->rx_overflow : 0U;
}

/* --------------------------------------------------------------------------
 * HAL 回调：本工程中由 bsp_uart.c 统一接管
 * ------------------------------------------------------------------------ */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_id_t id = bsp_uart_id_from_handle(huart);

    if (id == BSP_UART_ID_MAX) {
        return;
    }

    bsp_uart_dev_t *dev = &s_uart[id];

    if (!rb_putc(&dev->rx_rb, dev->rx_byte)) {
        dev->rx_overflow++;             /* 主循环取数太慢，缓冲区已满 */
    }
    if (dev->rx_cb != NULL) {
        dev->rx_cb(id, dev->rx_byte);
    }

    /* 立即重新武装，保证字节之间不产生丢包 */
    (void)HAL_UART_Receive_IT(huart, &dev->rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    bsp_uart_id_t id = bsp_uart_id_from_handle(huart);

    if (id == BSP_UART_ID_MAX) {
        return;
    }

    if (huart->ErrorCode != HAL_UART_ERROR_NONE) {
        /* 清掉过载/帧/噪声/校验错误标志，否则会一直重复进中断 */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
    }

    /* 错误后接收状态机可能已停止，重新武装 */
    (void)HAL_UART_Receive_IT(huart, &s_uart[id].rx_byte, 1U);
}
