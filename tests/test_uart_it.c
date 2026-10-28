#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bsp_uart.h"
#include "app_chassis.h"

static unsigned blocking_sends;
static unsigned it_sends;
static char configs[8][BSP_UART_TX_BUFSIZE];
static uint32_t init_delay_ms;
static HAL_StatusTypeDef force_it_status = HAL_OK;
static HAL_StatusTypeDef force_block_status = HAL_OK;

void HAL_Delay(uint32_t delay)
{
    init_delay_ms += delay;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *data,
                                   uint16_t len, uint32_t timeout)
{
    (void)timeout;
    if (uart->gState != HAL_UART_STATE_READY) { return HAL_BUSY; }
    if (force_block_status != HAL_OK) { return force_block_status; }
    if (blocking_sends < 8U) {
        assert(len < BSP_UART_TX_BUFSIZE);
        memcpy(configs[blocking_sends], data, len);
        configs[blocking_sends][len] = '\0';
    }
    ++blocking_sends;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, const uint8_t *data,
                                      uint16_t len)
{
    if (uart->gState != HAL_UART_STATE_READY) { return HAL_BUSY; }
    if (force_it_status != HAL_OK) { return force_it_status; }
    /* Like HAL, retain the pointer; do not consume the data synchronously. */
    uart->test_tx_ptr = data;
    uart->test_tx_len = len;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    ++it_sends;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    (void)uart; (void)data; (void)len;
    return HAL_OK;
}

static void check_pending(UART_HandleTypeDef *uart, const char *expected)
{
    assert(uart->test_tx_len == strlen(expected));
    assert(memcmp(uart->test_tx_ptr, expected, uart->test_tx_len) == 0);
}

int main(void)
{
    UART_HandleTypeDef chassis = {.gState = HAL_UART_STATE_READY};
    UART_HandleTypeDef vision = {.gState = HAL_UART_STATE_READY};
    assert(bsp_uart_send_it(BSP_UART_CHASSIS, (const uint8_t *)"x", 1) == RET_NOTREADY);
    assert(app_chassis_init(&chassis) == RET_OK);
    const char *expected[] = {
        "$flash_reset#",
        "$mtype:1#", "$deadzone:1750#", "$mline:500#", "$wdiameter:77#",
        "$MPID:2.60,0.14,0.00#", "$spd:0,0,0,0#", "$upload:1,0,0#"
    };
    assert(blocking_sends == 8U && it_sends == 0U);
    assert(init_delay_ms == 3200U);
    for (unsigned i = 0; i < 8U; ++i) { assert(strcmp(configs[i], expected[i]) == 0); }

    /* Actual app layer's local frame must survive return from app_chassis_move. */
    assert(app_chassis_move(0, 100, 0) == RET_OK);
    assert(blocking_sends == 8U && it_sends == 1U);
    check_pending(&chassis, "$spd:-100,-100,-100,-100#");
    assert(app_chassis_move(0, 200, 0) == RET_BUSY);
    assert(it_sends == 1U);
    check_pending(&chassis, "$spd:-100,-100,-100,-100#");
    assert(bsp_uart_send(BSP_UART_CHASSIS, (const uint8_t *)"x", 1, 100) == RET_BUSY);
    check_pending(&chassis, "$spd:-100,-100,-100,-100#");

    /* HAL releases READY only at TX completion; next call sends current target. */
    chassis.gState = HAL_UART_STATE_READY;
    assert(app_chassis_move(0, 0, 0) == RET_OK);
    check_pending(&chassis, "$spd:0,0,0,0#");

    assert(bsp_uart_attach(BSP_UART_VISION, &vision) == RET_OK);
    char source[] = "caller-owned";
    assert(bsp_uart_send_it(BSP_UART_VISION, (const uint8_t *)source, sizeof(source)-1) == RET_OK);
    memset(source, '!', sizeof(source));
    check_pending(&vision, "caller-owned");
    check_pending(&chassis, "$spd:0,0,0,0#"); /* independent UART buffers */

    vision.gState = HAL_UART_STATE_READY;
    uint8_t too_long[BSP_UART_TX_BUFSIZE + 1U] = {0};
    assert(bsp_uart_send_it(BSP_UART_ID_MAX, too_long, 1) == RET_PARAM);
    assert(bsp_uart_send_it(BSP_UART_VISION, NULL, 1) == RET_PARAM);
    assert(bsp_uart_send_it(BSP_UART_VISION, too_long, 0) == RET_PARAM);
    assert(bsp_uart_send_it(BSP_UART_VISION, too_long, sizeof(too_long)) == RET_PARAM);
    force_it_status = HAL_ERROR;
    assert(bsp_uart_send_it(BSP_UART_VISION, too_long, 1) == RET_ERROR);
    force_it_status = HAL_BUSY;
    assert(bsp_uart_send_it(BSP_UART_VISION, too_long, 1) == RET_BUSY);
    force_it_status = HAL_OK;
    assert(bsp_uart_send_it(BSP_UART_VISION, too_long, BSP_UART_TX_BUFSIZE) == RET_OK);
    vision.gState = HAL_UART_STATE_READY;
    force_block_status = HAL_TIMEOUT;
    assert(bsp_uart_send(BSP_UART_VISION, too_long, 1, 100) == RET_TIMEOUT);
    force_block_status = HAL_ERROR;
    assert(bsp_uart_send(BSP_UART_VISION, too_long, 1, 100) == RET_ERROR);
    puts("PASS: chassis init ordering, IT speed frames, owned buffers, busy preservation, completion/retry and error returns.");
    return 0;
}
