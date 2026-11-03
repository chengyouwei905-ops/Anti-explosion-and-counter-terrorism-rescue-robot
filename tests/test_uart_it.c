#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "bsp_uart.h"
#include "app_chassis.h"
#include "alg_chassis_kinematics.h"

static unsigned blocking_sends;
static unsigned it_sends;
static char configs[8][BSP_UART_TX_BUFSIZE];
static uint32_t init_delay_ms;
static HAL_StatusTypeDef force_it_status = HAL_OK;
static HAL_StatusTypeDef force_block_status = HAL_OK;
static uint32_t tick_ms;
static uint8_t *encoder_rx;
uint32_t HAL_GetTick(void) { return tick_ms; }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);

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
    (void)uart; (void)len;
    encoder_rx = data;
    return HAL_OK;
}

static void check_pending(UART_HandleTypeDef *uart, const char *expected)
{
    assert(uart->test_tx_len == strlen(expected));
    assert(memcmp(uart->test_tx_ptr, expected, uart->test_tx_len) == 0);
}

static void encoder_frame(UART_HandleTypeDef *uart, const char *text)
{
    for (unsigned i=0;text[i];++i) { *encoder_rx=(uint8_t)text[i];HAL_UART_RxCpltCallback(uart); }
    app_chassis_poll();
}

static void test_distance_feedback(UART_HandleTypeDef *uart)
{
    Chassis_DistanceSample_t sample;
    assert(!app_chassis_get_distance_sample(&sample));
    assert(!app_chassis_get_distance_sample(NULL));
    tick_ms=100;
    encoder_frame(uart,"$MAll:0,0,0,0#");
    assert(app_chassis_get_distance_sample(&sample));
    assert(sample.distance_mm==0 && sample.age_ms==0 && sample.frame_id==1);
    uint32_t generation=sample.reset_count;
    tick_ms+=20;
    encoder_frame(uart,"$MAll:1000,1000,1000,1000#");
    float leg=1000*APP_CHASSIS_MM_PER_COUNT;
    assert(app_chassis_get_distance_sample(&sample));
    assert(fabsf(sample.distance_mm-leg)<.0001f && sample.age_ms==0);
    tick_ms+=20;
    encoder_frame(uart,"$MAll:0,0,0,0#");
    assert(fabsf(app_chassis_get_distance()-2*leg)<.0001f); /* Both directions count as mileage. */
    tick_ms+=501;
    assert(app_chassis_get_distance_sample(&sample) && sample.age_ms==501);
    uint32_t frame=sample.frame_id;
    encoder_frame(uart,"$MAll:bad#");
    assert(app_chassis_get_distance_sample(&sample) && sample.frame_id==frame && sample.age_ms==501);
    encoder_frame(uart,"$MAll:0,0,0,0#"); /* Valid stationary frames refresh age. */
    assert(app_chassis_get_distance_sample(&sample) && sample.age_ms==0 && sample.frame_id==frame+1);
    app_chassis_reset_distance();
    assert(app_chassis_get_distance_sample(&sample) && sample.distance_mm==0 && sample.reset_count!=generation);
    encoder_frame(uart,"$MAll:10,10,10,10#");
    assert(app_chassis_get_distance()==0); /* Existing 0.2mm deadband is preserved. */
    encoder_frame(uart,"$MAll:60,60,60,60#");
    assert(fabsf(app_chassis_get_distance()-60*APP_CHASSIS_MM_PER_COUNT)<.0001f);
    tick_ms=UINT32_MAX-10U;
    encoder_frame(uart,"$MAll:60,60,60,60#");
    tick_ms+=30U;
    assert(app_chassis_get_distance_sample(&sample) && sample.age_ms==30U);
}

static void test_field_lateral_mapping(void)
{
    /* Wheel order: LF, RF, LR, RR. +X must reverse the old field-left
     * combination; forward/backward and angular combinations stay intact.
     */
    const struct { Chassis_State_t body; float wheel[4]; } cases[] = {
        {{20,0,0}, {-20,20,20,-20}},
        {{-20,0,0}, {20,-20,-20,20}},
        {{0,20,0}, {20,20,20,20}},
        {{0,-20,0}, {-20,-20,-20,-20}},
        {{0,0,1}, {-105,105,-105,105}},
        {{0,0,-1}, {105,-105,105,-105}},
        {{30,50,.2f}, {-1,101,59,41}}
    };
    for (unsigned i=0; i<sizeof(cases)/sizeof(cases[0]); ++i) {
        float wheel[4];
        Chassis_State_t recovered;
        alg_chassis_invert(&cases[i].body,wheel);
        for (unsigned j=0; j<4; ++j) {
            assert(fabsf(wheel[j]-cases[i].wheel[j])<.00001f);
        }
        alg_chassis_forward(wheel,&recovered);
        assert(fabsf(recovered.speed_x-cases[i].body.speed_x)<.00001f);
        assert(fabsf(recovered.speed_y-cases[i].body.speed_y)<.00001f);
        assert(fabsf(recovered.omega-cases[i].body.omega)<.00001f);
    }
}

int main(void)
{
    test_field_lateral_mapping();
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
    test_distance_feedback(&chassis);

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

    /* Verify the actual app/motor-ID/protocol path, not only the matrix. */
    chassis.gState = HAL_UART_STATE_READY;
    assert(app_chassis_move(20,0,0) == RET_OK);
    check_pending(&chassis, "$spd:-20,20,20,-20#");
    chassis.gState = HAL_UART_STATE_READY;
    assert(app_chassis_move(-20,0,0) == RET_OK);
    check_pending(&chassis, "$spd:20,-20,-20,20#");
    puts("PASS: field lateral mapping/FK, forward/yaw preservation, motor-ID speed frames, UART buffers/busy/retry/errors.");
    return 0;
}
