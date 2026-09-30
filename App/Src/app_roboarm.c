/**
  ******************************************************************************
  * @file    app_roboarm.c
  * @brief   机械臂（总线舵机控制板）应用层实现：组帧 + 下发
  * @note    每个函数都是固定套路：
  *            组帧（协议层） -> 失败就返回 RET_PARAM（协议层已记 pack_errors）
  *                          -> 成功就交给 BSP 串口发出去
  *          帧缓冲都是函数内的局部数组（最长 32 字节），不占静态 RAM，也不用加锁。
  ******************************************************************************
  */

#include "app_roboarm.h"
#include "bsp_uart.h"
#include <stddef.h>

static bool s_inited = false;

/* --------------------------------------------------------------------------
 * 组帧 + 发送的公共出口
 * ------------------------------------------------------------------------ */

static int send_frame(const uint8_t *frame, uint8_t frame_len)
{
    if (frame_len == 0U) {
        return RET_PARAM;       /* 参数非法，协议层已经记过一笔 pack_errors */
    }
    return bsp_uart_send(BSP_UART_ROBOARM, frame, frame_len, APP_ROBOARM_TX_TIMEOUT_MS);
}

/* --------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------ */

int app_roboarm_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return RET_PARAM;
    }

    /* 1) 绑定串口（HAL_UART_Init 已由 CubeMX 生成的 MX_USART10_UART_Init 完成）
          ⚠️ CubeMX 里 USART10 的波特率必须是 9600，否则下面照发不误但控制板不理 */
    if (bsp_uart_attach(BSP_UART_ROBOARM, huart) != RET_OK) {
        return RET_ERROR;
    }

    /* 2) 复位组帧统计 */
    proto_roboarm_init();

    /* 3) 只发不收，这里不启动中断接收（要收控制板上报时再 start_rx） */

    s_inited = true;
    return RET_OK;
}

/* --------------------------------------------------------------------------
 * 发送
 * ------------------------------------------------------------------------ */

int app_roboarm_servo_move(const proto_roboarm_servo_t *servos, uint8_t count,
                           uint16_t time_ms)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    if (!s_inited) {
        return RET_NOTREADY;
    }

    n = proto_roboarm_pack_servo_move(servos, count, time_ms, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_action_group_run(uint8_t group, uint16_t times)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    if (!s_inited) {
        return RET_NOTREADY;
    }

    n = proto_roboarm_pack_action_group_run(group, times, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_action_group_stop(void)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    if (!s_inited) {
        return RET_NOTREADY;
    }

    n = proto_roboarm_pack_action_group_stop(frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_action_group_speed(uint8_t group, uint16_t percent)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    if (!s_inited) {
        return RET_NOTREADY;
    }

    n = proto_roboarm_pack_action_group_speed(group, percent, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_servo_unload(const uint8_t *ids, uint8_t count)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    if (!s_inited) {
        return RET_NOTREADY;
    }

    n = proto_roboarm_pack_servo_unload(ids, count, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_read_battery_voltage(void)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    if (!s_inited) {
        return RET_NOTREADY;
    }

    n = proto_roboarm_pack_get_battery_voltage(frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_read_servo_pos(const uint8_t *ids, uint8_t count)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    if (!s_inited) {
        return RET_NOTREADY;
    }

    n = proto_roboarm_pack_servo_pos_read(ids, count, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

const proto_roboarm_stat_t *app_roboarm_get_stat(void)
{
    return proto_roboarm_get_stat();
}
