#include "stm32h7xx_hal.h"
#include "app_chassis.h"
#include "bsp_uart.h"
#include "proto_chassis.h"
#include "alg_chassis_kinematics.h"
#include "common_def.h"


static bool s_inited = false;

int app_chassis_init(UART_HandleTypeDef *huart)
{

    if (huart == NULL) {
        return RET_PARAM;
    }

    /* 1) 绑定串口（HAL_UART_Init 已由 CubeMX 生成的 MX_USART1_Init 完成） */
    if (bsp_uart_attach(BSP_UART_CHASSIS, huart) != RET_OK) {
        return RET_ERROR;
    }

    /* 2) 复位协议解析状态机 */
    proto_chassis_init();

    /* 3) 启动中断接收 */
    if (bsp_uart_start_rx(BSP_UART_CHASSIS) != RET_OK) {
        return RET_ERROR;
    }


    /* 底盘数据初始化 */
    /* 实际硬件更改后需要在此进行修改 */
    char frame[PROTO_CHASSIS_MAX_FRAME + 1U];
    uint8_t frame_len;
    frame_len = proto_chassis_pack(PROTO_CHASSIS_CMD_MTYPE, "1", frame, sizeof(frame));
    if(bsp_uart_send(BSP_UART_CHASSIS, (uint8_t *)frame, frame_len, 100U) != RET_OK) {
        return RET_ERROR;
    }
    frame_len = proto_chassis_pack(PROTO_CHASSIS_CMD_DEADZONE, "1750", frame, sizeof(frame));    
    if(bsp_uart_send(BSP_UART_CHASSIS, (uint8_t *)frame, frame_len, 100U) != RET_OK) {
        return RET_ERROR;
    }
    frame_len = proto_chassis_pack(PROTO_CHASSIS_CMD_MLINE, "500", frame, sizeof(frame));
    if(bsp_uart_send(BSP_UART_CHASSIS, (uint8_t *)frame, frame_len, 100U) != RET_OK) {
        return RET_ERROR;
    }
    frame_len = proto_chassis_pack(PROTO_CHASSIS_CMD_WDIAMETER, "77", frame, sizeof(frame));
    if(bsp_uart_send(BSP_UART_CHASSIS, (uint8_t *)frame, frame_len, 100U) != RET_OK) {
        return RET_ERROR;
    }
    frame_len = proto_chassis_pack(PROTO_CHASSIS_CMD_MPID, "2.60,0.14,0.00", frame, sizeof(frame));
    if(bsp_uart_send(BSP_UART_CHASSIS, (uint8_t *)frame, frame_len, 100U) != RET_OK) {
        return RET_ERROR;
    }
    frame_len = proto_chassis_pack_spd((int16_t[4]){0, 0, 0, 0}, frame, sizeof(frame));/* 初始先锁死底盘 */
    if(bsp_uart_send(BSP_UART_CHASSIS, (uint8_t *)frame, frame_len, 100U) != RET_OK) {
        return RET_ERROR;
    }
    /* 底盘数据初始化完成 */

    s_inited = true;
    return RET_OK;
}

int app_chassis_move(float speed_x, float speed_y, float omega)
{
    if (!s_inited) {
        return RET_NOTREADY;
    }

    Chassis_State_t state;
    state.speed_x = speed_x;
    state.speed_y = speed_y;
    state.omega = omega;
    float wheel_speed[WHEEL_NUMBERS];
    alg_chassis_invert(&state, wheel_speed);    /* 麦轮底盘运动学逆解 */

    int16_t wheel_speed_int[WHEEL_NUMBERS];
    for (int i = 0; i < WHEEL_NUMBERS; i++) {
        wheel_speed_int[i] = (int16_t)CLAMP(wheel_speed[i], -32768.0f, 32767.0f);
    }

    /* 速度限幅 */
    for (int i = 0; i < WHEEL_NUMBERS; i++) {
        wheel_speed_int[i] = (int16_t)CLAMP(wheel_speed_int[i], PROTO_CHASSIS_SPD_MIN, PROTO_CHASSIS_SPD_MAX);
    }

    /* 轮序映射 含方向映射 */
    /* M1为右前，M2为右后，M3为左前，M4为左后 */
    int16_t wheel_speed_id[WHEEL_NUMBERS];
    wheel_speed_id[0] = -wheel_speed_int[RIGHT_FRONT_WHELL];
    wheel_speed_id[1] = -wheel_speed_int[RIGHT_REAR_WHELL];
    wheel_speed_id[2] = -wheel_speed_int[LEFT_FRONT_WHELL];
    wheel_speed_id[3] = -wheel_speed_int[LEFT_REAR_WHELL];

    char frame[PROTO_CHASSIS_MAX_FRAME + 1U];
    uint8_t frame_len;
    frame_len = proto_chassis_pack_spd(wheel_speed_id, frame, sizeof(frame));
    if(bsp_uart_send(BSP_UART_CHASSIS, (uint8_t *)frame, frame_len, 100U) != RET_OK) {
        return RET_ERROR;
    } /* 命令下放 */

    return RET_OK;
}


