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
#include "common_def.h"
#include <stddef.h>
#include <stdint.h>

/* 0点偏移量Roboarm_Servo_Offset_Position_t在此使用 */
static uint16_t roboarm_servo_offset_position[ROBOARM_DOF] = {
    YAW_SERVO_OFFSET,
    JOINT1_OFFSET,
    JOINT2_OFFSET,
    JOINT3_OFFSET
};

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
    /* 通信串口初始化 */
    if (huart == NULL) {
        return RET_PARAM;
    }
    if (bsp_uart_attach(BSP_UART_ROBOARM, huart) != RET_OK) {
        return RET_ERROR;
    }

    proto_roboarm_init();
    alg_roboarm_init();     /* 机械臂运动学参数初始化 */
    app_roboarm_clamp_open();          /* 机械臂夹爪初始化为张开状态 */
    const Roboarm_Servo_t roboarm_servo[ROBOARM_DOF]= {
        {YAW_SERVO, 0.0},  
        {JOINT1, 0.0},  
        {JOINT2, 90/57.2958},  
        {JOINT3, 0.0}
    };
    if(app_roboarm_servo_move_direct(roboarm_servo, ROBOARM_DOF, 2000) != RET_OK) {
        return RET_ERROR;
    }   /* 机械臂舵机初始化到初始位置 */

    return RET_OK;
}

/* --------------------------------------------------------------------------
 * 直接发送舵机控制指令，直接控制舵机运动到指定位置
 * 注意Roboarm_Servo_t结构体的角度单位为弧度制，且以使机械臂竖直时的朝向为各舵机0度位置（含0点标定逻辑）
 * 不含死区检测，该指令会导致舵机与机械限位相撞，使用时应当注意
 * ------------------------------------------------------------------------ */
int  app_roboarm_servo_move_direct(const Roboarm_Servo_t *servos, uint8_t count, uint16_t time_ms)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    proto_roboarm_servo_t servos_sendcommand[ROBOARM_DOF + 1];      //用于底层proto控制命令发送端使用的中间变量

    /* 0点矫正 */
    for (uint8_t i = 0; i< count; i++) {
        switch (servos[i].servoname) {
        case YAW_SERVO:
                servos_sendcommand[i].id = YAW_SERVO;
                servos_sendcommand[i].position = (uint16_t)(-(servos[i].angle * 57.29577951308232 / 240.0 * 1000.0) + roboarm_servo_offset_position[i]);  //角度转化为舵机位置值
            break;
        case JOINT1:
                servos_sendcommand[i].id = JOINT1;
                servos_sendcommand[i].position = (uint16_t)(servos[i].angle * 57.29577951308232 / 240.0 * 1000.0 + roboarm_servo_offset_position[i]);  //角度转化为舵机位置值
            break;
        case JOINT2:
                servos_sendcommand[i].id = JOINT2;
                servos_sendcommand[i].position = (uint16_t)(-(servos[i].angle * 57.29577951308232 / 240.0 * 1000.0) + roboarm_servo_offset_position[i]);  //角度转化为舵机位置值
            break;
        case JOINT3:
                servos_sendcommand[i].id = JOINT3;
                servos_sendcommand[i].position = (uint16_t)(servos[i].angle * 57.29577951308232 / 240.0 * 1000.0 + roboarm_servo_offset_position[i]);  //角度转化为舵机位置值
            break;
        case JOINT_CLAMP:
                servos_sendcommand[i].id = JOINT_CLAMP;
                servos_sendcommand[i].position = (uint16_t)(servos[i].angle * 57.29577951308232 / 240.0 * 1000.0);  //角度转化为舵机位置值
            break;
        default:    
            break;
        }
    }

    n = proto_roboarm_pack_servo_move(servos_sendcommand, count, time_ms, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_servo_move_to(double x, double y, double z, double pitch)
{
    End_Effector_Pos_t end_effector_pos = {x, y, z, pitch};
    Roboarm_State_t roboarm_state;
    if(alg_roboarm_invert(&end_effector_pos, &roboarm_state) == ROBOARM_KINEMATICS_OK)  /* 机械臂运动学逆解 */
    {
        const Roboarm_Servo_t roboarm_servo[4]= {
            {YAW_SERVO, roboarm_state.yaw_angle},
            {JOINT1, roboarm_state.joint1_angle},
            {JOINT2, roboarm_state.joint2_angle},
            {JOINT3, roboarm_state.joint3_angle}
        };
        /* 暂定一个通用position */
        uint16_t time_ms = 3000;
        if(app_roboarm_servo_move_direct(roboarm_servo, ROBOARM_DOF, time_ms) != RET_OK) {
            return RET_ERROR;
        }
        return RET_OK;
    }
    else 
    {
        return RET_ERROR;
    }
}

void app_roboarm_clamp_open(void)
{
    const Roboarm_Servo_t roboarm_servo[1]= {
        {JOINT_CLAMP, CLAMP_OPEN_POSITION}
    };
    app_roboarm_servo_move_direct(roboarm_servo, 1, 1000);
}

void app_roboarm_clamp_close(void)
{
    const Roboarm_Servo_t roboarm_servo[1]= {
        {JOINT_CLAMP, CLAMP_CLOSE_POSITION}
    };
    app_roboarm_servo_move_direct(roboarm_servo, 1, 1000);
}

int app_roboarm_action_group_run(uint8_t group, uint16_t times)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    n = proto_roboarm_pack_action_group_run(group, times, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_action_group_stop(void)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    n = proto_roboarm_pack_action_group_stop(frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_action_group_speed(uint8_t group, uint16_t percent)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    n = proto_roboarm_pack_action_group_speed(group, percent, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_servo_unload(const uint8_t *ids, uint8_t count)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    n = proto_roboarm_pack_servo_unload(ids, count, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_read_battery_voltage(void)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    n = proto_roboarm_pack_get_battery_voltage(frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

int app_roboarm_read_servo_pos(const uint8_t *ids, uint8_t count)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t n;

    n = proto_roboarm_pack_servo_pos_read(ids, count, frame, (uint8_t)sizeof(frame));

    return send_frame(frame, n);
}

const proto_roboarm_stat_t *app_roboarm_get_stat(void)
{
    return proto_roboarm_get_stat();
}
