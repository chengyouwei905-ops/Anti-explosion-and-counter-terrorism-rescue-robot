#include "stm32h7xx_hal.h"
#include "app_chassis.h"
#include "bsp_uart.h"
#include "proto_chassis.h"
#include "alg_chassis_kinematics.h"
#include "common_def.h"
#include <string.h>
#include <math.h>


static bool s_inited = false;

/* ==========================================================================
 * 里程解算
 * ========================================================================== */

/** @brief 轮子索引(Wheel_ID_t) -> 驱动板电机索引：M1=右前 M2=右后 M3=左前 M4=左后 */
static const uint8_t s_motor_of_wheel[WHEEL_NUMBERS] = { 2U, 0U, 3U, 1U };

static int64_t s_enc_prev[PROTO_CHASSIS_MOTOR_MAX];      /**< 上一次结算时的编码器 total（死区内不推进基准） */
static bool    s_enc_prev_valid = false;                 /**< 是否已记下基准（第一帧只记基准） */
static float   s_distance_mm    = 0.0f;                  /**< 累计行驶路程(mm) */

/* 本段四轮计数增量 -> 各轮行程(mm) -> 运动学正解出底盘中心位移 -> 累计路程 */
static void app_chassis_update_odom(const int32_t enc[PROTO_CHASSIS_MOTOR_MAX])
{
    int32_t         delta[PROTO_CHASSIS_MOTOR_MAX];
    float           wheel_travel[WHEEL_NUMBERS];
    Chassis_State_t displacement;       /* 运动学是线性的：字段装的是"本段中心位移(mm)" */
    float           step_mm;

    if (!s_enc_prev_valid) {
        /* 第一帧只作基准：上电前轮子被挪动过，不能计入里程 */
        for (uint8_t i = 0U; i < PROTO_CHASSIS_MOTOR_MAX; i++) {
            s_enc_prev[i] = (int64_t)enc[i];
        }
        s_enc_prev_valid = true;
        return;
    }

    for (uint8_t i = 0U; i < PROTO_CHASSIS_MOTOR_MAX; i++) {
        /* 驱动板 total 是 32 位计数器，回绕（越过 ±21 亿）时数值会突变；
           在 int32 域相减，int32 / uint32 两种回绕都能还原出真实增量 */
        delta[i] = (int32_t)((uint32_t)enc[i] - (uint32_t)s_enc_prev[i]);
    }

    /* 电机正方向与"驱动底盘前进"相反，见 app_chassis_move() 下发速度时的取负 */
    for (uint8_t i = 0U; i < WHEEL_NUMBERS; i++) {
        wheel_travel[i] = -(float)delta[s_motor_of_wheel[i]] * APP_CHASSIS_MM_PER_COUNT;
    }

    alg_chassis_forward(wheel_travel, &displacement);
    step_mm = sqrtf((displacement.speed_x * displacement.speed_x) +
                    (displacement.speed_y * displacement.speed_y));

    /* 死区：静止时编码器 total 会上下跳几个计数，这种"位移"当噪声不结算。
       注意此时不推进基准 —— 位移先攒着，顶出死区后随下一段一起计入，计数不会丢 */
    if (step_mm < APP_CHASSIS_ODOM_DEADBAND_MM) {
        return;
    }

    s_distance_mm += step_mm;
    for (uint8_t i = 0U; i < PROTO_CHASSIS_MOTOR_MAX; i++) {
        s_enc_prev[i] = (int64_t)enc[i];
    }
}

/* 收到一整帧（主循环上下文）：只认驱动板主动上报的编码器帧 $MAll:m1,m2,m3,m4# */
static void app_chassis_on_frame(const char *text, uint16_t len)
{
    char    cmd[16];
    int32_t enc[PROTO_CHASSIS_MOTOR_MAX];
    uint8_t argc = 0U;

    UNUSED(len);        /* 协议层已给整帧补 '\0'，长度用不上 */

    if (proto_chassis_parse(text, cmd, (uint8_t)sizeof(cmd),
                            enc, PROTO_CHASSIS_MOTOR_MAX, &argc) != RET_OK) {
        return;         /* 回显的命令、含小数的帧等都不是编码器数据，忽略 */
    }
    if ((strcmp(cmd, PROTO_CHASSIS_CMD_MALL) != 0) || (argc != PROTO_CHASSIS_MOTOR_MAX)) {
        return;
    }

    app_chassis_update_odom(enc);
}

int app_chassis_init(UART_HandleTypeDef *huart)
{

    if (huart == NULL) {
        return RET_PARAM;
    }

    /* 1) 绑定串口（HAL_UART_Init 已由 CubeMX 生成的 MX_USART1_Init 完成） */
    if (bsp_uart_attach(BSP_UART_CHASSIS, huart) != RET_OK) {
        return RET_ERROR;
    }

    /* 2) 复位协议解析状态机，注册整帧回调（编码器上报帧从这里进来） */
    proto_chassis_init();
    proto_chassis_set_cb(app_chassis_on_frame);

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
    frame_len = proto_chassis_pack(PROTO_CHASSIS_CMD_UPLOAD, "1,0,0", frame, sizeof(frame));/* 请求上传编码器数据 */
    if(bsp_uart_send(BSP_UART_CHASSIS, (uint8_t *)frame, frame_len, 100U) != RET_OK) {
        return RET_ERROR;
    }/* 只上传编码器 total 值：回绕已在 app_chassis_update_odom() 里按 32 位处理 */
    /* 底盘数据初始化完成 */
    s_enc_prev_valid = false;       /* 里程从 0 开始，第一帧数据只作基准 */
    s_distance_mm    = 0.0f;

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
    /* BSP 先复制帧，再由中断发送；忙时不覆盖旧帧，返回 RET_BUSY。
     * 周期闭环下一拍发送最新目标，不积压过时的速度指令。 */
    return bsp_uart_send_it(BSP_UART_CHASSIS, (const uint8_t *)frame, frame_len);
}

void app_chassis_poll(void)
{
    uint8_t  buf[APP_CHASSIS_CHUNK_SIZE];
    uint16_t got;

    if (!s_inited) {
        return;
    }

    /* 一次性把缓冲里的数据搬空，避免逐字节调用的开销。
       每次读到的可能只有半帧，proto_chassis 内部有状态机会自己拼接。 */
    while ((got = bsp_uart_read(BSP_UART_CHASSIS, buf, (uint16_t)sizeof(buf))) > 0U) {
        proto_chassis_feed(buf, got);
    }
}

float app_chassis_get_distance(void)
{
    return s_distance_mm;
}

void app_chassis_reset_distance(void)
{
    s_distance_mm = 0.0f;
}


