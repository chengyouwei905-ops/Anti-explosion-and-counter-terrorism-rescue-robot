/**
  ******************************************************************************
  * @file    proto_roboarm.h
  * @brief   幻尔总线舵机控制板协议：《总线舵机控制板通信协议》的组帧
  * @note    本层只负责"参数 <-> 字节流"的转换，不知道机械臂该摆成什么姿势 ——
  *          那是 App / Algorithm 层的事。
  *
  *          帧格式（串口 9600-8-N-1，16 进制；本工程走 USART10：PE3 = TX / PE2 = RX）：
  *
  *              帧头          数据长度      指令      参数
  *              0x55 0x55     Length       Cmd       Prm1 … PrmN
  *
  *              Length      = N + 2     （N 是参数字节数：+1 指令、+1 Length 自己）
  *              整帧字节数  = N + 4     （帧头 2 + 长度 1 + 指令 1 + 参数 N）
  *
  *          手册示例（1 号舵机用 1000ms 转到 800 位置）：
  *              55 55 08 03 01 E8 03 01 20 03
  *              └帧头 ┘ 长 指令 └个数┘└时间 ┘└ID ┘└─位置─┘
  *
  *          ⚠️ 三个容易踩的点（和底盘 / 视觉链路都不一样）：
  *          1. 波特率是 **9600**，不是 115200；
  *          2. **没有校验和**，别照抄 proto_chassis / proto_vision 的习惯；
  *          3. Length 数的是"它自己 + 指令 + 参数"，**不数帧头** ——
  *             所以整帧 = Length + 2，而不是 + 3。
  *
  *          目前只实现手册"一、用户主动给控制板发送数据部分"（App 层只用这一半）。
  *          控制板主动上报（动作组结束等）和读指令的回包，等 App 真正要用时再加。
  ******************************************************************************
  */

#ifndef __PROTO_ROBOARM_H
#define __PROTO_ROBOARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "common_def.h"

/* ==========================================================================
 * 帧格式常量
 * ========================================================================== */

/** @brief 帧头字节：连续两个 0x55 表示一帧开始 */
#define PROTO_ROBOARM_SOF               0x55U

/** @brief 组帧固定开销：帧头(2) + 数据长度(1) + 指令(1)，不含参数 */
#define PROTO_ROBOARM_OVERHEAD          4U

/** @brief 单帧最大字节数（整帧，含帧头）。
  *        最长的一帧是"多舵机同步转动"：3×舵机数 + 7，六轴机械臂也才 25 字节。 */
#define PROTO_ROBOARM_MAX_FRAME         32U

/** @brief 一条多舵机指令最多能带几个舵机。
  *        参数 = 个数(1) + 时间(2) + 3×舵机数，整帧 = 3×舵机数 + 7，
  *        受 PROTO_ROBOARM_MAX_FRAME 限制 → (32 - 7) / 3 = 8 个。 */
#define PROTO_ROBOARM_MAX_SERVO         8U

/* ==========================================================================
 * 指令（手册"一、用户主动给控制板发送数据部分"）
 * ========================================================================== */

#define PROTO_ROBOARM_CMD_SERVO_MOVE            0x03U   /**< 控制任意个舵机转动 */
#define PROTO_ROBOARM_CMD_ACTION_GROUP_RUN      0x06U   /**< 运行动作组（次数 0 = 一直运行） */
#define PROTO_ROBOARM_CMD_ACTION_GROUP_STOP     0x07U   /**< 停止正在运行的动作组（无参数） */
#define PROTO_ROBOARM_CMD_ACTION_GROUP_SPEED    0x0BU   /**< 动作组速度百分比（编号 0xFF = 全部动作组） */
#define PROTO_ROBOARM_CMD_GET_BATTERY_VOLTAGE   0x0FU   /**< 读电池电压，单位 mV（无参数，控制板立刻回包） */
#define PROTO_ROBOARM_CMD_MULT_SERVO_UNLOAD     0x14U   /**< 多个舵机掉电卸力 */
#define PROTO_ROBOARM_CMD_MULT_SERVO_POS_READ   0x15U   /**< 读多个舵机的角度位置（控制板会回包） */

/** @brief 控制板主动上报的指令（接收方向，App 用到时再实现解析，这里仅备查） */
#define PROTO_ROBOARM_CMD_ACTION_GROUP_COMPLETE 0x08U   /**< 动作组自然运行结束 */

/** @brief 动作组"全部"通配编号，用于 PROTO_ROBOARM_CMD_ACTION_GROUP_SPEED */
#define PROTO_ROBOARM_GROUP_ALL                 0xFFU

/* ==========================================================================
 * 参数范围
 *
 * ⚠️ 控制板手册只规定了帧格式，没规定取值范围；下面这些是幻尔总线舵机
 *    （LX-16A / LX-224 / HTD-45H 等同协议型号）本身的规格，换型号改这里即可。
 * ========================================================================== */

/** @brief 舵机 ID 范围（总线理论最多挂 253 个） */
#define PROTO_ROBOARM_ID_MIN            0U
#define PROTO_ROBOARM_ID_MAX            253U

/** @brief 角度位置范围：0 ~ 1000 对应 0° ~ 240° */
#define PROTO_ROBOARM_POS_MIN           0U
#define PROTO_ROBOARM_POS_MAX           1000U

/** @brief 转动时间范围（ms）：0 = 不控速，立刻到位 */
#define PROTO_ROBOARM_TIME_MIN          0U
#define PROTO_ROBOARM_TIME_MAX          30000U

/** @brief 动作组速度百分比范围（100 = 原速） */
#define PROTO_ROBOARM_SPEED_PERCENT_MIN 0U
#define PROTO_ROBOARM_SPEED_PERCENT_MAX 1000U

/* ==========================================================================
 * 数据结构
 * ========================================================================== */

/** @brief 一个舵机的目标：ID + 角度位置
  * @note  只作为组帧的入参描述，**不会**被整体塞进帧里 ——
  *        帧里每个字段都是逐字节显式序列化出来的。 */
typedef struct {
    uint8_t  id;        /**< 舵机 ID，PROTO_ROBOARM_ID_MIN ~ PROTO_ROBOARM_ID_MAX */
    uint16_t position;  /**< 角度位置，PROTO_ROBOARM_POS_MIN ~ PROTO_ROBOARM_POS_MAX */
} proto_roboarm_servo_t;

/** @brief 组帧统计（"发给控制板没反应"时先看这里） */
typedef struct {
    uint32_t ok_frames;     /**< 成功组出的帧数 */
    uint32_t pack_errors;   /**< 参数非法 / 输出缓冲区不够而拒绝的次数 */
} proto_roboarm_stat_t;

/* ==========================================================================
 * 状态
 * ========================================================================== */

/** @brief 初始化（清空组帧统计） */
void    proto_roboarm_init(void);

/** @brief 获取组帧统计（只读） */
const proto_roboarm_stat_t *proto_roboarm_get_stat(void);

/* ==========================================================================
 * 组帧（发送方向）
 *
 * 所有 pack_xxx() 返回"整帧字节数"；返回 0 表示失败
 * （参数非法 / 超出范围 / 输出缓冲区不够），此时一个字节都不会写进 out。
 *
 * App 层的用法：组帧 → 交给 BSP 串口发出去，例如
 *     n = proto_roboarm_pack_servo_move(sv, 3U, 800U, buf, sizeof(buf));
 *     if (n > 0U) { bsp_uart_send(BSP_UART_ROBOARM, buf, n, 10U); }
 * ========================================================================== */

/**
  * @brief  通用组帧：55 55 <Length> <Cmd> <Prm...>
  * @param  cmd      指令，用 PROTO_ROBOARM_CMD_xxx
  * @param  prm      参数字节，可为 NULL（仅当 prm_len 为 0）
  * @param  prm_len  参数字节数 N
  * @param  out      输出缓冲区
  * @param  out_size out 的字节数（这里是二进制帧，不用给 '\0' 留位置）
  * @retval 整帧字节数（= prm_len + 4）；0 = 失败
  * @note   需要手册里没列出的指令时用它，其它情况优先用下面的专用函数。
  */
uint8_t proto_roboarm_pack(uint8_t cmd, const uint8_t *prm, uint8_t prm_len,
                           uint8_t *out, uint8_t out_size);

/**
  * @brief  多舵机同步转动：55 55 <3N+5> 03 <N> <tL> <tH> <ID1 pL1 pH1> ... <IDN pLN pHN>
  * @param  servos   舵机数组（ID + 目标位置）
  * @param  count    舵机个数，1 ~ PROTO_ROBOARM_MAX_SERVO
  * @param  time_ms  转动时间（ms），0 ~ PROTO_ROBOARM_TIME_MAX，0 表示立刻到位
  * @retval 整帧字节数（= 3×count + 7）；0 = 失败
  * @note   所有舵机共用同一个时间；手册示例 ①（1 号舵机 1000ms 转到 800）：
  *         servo = { .id = 1U, .position = 800U };
  *         proto_roboarm_pack_servo_move(&servo, 1U, 1000U, out, out_size)
  *             → 55 55 08 03 01 E8 03 01 20 03
  */
uint8_t proto_roboarm_pack_servo_move(const proto_roboarm_servo_t *servos, uint8_t count,
                                      uint16_t time_ms, uint8_t *out, uint8_t out_size);

/**
  * @brief  运行已下载到控制板的动作组
  * @param  group  动作组编号
  * @param  times  运行次数，0 = 无限次
  * @retval 整帧字节数（= 8）；0 = 失败
  */
uint8_t proto_roboarm_pack_action_group_run(uint8_t group, uint16_t times,
                                            uint8_t *out, uint8_t out_size);

/**
  * @brief  停止正在运行的动作组（本来没在运行也不受影响）
  * @retval 整帧字节数（= 6）；0 = 失败
  */
uint8_t proto_roboarm_pack_action_group_stop(uint8_t *out, uint8_t out_size);

/**
  * @brief  设置动作组运行速度（百分比，关机不保存）
  * @param  group    动作组编号，PROTO_ROBOARM_GROUP_ALL 表示调整全部动作组
  * @param  percent  速度百分比，100 = 原速
  * @retval 整帧字节数（= 8）；0 = 失败
  */
uint8_t proto_roboarm_pack_action_group_speed(uint8_t group, uint16_t percent,
                                              uint8_t *out, uint8_t out_size);

/**
  * @brief  读取控制板电池电压（控制板会立刻回一包，回包解析等 App 用到时再加）
  * @retval 整帧字节数（= 5）；0 = 失败
  */
uint8_t proto_roboarm_pack_get_battery_voltage(uint8_t *out, uint8_t out_size);

/**
  * @brief  多个舵机掉电卸力（卸力后可以用手随意转动）
  * @param  ids    舵机 ID 数组，每个都不得超过 PROTO_ROBOARM_ID_MAX
  * @param  count  舵机个数，1 ~ PROTO_ROBOARM_MAX_SERVO
  * @retval 整帧字节数（= count + 5）；0 = 失败
  */
uint8_t proto_roboarm_pack_servo_unload(const uint8_t *ids, uint8_t count,
                                        uint8_t *out, uint8_t out_size);

/**
  * @brief  读取多个舵机的角度位置（控制板会回一包，回包解析等 App 用到时再加）
  * @param  ids    舵机 ID 数组，每个都不得超过 PROTO_ROBOARM_ID_MAX
  * @param  count  舵机个数，1 ~ PROTO_ROBOARM_MAX_SERVO
  * @retval 整帧字节数（= count + 5）；0 = 失败
  */
uint8_t proto_roboarm_pack_servo_pos_read(const uint8_t *ids, uint8_t count,
                                          uint8_t *out, uint8_t out_size);

#ifdef __cplusplus
}
#endif

#endif /* __PROTO_ROBOARM_H */
