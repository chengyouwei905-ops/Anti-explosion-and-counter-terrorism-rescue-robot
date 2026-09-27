/**
  ******************************************************************************
  * @file    proto_chassis.h
  * @brief   底盘四电机驱动板串口协议：文本命令 $<命令>[:<参数>]# 的组帧与解帧
  * @note    本层只负责"字符串 <-> 帧"的转换，不知道速度该给多少、车该往哪走 ——
  *          那是 App / Algorithm 层的事。
  *
  *          帧格式（与视觉链路的二进制帧完全不同，注意区分）：
  *
  *              $<命令>[:<参数1>,<参数2>,...]#
  *
  *          例：  $spd:100,-100,0,50#     四电机速度
  *                $flash_reset#           无参数命令
  ******************************************************************************
  */

#ifndef __PROTO_CHASSIS_H
#define __PROTO_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "common_def.h"

/* ==========================================================================
 * 帧格式常量
 * ========================================================================== */

#define PROTO_CHASSIS_SOF           '$'     /**< 帧起始符 */
#define PROTO_CHASSIS_EOF           '#'     /**< 帧结束符 */
#define PROTO_CHASSIS_SEP           ':'     /**< 命令与参数的分隔符 */
#define PROTO_CHASSIS_COMMA         ','     /**< 参数之间的分隔符 */

/** @brief 单帧最大长度（含 '$' 和 '#'，不含结尾 '\0'）。
  *        实测最长命令 $spd:100,-100,0,50# 只有 19 字节，64 足够。 */
#define PROTO_CHASSIS_MAX_FRAME     64U

/** @brief 电机数量（板上 M1~M4） */
#define PROTO_CHASSIS_MOTOR_MAX     4U

/** @brief 速度范围（驱动板手册规定的硬限制，超范围会被驱动板忽略） */
#define PROTO_CHASSIS_SPD_MIN       (-1000)
#define PROTO_CHASSIS_SPD_MAX       (1000)

/** @brief PWM 范围 */
#define PROTO_CHASSIS_PWM_MIN       (-3600)
#define PROTO_CHASSIS_PWM_MAX       (3600)

/* ==========================================================================
 * 命令字（必须与驱动板手册逐字一致，包括手册里的拼写）
 * ========================================================================== */

#define PROTO_CHASSIS_CMD_MTYPE         "mtype"         /**< 电机型号：1=520 2=310 3=TT(带编码器) 4=TT(无编码器) */
#define PROTO_CHASSIS_CMD_DEADZONE      "deadzone"      /**< 死区，范围 0~3600 */
#define PROTO_CHASSIS_CMD_MLINE         "mline"         /**< 电机相线数（减速比参数） */
#define PROTO_CHASSIS_CMD_WDIAMETER     "wdiameter"     /**< 轮径 */
#define PROTO_CHASSIS_CMD_MPID          "MPID"          /**< PID 参数（含小数，见 proto_chassis_parse 的说明） */
#define PROTO_CHASSIS_CMD_FLASH_RESET   "flash_reset"   /**< 恢复出厂设置（无参数） */
#define PROTO_CHASSIS_CMD_SPD           "spd"           /**< 四电机速度控制，范围 -1000~1000 */
#define PROTO_CHASSIS_CMD_PWM           "pwm"           /**< 四电机 PWM 控制，范围 -3600~3600 */
#define PROTO_CHASSIS_CMD_UPLOAD        "upload"        /**< 上传编码器数据 */
#define PROTO_CHASSIS_CMD_READ_FLASH    "read_flash"    /**< 查询 flash 变量（无参数） */
#define PROTO_CHASSIS_CMD_READ_VOLTAGE  "read_vlolt"    /**< 查电池电压（无参数）⚠️ 手册原文拼写就是 vlolt，别"纠正" */

/** @brief 运行统计 */
typedef struct {
    uint32_t rx_bytes;      /**< 累计接收字节数 */
    uint32_t ok_frames;     /**< 成功解析出的帧数 */
    uint32_t frame_errors;  /**< 超长 / 格式非法而丢弃的帧数 */
} proto_chassis_stat_t;

/** @brief 收到一整条命令（以 '#' 结尾）时的回调
  * @param  text 以 '\0' 结尾的完整帧，如 "$upload:1,0,0#"（'\0' 不计入 len）
  * @param  len  帧长度（不含 '\0'）
  * @note   在主循环上下文执行，可做业务处理
  */
typedef void (*proto_chassis_frame_cb_t)(const char *text, uint16_t len);

/* ==========================================================================
 * 状态与回调
 * ========================================================================== */

/** @brief 初始化解析器（清空状态机与统计） */
void    proto_chassis_init(void);

/** @brief 注册"收到一整帧"的回调；传 NULL 取消注册 */
void    proto_chassis_set_cb(proto_chassis_frame_cb_t cb);

/**
  * @brief  向解析器喂入串口收到的原始字节（支持半帧 / 粘包）
  * @param  data 字节流
  * @param  len  字节数
  */
void    proto_chassis_feed(const uint8_t *data, uint16_t len);

/** @brief 获取统计信息（只读） */
const proto_chassis_stat_t *proto_chassis_get_stat(void);

/* ==========================================================================
 * 组帧（发送方向）
 *
 * 所有 pack_xxx() 返回"不含结尾 '\0' 的帧长度"；返回 0 表示失败
 * （参数非法 / 超出范围 / 输出缓冲区不够）。
 * ⚠️ out 里装的是 ASCII 字符，发送时请转成 (const uint8_t *)。
 * ========================================================================== */

/**
  * @brief  通用组帧：$<cmd>[:<params>]#
  * @param  cmd      命令字，用 PROTO_CHASSIS_CMD_xxx 宏
  * @param  params   参数字符串，可为 NULL（无参数命令）。
  *                  需要小数的命令（如 MPID）用这个版本传原始字符串。
  * @param  out      输出缓冲区
  * @param  out_size out 的字节数（要留出结尾 '\0' 的位置）
  * @retval 帧长度（不含 '\0'）；0 = 失败
  */
uint8_t proto_chassis_pack(const char *cmd, const char *params,
                           char *out, uint8_t out_size);

/**
  * @brief  整数数组组帧：$<cmd>:<a1>,<a2>,...#
  * @param  args 参数数组
  * @param  argc 参数个数
  * @retval 帧长度（不含 '\0'）；0 = 失败
  */
uint8_t proto_chassis_pack_i32(const char *cmd, const int32_t *args, uint8_t argc,
                               char *out, uint8_t out_size);

/**
  * @brief  四电机速度命令：$spd:m1,m2,m3,m4#
  * @param  spd 四个电机的目标速度，范围 PROTO_CHASSIS_SPD_MIN ~ MAX
  * @retval 帧长度；0 = 失败（有值超出范围）
  */
uint8_t proto_chassis_pack_spd(const int16_t spd[PROTO_CHASSIS_MOTOR_MAX],
                               char *out, uint8_t out_size);

/**
  * @brief  四电机 PWM 命令：$pwm:m1,m2,m3,m4#
  * @param  pwm 四个电机的 PWM 值，范围 PROTO_CHASSIS_PWM_MIN ~ MAX
  * @retval 帧长度；0 = 失败（有值超出范围）
  */
uint8_t proto_chassis_pack_pwm(const int16_t pwm[PROTO_CHASSIS_MOTOR_MAX],
                               char *out, uint8_t out_size);

/**
  * @brief  请求驱动板上传编码器数据：$upload:a,b,c#
  * @param  total    是否上传"累计编码器值"（0/1）
  * @param  realtime 是否上传"实时编码器值"（0/1）
  * @param  speed    是否上传"速度值"（0/1）
  * @retval 帧长度；0 = 失败
  * @note   手册 Command 列写的是 4 个 0，Example 列写的是 3 个 0，这里按 3 个实现。
  *         若实际板子要 4 个，改用 proto_chassis_pack_i32()。
  */
uint8_t proto_chassis_pack_upload(uint8_t total, uint8_t realtime, uint8_t speed,
                                  char *out, uint8_t out_size);

/* ==========================================================================
 * 解帧（接收方向）
 * ========================================================================== */

/**
  * @brief  把 "$<cmd>:<a1>,<a2>#" 拆成命令名 + 整数参数
  * @param  text     待解析的帧（以 '\0' 结尾）
  * @param  cmd      输出：命令名，如 "upload"
  * @param  cmd_size cmd 缓冲区大小（要留出结尾 '\0' 的位置）
  * @param  args     输出：参数数组（可为 NULL，此时只取命令名）
  * @param  arg_max  args 数组容量
  * @param  argc     输出：实际参数个数（可为 NULL）
  * @retval RET_OK / RET_PARAM / RET_NOMEM / RET_ERROR
  *
  * @note   ⚠️ 只支持**整数**参数。含小数的命令（如 $MPID:1,5,0.03,0.1#）
  *         会返回 RET_ERROR，而不是给出被截断的错误数值 ——
  *         这类命令目前只需要发送（用 proto_chassis_pack 传字符串），
  *         接收解析等 App 层真正用到时再加。
  */
int proto_chassis_parse(const char *text, char *cmd, uint8_t cmd_size,
                        int32_t *args, uint8_t arg_max, uint8_t *argc);

#ifdef __cplusplus
}
#endif

#endif /* __PROTO_CHASSIS_H */
