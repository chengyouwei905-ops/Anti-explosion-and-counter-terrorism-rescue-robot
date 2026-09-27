/**
  ******************************************************************************
  * @file    proto_chassis.c
  * @brief   底盘驱动板文本协议实现
  * @note    与 proto_vision.c 的区别：
  *          二进制帧必须"逐字段推进状态"（长度、序号、载荷、CRC 各一个状态），
  *          而文本协议的字段是靠分隔符界定的 —— 所以只要先把整帧攒齐
  *          （从 '$' 到 '#'），再一次性切分就够了，状态机只有两个状态。
  ******************************************************************************
  */

#include "proto_chassis.h"
#include <stddef.h>
#include <string.h>

/** @brief 解包状态机状态 */
typedef enum {
    PC_IDLE = 0,    /**< 等待帧起始符 '$' */
    PC_BODY,        /**< 攒帧中（'$' 与 '#' 之间的内容） */
} proto_chassis_state_t;

static struct {
    proto_chassis_state_t    state;
    char                     buf[PROTO_CHASSIS_MAX_FRAME + 1U]; /**< +1 用于放结尾 '\0' */
    uint16_t                 idx;                               /**< 已写入的字符数 */
    proto_chassis_frame_cb_t cb;
    proto_chassis_stat_t     stat;
} s_ctx;

/* ==========================================================================
 * 组帧用的小工具
 * 自己实现整数转字符串，不依赖 printf/snprintf —— 避免链接整个格式化库
 * ========================================================================== */

typedef struct {
    char    *buf;
    uint8_t  cap;       /**< 可写字符数（已扣掉结尾 '\0' 的位置） */
    uint8_t  len;
    bool     overflow;  /**< 写不下时置位，最终返回 0 */
} pc_writer_t;

static bool pc_begin(pc_writer_t *w, char *out, uint8_t out_size)
{
    if ((out == NULL) || (out_size < 4U)) {     /* 最少也要装下 "$x#" + '\0' */
        return false;
    }
    w->buf      = out;
    w->cap      = (uint8_t)(out_size - 1U);
    w->len      = 0U;
    w->overflow = false;
    return true;
}

static void pc_char(pc_writer_t *w, char c)
{
    if (w->len < w->cap) {
        w->buf[w->len] = c;
        w->len++;
    } else {
        w->overflow = true;
    }
}

static void pc_str(pc_writer_t *w, const char *s)
{
    while (*s != '\0') {
        pc_char(w, *s);
        s++;
    }
}

/** @brief 写一个有符号十进制整数 */
static void pc_i32(pc_writer_t *w, int32_t v)
{
    char     tmp[12];       /* uint32 最多 10 位数字，够用 */
    uint8_t  n   = 0U;
    bool     neg = (v < 0);
    /* 用 int64 取绝对值，避开 v == INT32_MIN 时取负溢出的未定义行为 */
    uint32_t u   = neg ? (uint32_t)(-(int64_t)v) : (uint32_t)v;

    do {
        tmp[n] = (char)('0' + (char)(u % 10U));
        n++;
        u /= 10U;
    } while ((u > 0U) && (n < (uint8_t)sizeof(tmp)));

    if (neg) {
        pc_char(w, '-');
    }
    while (n > 0U) {
        n--;
        pc_char(w, tmp[n]);
    }
}

/** @brief 收尾：补 '\0' 并返回帧长（不含 '\0'）；写不下则返回 0 */
static uint8_t pc_finish(pc_writer_t *w)
{
    if (w->overflow) {
        return 0U;
    }
    w->buf[w->len] = '\0';      /* len <= cap < out_size，安全 */
    return w->len;
}

/* ==========================================================================
 * 状态与回调
 * ========================================================================== */

void proto_chassis_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = PC_IDLE;
}

void proto_chassis_set_cb(proto_chassis_frame_cb_t cb)
{
    s_ctx.cb = cb;
}

const proto_chassis_stat_t *proto_chassis_get_stat(void)
{
    return &s_ctx.stat;
}

/* ==========================================================================
 * 组帧（发送方向）
 * ========================================================================== */

uint8_t proto_chassis_pack(const char *cmd, const char *params,
                           char *out, uint8_t out_size)
{
    pc_writer_t w;

    if ((cmd == NULL) || (!pc_begin(&w, out, out_size))) {
        return 0U;
    }

    pc_char(&w, (char)PROTO_CHASSIS_SOF);
    pc_str(&w, cmd);
    if (params != NULL) {
        pc_char(&w, (char)PROTO_CHASSIS_SEP);
        pc_str(&w, params);
    }
    pc_char(&w, (char)PROTO_CHASSIS_EOF);

    return pc_finish(&w);
}

uint8_t proto_chassis_pack_i32(const char *cmd, const int32_t *args, uint8_t argc,
                               char *out, uint8_t out_size)
{
    pc_writer_t w;

    if ((cmd == NULL) || (!pc_begin(&w, out, out_size))) {
        return 0U;
    }
    if ((argc > 0U) && (args == NULL)) {
        return 0U;
    }

    pc_char(&w, (char)PROTO_CHASSIS_SOF);
    pc_str(&w, cmd);
    if (argc > 0U) {
        pc_char(&w, (char)PROTO_CHASSIS_SEP);
        for (uint8_t i = 0U; i < argc; i++) {
            if (i > 0U) {
                pc_char(&w, (char)PROTO_CHASSIS_COMMA);
            }
            pc_i32(&w, args[i]);
        }
    }
    pc_char(&w, (char)PROTO_CHASSIS_EOF);

    return pc_finish(&w);
}

/** @brief 四个电机值的公共组帧逻辑（含范围检查） */
static uint8_t pack_motors(const char *cmd, const int16_t v[PROTO_CHASSIS_MOTOR_MAX],
                           int32_t min, int32_t max, char *out, uint8_t out_size)
{
    int32_t args[PROTO_CHASSIS_MOTOR_MAX];

    if (v == NULL) {
        return 0U;
    }
    for (uint8_t i = 0U; i < PROTO_CHASSIS_MOTOR_MAX; i++) {
        if ((v[i] < min) || (v[i] > max)) {
            return 0U;      /* 超出驱动板允许范围：直接拒绝，别把非法值发出去 */
        }
        args[i] = (int32_t)v[i];
    }

    return proto_chassis_pack_i32(cmd, args, PROTO_CHASSIS_MOTOR_MAX, out, out_size);
}

uint8_t proto_chassis_pack_spd(const int16_t spd[PROTO_CHASSIS_MOTOR_MAX],
                               char *out, uint8_t out_size)
{
    return pack_motors(PROTO_CHASSIS_CMD_SPD, spd,
                       PROTO_CHASSIS_SPD_MIN, PROTO_CHASSIS_SPD_MAX, out, out_size);
}

uint8_t proto_chassis_pack_pwm(const int16_t pwm[PROTO_CHASSIS_MOTOR_MAX],
                               char *out, uint8_t out_size)
{
    return pack_motors(PROTO_CHASSIS_CMD_PWM, pwm,
                       PROTO_CHASSIS_PWM_MIN, PROTO_CHASSIS_PWM_MAX, out, out_size);
}

uint8_t proto_chassis_pack_upload(uint8_t total, uint8_t realtime, uint8_t speed,
                                  char *out, uint8_t out_size)
{
    const int32_t args[3] = { (int32_t)total, (int32_t)realtime, (int32_t)speed };

    return proto_chassis_pack_i32(PROTO_CHASSIS_CMD_UPLOAD, args, 3U, out, out_size);
}

/* ==========================================================================
 * 解帧（接收方向）
 * ========================================================================== */

void proto_chassis_feed(const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return;
    }

    for (uint16_t i = 0U; i < len; i++) {
        char c = (char)data[i];

        s_ctx.stat.rx_bytes++;

        switch (s_ctx.state) {
        case PC_IDLE:
            if (c == (char)PROTO_CHASSIS_SOF) {
                s_ctx.buf[0] = c;
                s_ctx.idx    = 1U;
                s_ctx.state  = PC_BODY;
            }
            /* 其余字节直接丢弃：上电抖动、线路噪声都可能产生杂散字符 */
            break;

        case PC_BODY:
            if (c == (char)PROTO_CHASSIS_SOF) {
                /* 又出现 '$'：说明上一帧没等到 '#'（丢了），丢弃重来 */
                s_ctx.buf[0] = c;
                s_ctx.idx    = 1U;
                break;
            }

            if (c == (char)PROTO_CHASSIS_EOF) {
                if (s_ctx.idx < 2U) {
                    /* 形如 "$#" 的空帧，判为垃圾 */
                    s_ctx.stat.frame_errors++;
                    s_ctx.state = PC_IDLE;
                    break;
                }
                /* 把结束符 '#' 也一起交给上层，回调拿到的是"完整帧"：
                   "$upload:1,2,3#" —— 这样可以直接喂给 proto_chassis_parse() */
                s_ctx.buf[s_ctx.idx] = c;
                s_ctx.idx++;
                s_ctx.buf[s_ctx.idx] = '\0';
                s_ctx.stat.ok_frames++;
                s_ctx.state = PC_IDLE;
                if (s_ctx.cb != NULL) {
                    s_ctx.cb(s_ctx.buf, (uint16_t)s_ctx.idx);
                }
                break;
            }

            if (s_ctx.idx >= (PROTO_CHASSIS_MAX_FRAME - 1U)) {
                /* 再加一个字符就放不下结束符 '#' 了 → 这帧肯定超长，丢弃 */
                s_ctx.stat.frame_errors++;
                s_ctx.state = PC_IDLE;
                break;
            }

            s_ctx.buf[s_ctx.idx] = c;
            s_ctx.idx++;
            break;

        default:
            s_ctx.state = PC_IDLE;
            break;
        }
    }
}

int proto_chassis_parse(const char *text, char *cmd, uint8_t cmd_size,
                        int32_t *args, uint8_t arg_max, uint8_t *argc)
{
    const char *p  = text;
    uint8_t     ci = 0U;
    uint8_t     ai = 0U;

    if ((text == NULL) || (cmd == NULL) || (cmd_size < 2U)) {
        return RET_PARAM;
    }
    if (argc != NULL) {
        *argc = 0U;
    }

    if (*p != (char)PROTO_CHASSIS_SOF) {
        return RET_ERROR;
    }
    p++;

    /* ---- 1) 取命令名：直到 ':' / '#' / 字符串结尾 ---- */
    while ((*p != '\0') &&
           (*p != (char)PROTO_CHASSIS_SEP) &&
           (*p != (char)PROTO_CHASSIS_EOF)) {
        if (ci >= (uint8_t)(cmd_size - 1U)) {
            return RET_NOMEM;           /* 命令名太长，装不下 */
        }
        cmd[ci] = *p;
        ci++;
        p++;
    }
    cmd[ci] = '\0';

    /* ---- 2) 取参数（可选） ---- */
    if (*p == (char)PROTO_CHASSIS_SEP) {
        p++;
        while ((*p != '\0') && (*p != (char)PROTO_CHASSIS_EOF)) {
            bool    neg       = false;
            bool    has_digit = false;
            int32_t val       = 0;

            if (*p == '-') {
                neg = true;
                p++;
            }
            while ((*p >= '0') && (*p <= '9')) {
                val = (val * 10) + (int32_t)(*p - '0');
                has_digit = true;
                p++;
            }
            if (!has_digit) {
                /* 空字段 / 字母 / 小数点都会落到这里 */
                return RET_ERROR;
            }
            if ((args == NULL) || (ai >= arg_max)) {
                return RET_NOMEM;       /* 参数太多或没给数组 */
            }
            args[ai] = neg ? -val : val;
            ai++;

            if (*p == (char)PROTO_CHASSIS_COMMA) {
                p++;
                continue;
            }
            break;
        }
    }

    /* ---- 3) 必须正好停在 '#' 或字符串结尾 ---- */
    if ((*p != '\0') && (*p != (char)PROTO_CHASSIS_EOF)) {
        /* 还有没吃掉的字符，典型例子：$MPID:1,5,0.03,0.1# 里的小数点。
           宁可报错，也不返回被截断的错误数值。 */
        return RET_ERROR;
    }

    if (argc != NULL) {
        *argc = ai;
    }
    return RET_OK;
}
