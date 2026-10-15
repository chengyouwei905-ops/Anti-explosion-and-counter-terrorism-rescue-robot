/**
  ******************************************************************************
  * @file    proto_roboarm.c
  * @brief   幻尔总线舵机控制板协议实现（组帧，无校验和）
  * @note    帧格式见 proto_roboarm.h。和 proto_vision.c 那种"流式状态机"不同，
  *          控制板这一侧目前只用发送 —— 每调一次 pack_xxx() 就是独立的一帧，
  *          不需要保存任何跨调用的状态，所以这里只有统计量。
  ******************************************************************************
  */

#include "proto_roboarm.h"
#include <stddef.h>
#include <string.h>

static struct {
    proto_roboarm_stat_t stat;
} s_ctx;

/* ==========================================================================
 * 状态
 * ========================================================================== */

void proto_roboarm_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
}

const proto_roboarm_stat_t *proto_roboarm_get_stat(void)
{
    return &s_ctx.stat;
}

/** @brief 所有组帧失败都从这里出去：记一次错并返回 0 */
static uint8_t pack_fail(void)
{
    s_ctx.stat.pack_errors++;
    return 0U;
}

/* ==========================================================================
 * 组帧（发送方向）
 * ========================================================================== */

uint8_t proto_roboarm_pack(uint8_t cmd, const uint8_t *prm, uint8_t prm_len,
                           uint8_t *out, uint8_t out_size)
{
    /* 先按 32 位算长度：uint8_t 相加溢出的话，下面的范围检查就形同虚设了 */
    uint32_t total = (uint32_t)prm_len + PROTO_ROBOARM_OVERHEAD;

    if ((out == NULL) || ((prm_len > 0U) && (prm == NULL))) {
        return pack_fail();
    }
    if (((uint32_t)out_size < total) || (total > PROTO_ROBOARM_MAX_FRAME)) {
        return pack_fail();
    }

    out[0] = PROTO_ROBOARM_SOF;
    out[1] = PROTO_ROBOARM_SOF;
    out[2] = (uint8_t)(prm_len + 2U);   /* Length = 参数个数 + 指令 + 它自己 */
    out[3] = cmd;
    if (prm_len > 0U) {
        memcpy(&out[PROTO_ROBOARM_OVERHEAD], prm, prm_len);
    }

    s_ctx.stat.ok_frames++;
    return (uint8_t)total;
}

/** @brief 一批舵机 ID 是否都在合法范围内 */
static bool ids_ok(const uint8_t *ids, uint8_t count)
{
    for (uint8_t i = 0U; i < count; i++) {
        if (ids[i] > (uint8_t)PROTO_ROBOARM_ID_MAX) {
            return false;
        }
    }
    return true;
}

uint8_t proto_roboarm_pack_servo_move(const proto_roboarm_servo_t *servos, uint8_t count,
                                      uint16_t time_ms, uint8_t *out, uint8_t out_size)
{
    uint8_t prm[3U + (3U * PROTO_ROBOARM_MAX_SERVO)];   /* 个数 + 时间 + 每个舵机 3 字节 */
    uint8_t n = 0U;

    if ((servos == NULL) || (count == 0U) ||
        (count > (uint8_t)PROTO_ROBOARM_MAX_SERVO) ||
        (time_ms > (uint16_t)PROTO_ROBOARM_TIME_MAX)) {
        return pack_fail();
    }

    prm[n] = count;
    n++;
    le_u16_put(&prm[n], time_ms);
    n = (uint8_t)(n + 2U);

    for (uint8_t i = 0U; i < count; i++) {
        /* 超出舵机能转的范围就整帧拒绝：宁可这一帧不发，也别让舵机硬顶到机械限位 */
        if ((servos[i].id > (uint8_t)PROTO_ROBOARM_ID_MAX) ||
            (servos[i].position > (uint16_t)PROTO_ROBOARM_POS_MAX)) {
            return pack_fail();
        }
        prm[n] = servos[i].id;
        n++;
        le_u16_put(&prm[n], servos[i].position);
        n = (uint8_t)(n + 2U);
    }

    return proto_roboarm_pack(PROTO_ROBOARM_CMD_SERVO_MOVE, prm, n, out, out_size);
}

uint8_t proto_roboarm_pack_action_group_run(uint8_t group, uint16_t times,
                                            uint8_t *out, uint8_t out_size)
{
    uint8_t prm[3];                  /* 组号 + 次数(低/高) */

    prm[0] = group;
    le_u16_put(&prm[1], times);

    return proto_roboarm_pack(PROTO_ROBOARM_CMD_ACTION_GROUP_RUN, prm, 3U, out, out_size);
}

uint8_t proto_roboarm_pack_action_group_stop(uint8_t *out, uint8_t out_size)
{
    return proto_roboarm_pack(PROTO_ROBOARM_CMD_ACTION_GROUP_STOP, NULL, 0U, out, out_size);
}

uint8_t proto_roboarm_pack_action_group_speed(uint8_t group, uint16_t percent,
                                              uint8_t *out, uint8_t out_size)
{
    uint8_t prm[3];                  /* 组号 + 百分比(低/高) */

    if (percent > (uint16_t)PROTO_ROBOARM_SPEED_PERCENT_MAX) {
        return pack_fail();
    }

    prm[0] = group;
    le_u16_put(&prm[1], percent);

    return proto_roboarm_pack(PROTO_ROBOARM_CMD_ACTION_GROUP_SPEED, prm, 3U, out, out_size);
}

uint8_t proto_roboarm_pack_get_battery_voltage(uint8_t *out, uint8_t out_size)
{
    return proto_roboarm_pack(PROTO_ROBOARM_CMD_GET_BATTERY_VOLTAGE, NULL, 0U, out, out_size);
}

/** @brief "掉电卸力"和"读角度位置"的指令格式一样：个数 + N 个 ID */
static uint8_t pack_servo_ids(uint8_t cmd, const uint8_t *ids, uint8_t count,
                              uint8_t *out, uint8_t out_size)
{
    uint8_t prm[1U + PROTO_ROBOARM_MAX_SERVO];

    if ((ids == NULL) || (count == 0U) ||
        (count > (uint8_t)PROTO_ROBOARM_MAX_SERVO) || (!ids_ok(ids, count))) {
        return pack_fail();
    }

    prm[0] = count;
    memcpy(&prm[1], ids, count);     /* ID 就是一串字节，没有对齐问题 */

    return proto_roboarm_pack(cmd, prm, (uint8_t)(count + 1U), out, out_size);
}

uint8_t proto_roboarm_pack_servo_unload(const uint8_t *ids, uint8_t count,
                                        uint8_t *out, uint8_t out_size)
{
    return pack_servo_ids(PROTO_ROBOARM_CMD_MULT_SERVO_UNLOAD, ids, count, out, out_size);
}

uint8_t proto_roboarm_pack_servo_pos_read(const uint8_t *ids, uint8_t count,
                                          uint8_t *out, uint8_t out_size)
{
    return pack_servo_ids(PROTO_ROBOARM_CMD_MULT_SERVO_POS_READ, ids, count, out, out_size);
}
