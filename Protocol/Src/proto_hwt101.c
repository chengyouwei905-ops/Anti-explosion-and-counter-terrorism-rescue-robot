#include "proto_hwt101.h"
#include "common_def.h"
#include <string.h>

static struct {
    uint8_t frame[PROTO_HWT101_FRAME_SIZE];
    uint8_t used;
    proto_hwt101_yaw_cb_t cb;
} s_ctx;

void proto_hwt101_init(proto_hwt101_yaw_cb_t cb)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.cb = cb;
}

void proto_hwt101_reset_rx(void)
{
    s_ctx.used = 0U;
}

uint16_t proto_hwt101_feed(const uint8_t *data, uint16_t len)
{
    uint16_t frames = 0U;
    if (data == NULL) {
        return 0U;
    }
    for (uint16_t i = 0U; i < len; i++) {
        if ((s_ctx.used == 0U) && (data[i] != PROTO_HWT101_SOF)) {
            continue;
        }
        s_ctx.frame[s_ctx.used++] = data[i];
        if ((s_ctx.used == 2U) && (s_ctx.frame[1] != PROTO_HWT101_ANGLE)) {
            s_ctx.used = (data[i] == PROTO_HWT101_SOF) ? 1U : 0U;
            continue;
        }
        if (s_ctx.used < PROTO_HWT101_FRAME_SIZE) {
            continue;
        }
        uint8_t sum = 0U;
        for (uint8_t k = 0U; k < PROTO_HWT101_FRAME_SIZE - 1U; k++) {
            sum = (uint8_t)(sum + s_ctx.frame[k]);
        }
        if (sum == s_ctx.frame[PROTO_HWT101_FRAME_SIZE - 1U]) {
            frames++;
            if (s_ctx.cb != NULL) {
                s_ctx.cb(le_i16_get(&s_ctx.frame[6]));
            }
            s_ctx.used = 0U;
        } else {
            /* 校验失败只跳到下一个55 53，保留可能已经收到的下一帧。
             * 不在载荷出现55时提前重启：有效角度数据也可能包含55。 */
            uint8_t skip = 1U;
            while (skip < s_ctx.used) {
                if ((s_ctx.frame[skip] == PROTO_HWT101_SOF) &&
                    ((skip == s_ctx.used - 1U) || (s_ctx.frame[skip + 1U] == PROTO_HWT101_ANGLE))) {
                    break;
                }
                skip++;
            }
            s_ctx.used = (uint8_t)(s_ctx.used - skip);
            memmove(s_ctx.frame, s_ctx.frame + skip, s_ctx.used);
        }
    }
    return frames;
}
