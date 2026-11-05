/** @brief HWT101 TTL串口协议；纯字节解析，不依赖HAL。 */
/* 55 53 00 00 00 00 YawL YawH VL VH SUM；SUM包含版本号字节。 */
#ifndef __PROTO_HWT101_H
#define __PROTO_HWT101_H

#include <stdint.h>

#define PROTO_HWT101_FRAME_SIZE 11U
#define PROTO_HWT101_SOF        0x55U
#define PROTO_HWT101_ANGLE      0x53U

/* yaw为有符号小端原始值，yaw / 32768 * 180度。回调在feed调用上下文执行。 */
typedef void (*proto_hwt101_yaw_cb_t)(int16_t yaw);

void proto_hwt101_init(proto_hwt101_yaw_cb_t cb);
/* 只丢弃半帧，保留回调；丢字节/长时间停顿后使用。 */
void proto_hwt101_reset_rx(void);
/* 只读取55 53角度帧，返回本次解出的有效角度帧数。 */
uint16_t proto_hwt101_feed(const uint8_t *data, uint16_t len);

#endif /* __PROTO_HWT101_H */
