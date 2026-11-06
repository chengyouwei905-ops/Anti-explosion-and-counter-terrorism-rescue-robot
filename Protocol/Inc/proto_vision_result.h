#ifndef PROTO_VISION_RESULT_H
#define PROTO_VISION_RESULT_H

#include "proto_vision.h"
#include <stddef.h>

/* 版本1结果帧。扫描请求4字节：[0=沿用QR目标, 停车位1..3, 窗口ms低字节, 高字节]。
 * QR/定位请求为空载荷；QR成功结果为15字节无坐标ACK，任务码保存在K230。
 * 扫描成功返回当前位，失败reason=20表示该位非目标。
 * 取消命令仅保留协议编号，MCU应用层禁止发送。
 */
enum {
    VISION_CMD_QR = 0x10,
    VISION_CMD_BOMB_SCAN = 0x20,
    VISION_CMD_BOMB_LOCATE = 0x21,
    VISION_CMD_TARGET_SCAN = 0x30,
    VISION_CMD_TARGET_LOCATE = 0x31,
    VISION_CMD_RESCUE_SCAN = 0x40,
    VISION_CMD_RESCUE_LOCATE = 0x41,
    VISION_CMD_CANCEL = 0x7F,
    VISION_CMD_RESULT = 0x81
};
enum {
    VISION_SUCCESS = 0, VISION_FAILURE = 1,
    VISION_CANCELLED = 2, VISION_OBSERVATION = 3
};
enum { VISION_REASON_SCAN_NOT_TARGET = 20 };
/* 4字节停车扫描请求中的16位窗口；与K230 protocol.py保持一致。 */
enum { VISION_SCAN_WINDOW_MIN_MS = 250, VISION_SCAN_WINDOW_MAX_MS = 60000 };
/* 与K230 MeasurementAverage允许的最大采样数一致。 */
enum { VISION_MEASUREMENT_MAX_SAMPLES = 32 };
/* 与2026-11-06 09点K230 runtime_config.MAX_POSITION_SCATTER_MM对齐。 */
#define VISION_MEASUREMENT_MAX_SCATTER_MM 20.0f
enum {
    VISION_POSITION = 0x01, VISION_CONTROL_READY = 0x02,
    VISION_QUATERNION = 0x04, VISION_NORMAL = 0x08,
    VISION_GRASP = 0x10, VISION_CANDIDATE = 0x80
};

typedef struct {
    uint8_t task_cmd, input_seq, status, flags;
    uint8_t slot_id, sample_count, reason;
    uint16_t run_id;
    uint32_t frame_id;
    float xyz_mm[3];
    float quaternion_wxyz[4];
    float normal[3];
    float grasp_mm[3];
    float scatter_mm;
} VisionResult_t;

bool proto_vision_is_scan(uint8_t cmd);
bool proto_vision_is_measure(uint8_t cmd);
/* true 仅表示结构合法，失败/取消包也能解析成功；控制许可另行检查。 */
bool proto_vision_decode_result(const uint8_t *payload, size_t len, VisionResult_t *out);
/* QR只读任务码完成：15字节SUCCESS回包，flags/slot/sample_count/reason均为0。 */
bool proto_vision_result_qr_ready(const VisionResult_t *result);
/* 接受min_samples..32帧有效测量；min_samples为下限，不要求恰好相等。 */
bool proto_vision_result_control_ready(const VisionResult_t *result,
                                      uint8_t min_samples, float max_scatter_mm);

#endif
