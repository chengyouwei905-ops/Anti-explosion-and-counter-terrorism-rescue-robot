#ifndef PROTO_VISION_RESULT_H
#define PROTO_VISION_RESULT_H

#include "proto_vision.h"
#include <stddef.h>

/* 版本1结果帧。扫描请求4字节：[0=沿用QR目标, 停车位1..3, 窗口ms低字节, 高字节]。
 * QR/定位/取消仍为空载荷；扫描成功返回当前位，失败reason=20表示该位非目标。
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
bool proto_vision_result_control_ready(const VisionResult_t *result,
                                      uint8_t samples, float max_scatter_mm);

#endif
