#include "proto_vision_result.h"
#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "Vision protocol requires IEEE754 binary32");

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool read_floats(const uint8_t *p, size_t len, size_t *offset,
                        float *out, size_t count)
{
    if (*offset > len || count > (len - *offset) / 4U) {
        return false;
    }
    for (size_t i = 0; i < count; ++i) {
        uint32_t bits = read_u32(p + *offset);
        memcpy(&out[i], &bits, sizeof(float));
        if (!isfinite(out[i])) {
            return false;
        }
        *offset += 4U;
    }
    return true;
}

bool proto_vision_is_scan(uint8_t cmd)
{
    return cmd == VISION_CMD_BOMB_SCAN || cmd == VISION_CMD_TARGET_SCAN ||
           cmd == VISION_CMD_RESCUE_SCAN;
}

bool proto_vision_is_measure(uint8_t cmd)
{
    return cmd == VISION_CMD_QR || cmd == VISION_CMD_BOMB_LOCATE ||
           cmd == VISION_CMD_TARGET_LOCATE || cmd == VISION_CMD_RESCUE_LOCATE;
}

bool proto_vision_decode_result(const uint8_t *p, size_t len, VisionResult_t *out)
{
    if (p == NULL || out == NULL || len < 15U || len > PROTO_VISION_MAX_PAYLOAD) {
        return false;
    }
    if (p[0] != 1U || p[3] > VISION_OBSERVATION || p[5] != 0U ||
        p[6] > 3U || (p[4] & 0x60U) != 0U) {
        return false;
    }
    if (!proto_vision_is_scan(p[1]) && !proto_vision_is_measure(p[1])) {
        return false;
    }
    VisionResult_t r = {0};
    r.task_cmd = p[1];
    r.input_seq = p[2];
    r.status = p[3];
    r.flags = p[4];
    r.slot_id = p[6];
    r.sample_count = p[7];
    r.run_id = read_u16(p + 8);
    r.frame_id = read_u32(p + 10);
    r.reason = p[14];

    if (r.status == VISION_FAILURE || r.status == VISION_CANCELLED) {
        if (len != 15U || r.flags != 0U || r.sample_count != 0U) {
            return false;
        }
        *out = r;
        return true;
    }
    if (proto_vision_is_scan(r.task_cmd)) {
        if (r.status != VISION_SUCCESS || len != 15U || r.flags != 0U ||
            r.sample_count != 0U || r.slot_id == 0U || r.reason != 0U) {
            return false;
        }
        *out = r;
        return true;
    }
    if ((r.flags & VISION_POSITION) == 0U || r.sample_count == 0U ||
        ((r.flags & VISION_QUATERNION) != 0U && r.task_cmd != VISION_CMD_QR)) {
        return false;
    }
    if (r.status == VISION_SUCCESS &&
        ((r.flags & VISION_CONTROL_READY) == 0U ||
         (r.flags & VISION_CANDIDATE) != 0U || r.reason != 0U)) {
        return false;
    }
    if (r.status == VISION_OBSERVATION && (r.flags & VISION_CONTROL_READY) != 0U) {
        return false;
    }

    size_t offset = 15U;
    if (!read_floats(p, len, &offset, r.xyz_mm, 3U)) {
        return false;
    }
    if ((r.flags & VISION_QUATERNION) != 0U &&
        !read_floats(p, len, &offset, r.quaternion_wxyz, 4U)) {
        return false;
    }
    if ((r.flags & VISION_NORMAL) != 0U &&
        !read_floats(p, len, &offset, r.normal, 3U)) {
        return false;
    }
    if ((r.flags & VISION_GRASP) != 0U &&
        !read_floats(p, len, &offset, r.grasp_mm, 3U)) {
        return false;
    }
    if (!read_floats(p, len, &offset, &r.scatter_mm, 1U) ||
        r.scatter_mm < 0.0f || offset != len) {
        return false;
    }
    *out = r;
    return true;
}

bool proto_vision_result_control_ready(const VisionResult_t *r,
                                      uint8_t samples, float max_scatter_mm)
{
    if (r == NULL || samples == 0U || !isfinite(max_scatter_mm) || max_scatter_mm < 0.0f) {
        return false;
    }
    if (!proto_vision_is_measure(r->task_cmd) || r->status != VISION_SUCCESS ||
        r->reason != 0U || (r->flags & 0x03U) != 0x03U ||
        (r->flags & VISION_CANDIDATE) != 0U || r->sample_count != samples ||
        !isfinite(r->scatter_mm) || r->scatter_mm < 0.0f || r->scatter_mm > max_scatter_mm) {
        return false;
    }
    return isfinite(r->xyz_mm[0]) && isfinite(r->xyz_mm[1]) && isfinite(r->xyz_mm[2]);
}
