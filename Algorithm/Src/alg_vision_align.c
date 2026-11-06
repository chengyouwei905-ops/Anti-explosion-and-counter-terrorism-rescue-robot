#include "alg_vision_align.h"
#include <math.h>
#include <stddef.h>

bool alg_vision_transform_from_arm_pose(const End_Effector_Pos_t *p, VisionTransform_t *out)
{
    if (p == NULL || out == NULL || !isfinite(p->x) || !isfinite(p->y) ||
        !isfinite(p->z) || !isfinite(p->pitch) || fabs(p->pitch) > 3.141592653589793 ||
        hypot(p->x, p->y) < 1e-6) {
        return false;
    }
    float yaw = (float)atan2(p->x, p->y);
    float c = cosf(yaw), s = sinf(yaw);
    float cp = (float)cos(p->pitch), sp = (float)sin(p->pitch);
    VisionTransform_t t = {
        .rotation = {{c, -s*cp, s*sp}, {s, c*cp, -c*sp}, {0.0f, sp, cp}},
        .translation = {(float)-p->x, (float)p->y, (float)p->z}
    };
    if (!isfinite(t.translation[0]) || !isfinite(t.translation[1]) || !isfinite(t.translation[2])) {
        return false;
    }
    *out = t;
    return true;
}

bool alg_vision_transform_from_arm_joints(const Roboarm_State_t *j, VisionTransform_t *out)
{
    if (j == NULL || out == NULL || !isfinite(j->yaw_angle) || !isfinite(j->joint1_angle) ||
        !isfinite(j->joint2_angle) || !isfinite(j->joint3_angle)) {
        return false;
    }
    End_Effector_Pos_t pose;
    if (alg_roboarm_forward(j, &pose) != ROBOARM_KINEMATICS_OK) {
        return false;
    }
    /* 正解已处理 yaw 的 1:2 齿轮减速，下面不再乘二/除二。 */
    return alg_vision_transform_from_arm_pose(&pose, out);
}

static bool finite_vector(const float *v, unsigned n)
{
    for (unsigned i = 0U; i < n; ++i) {
        if (!isfinite(v[i])) {
            return false;
        }
    }
    return true;
}

static bool unit_vector(const float *v, unsigned n)
{
    if (!finite_vector(v, n)) {
        return false;
    }
    float norm2 = 0.0f;
    for (unsigned i = 0; i < n; ++i) {
        norm2 += v[i] * v[i];
    }
    return fabsf(norm2 - 1.0f) <= 0.02f;
}

static void rotate(const VisionTransform_t *t, const float in[3], float out[3])
{
    for (unsigned i = 0; i < 3; ++i) {
        out[i] = t->rotation[i][0] * in[0] + t->rotation[i][1] * in[1] +
                 t->rotation[i][2] * in[2];
    }
}

bool alg_vision_alignment_config_valid(const VisionAlignmentConfig_t *c)
{
    if (c == NULL || !finite_vector(c->body_from_tcp.translation, 3) ||
        !finite_vector(c->desired_reference_body, 3) ||
        !finite_vector(c->min_reference_body, 3) || !finite_vector(c->max_reference_body, 3) ||
        !isfinite(c->imu_to_map_yaw) || !isfinite(c->desired_map_yaw)) {
        return false;
    }
    /* 行向量正交、单位长度且 det=+1，拒绝零矩阵/镜像/误填缩放。 */
    const VisionTransform_t *t = &c->body_from_tcp;
    for (unsigned i = 0; i < 3; ++i) {
        if (!unit_vector(t->rotation[i], 3) || c->min_reference_body[i] >= c->max_reference_body[i] ||
            c->desired_reference_body[i] < c->min_reference_body[i] ||
            c->desired_reference_body[i] > c->max_reference_body[i]) {
            return false;
        }
        for (unsigned j = i + 1; j < 3; ++j) {
            float dot = 0.0f;
            for (unsigned k = 0; k < 3; ++k) {
                dot += t->rotation[i][k] * t->rotation[j][k];
            }
            if (fabsf(dot) > 0.01f) {
                return false;
            }
        }
    }
    float det = t->rotation[0][0] * (t->rotation[1][1]*t->rotation[2][2] - t->rotation[1][2]*t->rotation[2][1])
              - t->rotation[0][1] * (t->rotation[1][0]*t->rotation[2][2] - t->rotation[1][2]*t->rotation[2][0])
              + t->rotation[0][2] * (t->rotation[1][0]*t->rotation[2][1] - t->rotation[1][1]*t->rotation[2][0]);
    if (fabsf(det - 1.0f) > 0.02f) {
        return false;
    }
    if (c->yaw_mode < VISION_YAW_NONE || c->yaw_mode > VISION_YAW_TARGET_NORMAL) {
        return false;
    }
    if (c->yaw_mode != VISION_YAW_NONE &&
        (!unit_vector(c->desired_direction_body, 3) ||
         hypotf(c->desired_direction_body[0], c->desired_direction_body[1]) < 0.2f)) {
        return false;
    }
    if (c->yaw_mode == VISION_YAW_QR_AXIS && !unit_vector(c->qr_axis, 3)) {
        return false;
    }
    return !c->map_reference_valid || finite_vector(c->reference_map, 3);
}

static bool get_direction(const VisionResult_t *r, const VisionAlignmentConfig_t *c,
                           float body_direction[3])
{
    float tcp_direction[3];
    if (c->yaw_mode == VISION_YAW_QR_AXIS) {
        if (r->task_cmd != VISION_CMD_QR || (r->flags & VISION_QUATERNION) == 0U ||
            !unit_vector(r->quaternion_wxyz, 4)) {
            return false;
        }
        /* q * axis * conjugate(q)，先归一化，避免有效浮点舍入改变尺度。 */
        float norm = sqrtf(r->quaternion_wxyz[0]*r->quaternion_wxyz[0] +
                          r->quaternion_wxyz[1]*r->quaternion_wxyz[1] +
                          r->quaternion_wxyz[2]*r->quaternion_wxyz[2] +
                          r->quaternion_wxyz[3]*r->quaternion_wxyz[3]);
        float w = r->quaternion_wxyz[0] / norm;
        float x = r->quaternion_wxyz[1] / norm;
        float y = r->quaternion_wxyz[2] / norm;
        float z = r->quaternion_wxyz[3] / norm;
        VisionTransform_t q = {.rotation = {
            {1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)},
            {2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)},
            {2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)}
        }};
        rotate(&q, c->qr_axis, tcp_direction);
    } else {
        if (r->task_cmd != VISION_CMD_TARGET_LOCATE || (r->flags & VISION_NORMAL) == 0U ||
            !unit_vector(r->normal, 3)) {
            return false;
        }
        for (unsigned i = 0; i < 3; ++i) {
            tcp_direction[i] = r->normal[i];
        }
    }
    rotate(&c->body_from_tcp, tcp_direction, body_direction);
    return hypotf(body_direction[0], body_direction[1]) >= 0.2f;
}

bool alg_vision_alignment_compute(const VisionResult_t *r,
                                  const VisionAlignmentConfig_t *c,
                                  float observation_imu_yaw, VisionAlignment_t *out)
{
    if (r == NULL || out == NULL || !isfinite(observation_imu_yaw) ||
        !alg_vision_alignment_config_valid(c) ||
        !proto_vision_result_control_ready(r, 3U, VISION_MEASUREMENT_MAX_SCATTER_MM)) {
        return false;
    }
    VisionAlignment_t a = {0};
    rotate(&c->body_from_tcp, r->xyz_mm, a.reference_body);
    for (unsigned i = 0; i < 3; ++i) {
        a.reference_body[i] += c->body_from_tcp.translation[i];
        /* 打靶底盘只修正X/Y；Z保留给作业层诊断，不用高度范围阻止作业。 */
        bool check_range = i != 2U || r->task_cmd != VISION_CMD_TARGET_LOCATE;
        if (!isfinite(a.reference_body[i]) ||
            (check_range && (a.reference_body[i] < c->min_reference_body[i] ||
                             a.reference_body[i] > c->max_reference_body[i]))) {
            return false;
        }
        a.chassis_from_reference_body[i] = -a.reference_body[i];
    }
    if (c->yaw_mode != VISION_YAW_NONE) {
        float measured[3];
        if (!get_direction(r, c, measured)) {
            return false;
        }
        const float *desired = c->desired_direction_body;
        a.delta_yaw = atan2f(desired[0]*measured[1] - desired[1]*measured[0],
                            desired[0]*measured[0] + desired[1]*measured[1]);
    }
    /* p_after = R(-dyaw) * (p_before - delta_B)
     * 因而 delta_B = p_before - R(dyaw) * desired_reference_B。
     * 作业层先转向并重新观测，再使用新坐标做平移，不复用转向前的旧 XYZ。
     */
    float cs = cosf(a.delta_yaw), sn = sinf(a.delta_yaw);
    a.delta_body[0] = a.reference_body[0] -
                      (cs*c->desired_reference_body[0] - sn*c->desired_reference_body[1]);
    a.delta_body[1] = a.reference_body[1] -
                      (sn*c->desired_reference_body[0] + cs*c->desired_reference_body[1]);
    a.map_valid = c->map_reference_valid;
    if (a.map_valid) {
        a.map_yaw = c->yaw_mode == VISION_YAW_NONE ? observation_imu_yaw + c->imu_to_map_yaw :
                    c->desired_map_yaw - a.delta_yaw;
        cs = cosf(a.map_yaw);
        sn = sinf(a.map_yaw);
        a.chassis_map[0] = c->reference_map[0] - (cs*a.reference_body[0] - sn*a.reference_body[1]);
        a.chassis_map[1] = c->reference_map[1] - (sn*a.reference_body[0] + cs*a.reference_body[1]);
        a.chassis_map[2] = c->reference_map[2] - a.reference_body[2];
        if (!finite_vector(a.chassis_map, 3) || !isfinite(a.map_yaw)) {
            return false;
        }
    }
    if (!finite_vector(a.delta_body, 2) || !isfinite(a.delta_yaw)) {
        return false;
    }
    *out = a;
    return true;
}
