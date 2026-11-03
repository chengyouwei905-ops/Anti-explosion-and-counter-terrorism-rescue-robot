#include "app_chassis_motion.h"
#include "app_chassis_closeloop.h"
#include "app_chassis.h"
#include "app_imu.h"
#include "alg_motion_profile.h"
#include <math.h>
#include <stddef.h>

static Chassis_Motion_State_t g_motion_state = MOTION_STATE_IDLE;
static Chassis_Motion_Result_t g_motion_result = MOTION_RESULT_IDLE;
static Chassis_Motion_Type_t g_motion_type = MOTION_TYPE_LINEAR_MOVE;
static float g_start_yaw;       /* 本条指令起点的实测航向 */
static float g_direction;       /* 直线在固定坐标系中的方向 */
static float g_orientation;     /* 圆弧在固定坐标系中的起始切线方向 */
static float g_radius;
static float g_turn_sign;
static float g_target_yaw;
static MotionProfilePlan_t g_arc_translation_profile;
static float g_arc_translation_time;
typedef struct {
    float start, last, progress_mark, target;
    float speed, max_speed, accel, decel, elapsed, progress_time;
    uint32_t reset_count;
} DistanceMotion_t;
static DistanceMotion_t g_distance_move;

static int motion_get_yaw(float *yaw)
{
    double euler[3];
    if (!app_imu_get_euler(euler) || !isfinite(euler[2])) {
        return 0;
    }
    *yaw = (float)euler[2]; /* motion 层现有指令接口仍使用 float。 */
    return 1;
}

static int motion_limits_valid(float max_speed, float accel, float decel)
{
    return isfinite(max_speed) && max_speed > 0.0f &&
           isfinite(accel) && accel > 0.0f &&
           isfinite(decel) && decel > 0.0f;
}

void app_chassis_motion_init(void)
{
    g_motion_state = MOTION_STATE_IDLE;
    g_motion_result = MOTION_RESULT_IDLE;
    alg_chassis_path_reset();
    g_start_yaw = 0.0f;
    g_target_yaw = 0.0f;
    app_chassis_closeloop_init();
    /* 有有效姿态时保持当前航向；否则沿用闭环初始化后的0 rad目标。 */
    if (motion_get_yaw(&g_start_yaw)) {
        g_target_yaw = g_start_yaw;
        app_chassis_closeloop_set_target(0.0f, 0.0f, g_target_yaw);
    }
}

void app_chassis_motion_linear_move(float distance, float direction,
                                    float max_speed, float accel, float decel)
{
    if (g_motion_state != MOTION_STATE_IDLE || !isfinite(distance) || distance < 0.0f ||
        !isfinite(direction) || !motion_limits_valid(max_speed, accel, decel) ||
        !motion_get_yaw(&g_start_yaw)) {
        return;
    }
    if (!isfinite(g_start_yaw + direction)) {
        return;
    }
    alg_motion_profile_init(distance, max_speed, accel, decel);
    g_direction = g_start_yaw + direction;
    g_target_yaw = g_start_yaw;
    g_motion_type = MOTION_TYPE_LINEAR_MOVE;
    g_motion_state = alg_motion_profile_update(0.0f) ? MOTION_STATE_IDLE : MOTION_STATE_RUNNING;
    g_motion_result = g_motion_state == MOTION_STATE_RUNNING ? MOTION_RESULT_RUNNING : MOTION_RESULT_DONE;
    app_chassis_closeloop_set_target(0.0f, 0.0f, g_target_yaw);
}

Chassis_Motion_Start_t app_chassis_motion_linear_move_distance(float distance, float direction,
                                                              float max_speed, float accel,
                                                              float decel)
{
    if (g_motion_state != MOTION_STATE_IDLE) return MOTION_START_BUSY;
    if (!isfinite(distance) || distance < 0.0f || !isfinite(direction) ||
        !motion_limits_valid(max_speed, accel, decel)) return MOTION_START_INVALID;
    /* 拒绝即使理想执行也无法在故障时限内完成的参数，使用double避免验算溢出。 */
    double nominal_s = (double)distance / max_speed + (double)max_speed / accel +
                       (double)max_speed / decel;
    if (nominal_s > APP_CHASSIS_MOTION_DISTANCE_TIMEOUT_MS * 0.001) return MOTION_START_INVALID;
    float yaw;
    if (!motion_get_yaw(&yaw)) return MOTION_START_NO_IMU;
    if (!isfinite(yaw + direction)) return MOTION_START_INVALID;
    Chassis_DistanceSample_t feedback;
    if (!app_chassis_get_distance_sample(&feedback) ||
        feedback.age_ms > APP_CHASSIS_MOTION_ODOM_MAX_AGE_MS ||
        !isfinite(feedback.distance_mm) || feedback.distance_mm < 0.0f) return MOTION_START_NO_ODOM;

    g_distance_move = (DistanceMotion_t){
        .start = feedback.distance_mm, .last = feedback.distance_mm,
        .progress_mark = feedback.distance_mm, .target = distance,
        .max_speed = max_speed, .accel = accel, .decel = decel,
        .reset_count = feedback.reset_count
    };
    g_start_yaw = g_target_yaw = yaw;
    g_direction = yaw + direction;
    g_motion_type = MOTION_TYPE_LINEAR_DISTANCE;
    bool done = distance <= APP_CHASSIS_MOTION_DISTANCE_TOLERANCE_MM;
    g_motion_state = done ? MOTION_STATE_IDLE : MOTION_STATE_RUNNING;
    g_motion_result = done ? MOTION_RESULT_DONE : MOTION_RESULT_RUNNING;
    app_chassis_closeloop_set_target(0.0f, 0.0f, yaw);
    return MOTION_START_OK;
}

static void distance_move_update(float dt, float actual_yaw)
{
    Chassis_DistanceSample_t feedback;
    Chassis_Motion_Result_t failure = MOTION_RESULT_IDLE;
    if (!app_chassis_get_distance_sample(&feedback) ||
        feedback.age_ms > APP_CHASSIS_MOTION_ODOM_MAX_AGE_MS ||
        feedback.reset_count != g_distance_move.reset_count ||
        !isfinite(feedback.distance_mm) || feedback.distance_mm < g_distance_move.last) {
        failure = MOTION_RESULT_ODOM_ERROR;
    } else {
        g_distance_move.elapsed += dt;
        g_distance_move.last = feedback.distance_mm;
        float remaining = g_distance_move.target - (feedback.distance_mm - g_distance_move.start);
        if (remaining <= APP_CHASSIS_MOTION_DISTANCE_TOLERANCE_MM) {
            g_motion_state = MOTION_STATE_IDLE;
            g_motion_result = MOTION_RESULT_DONE;
            app_chassis_closeloop_set_target(0.0f, 0.0f, g_target_yaw);
            return;
        }
        /* 进展门限小于编码器里程死区，避免每拍浮点噪声续命。 */
        if (feedback.distance_mm - g_distance_move.progress_mark >= 0.1f) {
            g_distance_move.progress_mark = feedback.distance_mm;
            g_distance_move.progress_time = g_distance_move.elapsed;
        }
        if (g_distance_move.elapsed >= APP_CHASSIS_MOTION_DISTANCE_TIMEOUT_MS * 0.001f) {
            failure = MOTION_RESULT_TIMEOUT;
        } else if (g_distance_move.elapsed - g_distance_move.progress_time >= APP_CHASSIS_MOTION_DISTANCE_STALL_S) {
            failure = MOTION_RESULT_STALLED;
        } else {
            /* 剩余路程控制制动速度；保留可调低速下限，防止尾段命令落入死区。
             * 先限加速度，再由到量判据立即置零；不靠固定时长宣布走够。
             */
            float creep = fminf(APP_CHASSIS_MOTION_DISTANCE_MIN_SPEED, g_distance_move.max_speed);
            float braking = (float)sqrt(2.0 * g_distance_move.decel * remaining);
            float target = fminf(g_distance_move.max_speed, fmaxf(creep, braking));
            if (target > g_distance_move.speed) {
                g_distance_move.speed = fminf(target, g_distance_move.speed + g_distance_move.accel * dt);
            } else {
                g_distance_move.speed = fmaxf(target, g_distance_move.speed - g_distance_move.decel * dt);
            }
            float direction = g_direction - actual_yaw;
            app_chassis_closeloop_set_target(-g_distance_move.speed * sinf(direction),
                                             g_distance_move.speed * cosf(direction), g_target_yaw);
            return;
        }
    }
    app_chassis_motion_stop();
    g_motion_result = failure;
}

void app_chassis_motion_circular_move(float radius, float angle, bool orientation,
                                      float max_speed, float accel, float decel)
{
    if (g_motion_state != MOTION_STATE_IDLE || !isfinite(radius) || radius < 0.0f ||
        !isfinite(angle) ||
        !motion_limits_valid(max_speed, accel, decel) || !motion_get_yaw(&g_start_yaw)) {
        return;
    }
    /* 原地转弯直接规划角度，避免除以零；正半径时规划非负弧长。 */
    float distance = (radius == 0.0f) ? fabsf(angle) : radius * fabsf(angle);
    /* 圆弧从当前航向起步，orientation只选择左/右转，不附加初始航向偏角。 */
    float turn_angle = (radius == 0.0f) ? angle :
                       (orientation ? fabsf(angle) : -fabsf(angle));
    if (!isfinite(distance) || !isfinite(g_start_yaw + turn_angle)) {
        return;
    }
    alg_motion_profile_init(distance, max_speed, accel, decel);
    g_orientation = g_start_yaw;
    g_target_yaw = g_start_yaw + turn_angle;
    g_radius = radius;
    g_turn_sign = (turn_angle < 0.0f) ? -1.0f : 1.0f;
    g_motion_type = MOTION_TYPE_CIRCULAR_MOVE;
    g_motion_state = alg_motion_profile_update(0.0f) ? MOTION_STATE_IDLE : MOTION_STATE_RUNNING;
    g_motion_result = g_motion_state == MOTION_STATE_RUNNING ? MOTION_RESULT_RUNNING : MOTION_RESULT_DONE;
    /* 零转角或规划无效时保持原朝向。 */
    if (g_motion_state == MOTION_STATE_IDLE) {
        g_target_yaw = g_start_yaw;
    }
    app_chassis_closeloop_set_target(0.0f, 0.0f, g_start_yaw);
}

Chassis_Motion_Start_t app_chassis_motion_arc_translate(float radius, float angle,
                                                       float direction, float max_speed,
                                                       float accel, float decel)
{
    if (g_motion_state != MOTION_STATE_IDLE) {
        return MOTION_START_BUSY;
    }
    if (!isfinite(radius) || radius <= 0.0f || !isfinite(angle) ||
        !isfinite(direction) || !motion_limits_valid(max_speed, accel, decel)) {
        return MOTION_START_INVALID;
    }
    float distance = radius * fabsf(angle);
    MotionProfilePlan_t profile = {0};
    if (!isfinite(distance) ||
        (angle != 0.0f && !alg_motion_profile_plan(&profile, distance, 0.0f, 0.0f,
                                                 max_speed, accel, decel))) {
        return MOTION_START_INVALID;
    }
    float yaw;
    if (!motion_get_yaw(&yaw)) {
        return MOTION_START_NO_IMU;
    }

    /* 只归一化切线的三角函数输入，车头目标保留IMU的连续角。 */
    const float full_turn = 6.28318530718f;
    g_orientation = remainderf(yaw, full_turn) + remainderf(direction, full_turn);
    g_start_yaw = g_target_yaw = yaw;
    g_radius = radius;
    g_turn_sign = angle < 0.0f ? -1.0f : 1.0f;
    g_arc_translation_profile = profile;
    g_arc_translation_time = 0.0f;
    g_motion_type = MOTION_TYPE_ARC_TRANSLATION;
    g_motion_state = angle == 0.0f ? MOTION_STATE_IDLE : MOTION_STATE_RUNNING;
    g_motion_result = angle == 0.0f ? MOTION_RESULT_DONE : MOTION_RESULT_RUNNING;
    app_chassis_closeloop_set_target(0.0f, 0.0f, yaw);
    return MOTION_START_OK;
}

void app_chassis_motion_update(float addtime)
{
    if (g_motion_state == MOTION_STATE_IDLE || !isfinite(addtime) || addtime <= 0.0f) {
        return;
    }
    float actual_yaw;
    if (!motion_get_yaw(&actual_yaw)) {
        app_chassis_motion_stop();
        g_motion_result = MOTION_RESULT_IMU_ERROR;
        return;
    }
    if (g_motion_type == MOTION_TYPE_LINEAR_DISTANCE) {
        distance_move_update(addtime, actual_yaw);
        return;
    }
    if (g_motion_type == MOTION_TYPE_PATH || g_motion_type == MOTION_TYPE_PATH_TRANSLATION) {
        bool finished = alg_chassis_path_update(addtime);
        AlgPathSample_t reference;
        if (!alg_chassis_path_get_sample(&reference)) {
            app_chassis_motion_stop();
            return;
        }
        bool translate = g_motion_type == MOTION_TYPE_PATH_TRANSLATION;
        g_target_yaw = translate ? g_start_yaw : reference.yaw;
        float relative_direction = reference.yaw - actual_yaw;
        app_chassis_closeloop_set_target_ff(-reference.speed * sinf(relative_direction),
                                            reference.speed * cosf(relative_direction),
                                            g_target_yaw, translate ? 0.0f : reference.omega);
        if (finished) {
            g_motion_state = MOTION_STATE_IDLE;
            g_motion_result = MOTION_RESULT_DONE;
        }
        return;
    }
    if (g_motion_type == MOTION_TYPE_ARC_TRANSLATION) {
        g_arc_translation_time = fminf(g_arc_translation_time + addtime,
                                       g_arc_translation_profile.duration);
        float progress, speed, accel;
        if (!alg_motion_profile_sample(&g_arc_translation_profile, g_arc_translation_time,
                                        &progress, &speed, &accel)) {
            app_chassis_motion_stop();
            return;
        }
        if (g_arc_translation_time >= g_arc_translation_profile.duration) {
            g_motion_state = MOTION_STATE_IDLE;
            g_motion_result = MOTION_RESULT_DONE;
            app_chassis_closeloop_set_target(0.0f, 0.0f, g_target_yaw);
            return;
        }
        /* 弧长决定平移切线；它与车头yaw目标相互独立，角速度前馈为零。
         * 用实际yaw转换到车体系，即使航向有误差也保持规划的平移方向。
         */
        float tangent = g_orientation + g_turn_sign * (progress / g_radius);
        float relative_direction = tangent - remainderf(actual_yaw, 6.28318530718f);
        app_chassis_closeloop_set_target(-speed * sinf(relative_direction),
                                         speed * cosf(relative_direction), g_target_yaw);
        return;
    }
    if (alg_motion_profile_update(addtime)) {
        g_motion_state = MOTION_STATE_IDLE;
        g_motion_result = MOTION_RESULT_DONE;
        app_chassis_closeloop_set_target(0.0f, 0.0f, g_target_yaw);
        return;
    }

    float speed = alg_motion_profile_get_speed();
    if (g_motion_type == MOTION_TYPE_LINEAR_MOVE) {
        /* 固定参考方向转换为当前车体速度；航向误差不会改变参考平移方向。 */
        float relative_direction = g_direction - actual_yaw;
        app_chassis_closeloop_set_target(-speed * sinf(relative_direction),
                                         speed * cosf(relative_direction), g_target_yaw);
    } else {
        float progress = alg_motion_profile_get_distance();
        float yaw = g_orientation + g_turn_sign * ((g_radius == 0.0f) ? progress : progress / g_radius);
        float omega = g_turn_sign * ((g_radius == 0.0f) ? speed : speed / g_radius);
        float linear_speed = (g_radius == 0.0f) ? 0.0f : speed;
        float relative_direction = yaw - actual_yaw;
        app_chassis_closeloop_set_target_ff(-linear_speed * sinf(relative_direction),
                                            linear_speed * cosf(relative_direction), yaw, omega);
    }
}

void app_chassis_motion_stop(void)
{
    g_motion_state = MOTION_STATE_IDLE;
    g_motion_result = MOTION_RESULT_CANCELLED;
    alg_chassis_path_reset();
    if (motion_get_yaw(&g_target_yaw)) {
        app_chassis_closeloop_set_target(0.0f, 0.0f, g_target_yaw);
    } else {
        /* 无有效姿态时复位为零平移速度和0 rad目标。 */
        app_chassis_closeloop_init();
    }
}

Chassis_Motion_State_t app_chassis_motion_get_state(void)
{
    return g_motion_state;
}

Chassis_Motion_Result_t app_chassis_motion_get_result(void)
{
    return g_motion_result;
}

static Chassis_Motion_Start_t motion_path_start(const AlgPathSegment_t *segments,
                                                uint8_t count,
                                                const AlgPathLimits_t *limits,
                                                bool translate)
{
    if (g_motion_state != MOTION_STATE_IDLE) {
        return MOTION_START_BUSY;
    }
    if (limits == NULL || !isfinite(limits->max_omega) ||
        limits->max_omega > APP_CHASSIS_MAX_OMEGA_RADPS) {
        return MOTION_START_INVALID;
    }
    float yaw;
    if (!motion_get_yaw(&yaw)) {
        return MOTION_START_NO_IMU;
    }
    if (!alg_chassis_path_plan(segments, count, limits, yaw)) {
        return MOTION_START_INVALID;
    }
    g_motion_type = translate ? MOTION_TYPE_PATH_TRANSLATION : MOTION_TYPE_PATH;
    g_start_yaw = g_target_yaw = yaw;
    g_motion_state = MOTION_STATE_RUNNING;
    g_motion_result = MOTION_RESULT_RUNNING;
    app_chassis_closeloop_set_target(0.0f, 0.0f, yaw);
    return MOTION_START_OK;
}

Chassis_Motion_Start_t app_chassis_motion_path_start(const AlgPathSegment_t *segments,
                                                    uint8_t count,
                                                    const AlgPathLimits_t *limits)
{
    return motion_path_start(segments, count, limits, false);
}

Chassis_Motion_Start_t app_chassis_motion_path_translate_start(const AlgPathSegment_t *segments,
                                                              uint8_t count,
                                                              const AlgPathLimits_t *limits)
{
    return motion_path_start(segments, count, limits, true);
}

bool app_chassis_motion_path_get_sample(AlgPathSample_t *out)
{
    return (g_motion_type == MOTION_TYPE_PATH || g_motion_type == MOTION_TYPE_PATH_TRANSLATION) &&
           alg_chassis_path_get_sample(out);
}
