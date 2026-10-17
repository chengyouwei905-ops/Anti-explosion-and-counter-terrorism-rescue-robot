#include "app_chassis_motion.h"
#include "app_chassis_closeloop.h"
#include "app_imu.h"
#include "alg_motion_profile.h"
#include <math.h>

static Chassis_Motion_State_t g_motion_state = MOTION_STATE_IDLE;
static Chassis_Motion_Type_t g_motion_type = MOTION_TYPE_LINEAR_MOVE;
static float g_start_yaw;       /* 本条指令起点的实测航向 */
static float g_direction;       /* 直线在固定坐标系中的方向 */
static float g_orientation;     /* 圆弧在固定坐标系中的起始切线方向 */
static float g_radius;
static float g_turn_sign;
static float g_target_yaw;

static int motion_get_yaw(float *yaw)
{
    float euler[3];
    if (!app_imu_get_euler(euler) || !isfinite(euler[2])) {
        return 0;
    }
    *yaw = euler[2];
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
    app_chassis_closeloop_set_target(0.0f, 0.0f, g_target_yaw);
}

void app_chassis_motion_circular_move(float radius, float angle, float orientation,
                                      float max_speed, float accel, float decel)
{
    if (g_motion_state != MOTION_STATE_IDLE || !isfinite(radius) || radius < 0.0f ||
        !isfinite(angle) || !isfinite(orientation) ||
        !motion_limits_valid(max_speed, accel, decel) || !motion_get_yaw(&g_start_yaw)) {
        return;
    }
    /* 原地转弯直接规划角度，避免除以零；正半径时规划非负弧长。 */
    float distance = (radius == 0.0f) ? fabsf(angle) : radius * fabsf(angle);
    float initial_yaw = (radius == 0.0f) ? g_start_yaw : g_start_yaw + orientation;
    if (!isfinite(distance) || !isfinite(initial_yaw + angle)) {
        return;
    }
    alg_motion_profile_init(distance, max_speed, accel, decel);
    g_orientation = initial_yaw;
    g_target_yaw = initial_yaw + angle;
    g_radius = radius;
    g_turn_sign = (angle < 0.0f) ? -1.0f : 1.0f;
    g_motion_type = MOTION_TYPE_CIRCULAR_MOVE;
    g_motion_state = alg_motion_profile_update(0.0f) ? MOTION_STATE_IDLE : MOTION_STATE_RUNNING;
    /* 零转角或规划无效时保持原朝向，不额外旋转orientation。 */
    if (g_motion_state == MOTION_STATE_IDLE) {
        g_target_yaw = g_start_yaw;
    }
    app_chassis_closeloop_set_target(0.0f, 0.0f, g_start_yaw);
}

void app_chassis_motion_update(float addtime)
{
    if (g_motion_state == MOTION_STATE_IDLE || !isfinite(addtime) || addtime <= 0.0f) {
        return;
    }
    float actual_yaw;
    if (!motion_get_yaw(&actual_yaw)) {
        app_chassis_motion_stop();
        return;
    }
    if (alg_motion_profile_update(addtime)) {
        g_motion_state = MOTION_STATE_IDLE;
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
