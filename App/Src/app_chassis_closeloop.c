#include "app_chassis_closeloop.h"
#include "app_chassis.h"
#include "app_imu.h"
#include "alg_pid.h"
#include <math.h>

/* 原来0.1(rad/s)/deg的比例增益换算为(rad/s)/rad；实车仍需验证。 */
#define CHASSIS_YAW_KP_RAD (0.1f * 57.29578f)
#define CHASSIS_MAX_OMEGA_RADPS 5.0f

static PID_t angle_pid_t;
static Chassis_CloseLoop_State_t chassis_closeloop_state;

void app_chassis_closeloop_init(void)
{
    PID_Init(&angle_pid_t, CHASSIS_YAW_KP_RAD, 0.0f, 0.0f,
             -CHASSIS_MAX_OMEGA_RADPS, CHASSIS_MAX_OMEGA_RADPS);
    chassis_closeloop_state = (Chassis_CloseLoop_State_t){0};
}

void app_chassis_closeloop_set_target(float vx, float vy, float yawangle)
{
    app_chassis_closeloop_set_target_ff(vx, vy, yawangle, 0.0f);
}

void app_chassis_closeloop_set_target_ff(float vx, float vy, float yawangle, float omega_ff)
{
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(yawangle) || !isfinite(omega_ff)) {
        return;
    }
    chassis_closeloop_state.speed_x = vx;
    chassis_closeloop_state.speed_y = vy;
    chassis_closeloop_state.yaw = yawangle;
    chassis_closeloop_state.omega_ff = omega_ff;
}

void app_chassis_closeloop_update(void)
{
    float current_yaw[3];
    if (!app_imu_get_euler(current_yaw) || !isfinite(current_yaw[2])) {
        (void)app_chassis_move(0.0f, 0.0f, 0.0f);
        return;
    }
    /* IMU和目标均为连续弧度角；不取最短角，允许执行指定方向的多圈转动。 */
    angle_pid_t.Actual = current_yaw[2];
    angle_pid_t.Target = chassis_closeloop_state.yaw;
    PID_Update(&angle_pid_t);

    float omega = chassis_closeloop_state.omega_ff + angle_pid_t.Out;
    omega = fmaxf(-CHASSIS_MAX_OMEGA_RADPS, fminf(omega, CHASSIS_MAX_OMEGA_RADPS));
    (void)app_chassis_move(chassis_closeloop_state.speed_x,
                          chassis_closeloop_state.speed_y, omega);
}
