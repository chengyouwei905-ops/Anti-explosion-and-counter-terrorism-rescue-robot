#include "app_chassis_closeloop.h"
#include "app_chassis.h"
#include "app_imu.h"
#include "alg_pid.h"
#include <math.h>

/* 实车0/90度往返和15度小阶跃调参；控制周期10ms。
 * PID_Update的D使用误差差分，不除dt；Kd=90对应10ms下约0.9的连续微分系数。
 * 修改控制周期或角速度限幅后需重新验证。 */
#define CHASSIS_YAW_KP_RAD 4.0f
#define CHASSIS_YAW_KI 0.0f
#define CHASSIS_YAW_KD 90.0f
#define CHASSIS_MAX_OMEGA_RADPS APP_CHASSIS_MAX_OMEGA_RADPS

static PID_t angle_pid_t;
static Chassis_CloseLoop_State_t chassis_closeloop_state;

void app_chassis_closeloop_init(void)
{
    PID_Init(&angle_pid_t, CHASSIS_YAW_KP_RAD, CHASSIS_YAW_KI, CHASSIS_YAW_KD,
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
