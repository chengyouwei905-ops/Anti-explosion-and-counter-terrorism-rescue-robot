#include "alg_chassis_kinematics.h"


/* 麦轮底盘运动学逆解
 * 约定：x 向右、y 向前、ω 俯视逆时针为正
 * 输出：四轮线速度 mm/s，正 = 驱动底盘前进
 */
void alg_chassis_invert(const Chassis_State_t *state, float wheel_speed[WHEEL_NUMBERS])
{
    float sx = state->speed_x;
    float sy = state->speed_y;
    float kw = CHASSIS_K_MM * state->omega;      /* mm × rad/s = mm/s */

    wheel_speed[LEFT_FRONT_WHELL]  =  sx + sy - kw;
    wheel_speed[RIGHT_FRONT_WHELL] = -sx + sy + kw;
    wheel_speed[LEFT_REAR_WHELL]   = -sx + sy - kw;
    wheel_speed[RIGHT_REAR_WHELL]  =  sx + sy + kw;
}

/* 麦轮底盘运动学正解 */
void alg_chassis_forward(const float wheel_speed[WHEEL_NUMBERS], Chassis_State_t *state)
{
    state->speed_x = 0;
    state->speed_y = 0;
    state->omega = 0;
    /* 待补充 */
}
