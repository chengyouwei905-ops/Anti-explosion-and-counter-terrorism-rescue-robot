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

/* 麦轮底盘运动学正解（逆解的逆）：四轮行程 -> 底盘中心位移，正方向约定同 alg_chassis_invert()
 * ⚠️ 运动学是线性的：输入"行程增量(mm)"时，输出的就是"位移增量(mm)"，
 *    轮序取 Wheel_ID_t，正 = 轮子驱动底盘前进 */
void alg_chassis_forward(const float wheel_speed[WHEEL_NUMBERS], Chassis_State_t *state)
{
    float lf = wheel_speed[LEFT_FRONT_WHELL];
    float rf = wheel_speed[RIGHT_FRONT_WHELL];
    float lr = wheel_speed[LEFT_REAR_WHELL];
    float rr = wheel_speed[RIGHT_REAR_WHELL];

    state->speed_x = (lf - rf - lr + rr) * 0.25f;
    state->speed_y = (lf + rf + lr + rr) * 0.25f;
    state->omega   = (rf + rr - lf - lr) * 0.25f / CHASSIS_K_MM;
}
