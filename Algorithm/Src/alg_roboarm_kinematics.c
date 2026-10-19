#include "alg_roboarm_kinematics.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

Roboarm_Kinematics_Status_t alg_roboarm_invert(const Roboarm_State_t *roboarm_state, End_Effector_Pos_t *end_effector_pos)
{
    end_effector_pos->z = ALG_ROBOARM_KINEMATICS_L0 
    + ALG_ROBOARM_KINEMATICS_L1 * cos(roboarm_state->joint1_angle) 
    + ALG_ROBOARM_KINEMATICS_L2 * cos(roboarm_state->joint1_angle + roboarm_state->joint2_angle) 
    + ALG_ROBOARM_KINEMATICS_L3 * cos(roboarm_state->joint1_angle + roboarm_state->joint2_angle + roboarm_state->joint3_angle);
    double r = ALG_ROBOARM_KINEMATICS_L1 * sin(roboarm_state->joint1_angle)
    + ALG_ROBOARM_KINEMATICS_L2 * sin(roboarm_state->joint1_angle + roboarm_state->joint2_angle)
    + ALG_ROBOARM_KINEMATICS_L3 * sin(roboarm_state->joint1_angle + roboarm_state->joint2_angle + roboarm_state->joint3_angle); //俯视投影长度r
    end_effector_pos->x = r * sin(roboarm_state->yaw_angle);
    end_effector_pos->y = r * cos(roboarm_state->yaw_angle);
    end_effector_pos->pitch = roboarm_state->joint1_angle + roboarm_state->joint2_angle + roboarm_state->joint3_angle - PI / 2;
    if(1)       //满足界限条件，待补充
    {
        return ROBOARM_KINEMATICS_OK;
    }
    //不满足需要清除错误的解算，防止误用
    return ROBOARM_KINEMATICS_ERR;
}

//只取从上方接触目标点的逆解，从下方弯折的逆解方式忽略
Roboarm_Kinematics_Status_t alg_roboarm_forward(const End_Effector_Pos_t *end_effector_pos, Roboarm_State_t *roboarm_state)
{
    roboarm_state->yaw_angle = atan2(end_effector_pos->x, end_effector_pos->y);
    double r = sqrt(end_effector_pos->x * end_effector_pos->x + end_effector_pos->y * end_effector_pos->y); //俯视投影长度r
    Roboarm_Rectangular_Plane_Coordinate_System_t coordinate_D = {r, end_effector_pos->z - ALG_ROBOARM_KINEMATICS_L0};
    Roboarm_Rectangular_Plane_Coordinate_System_t coordinate_C = {coordinate_D.x - ALG_ROBOARM_KINEMATICS_L3 * cos(end_effector_pos->pitch), coordinate_D.y - ALG_ROBOARM_KINEMATICS_L3 * sin(end_effector_pos->pitch)};
    double L_AC = sqrt(coordinate_C.x * coordinate_C.x + coordinate_C.y * coordinate_C.y);
    double temp_alpha = atan2(coordinate_C.y, coordinate_C.x);
    double temp_beta = atan2(coordinate_D.y, coordinate_D.x);
    roboarm_state->joint2_angle = PI - acos((ALG_ROBOARM_KINEMATICS_L1 * ALG_ROBOARM_KINEMATICS_L1 + ALG_ROBOARM_KINEMATICS_L2 * ALG_ROBOARM_KINEMATICS_L2 - L_AC * L_AC) / (2 * ALG_ROBOARM_KINEMATICS_L1 * ALG_ROBOARM_KINEMATICS_L2));
    roboarm_state->joint1_angle = temp_alpha - roboarm_state->joint2_angle;
    roboarm_state->joint3_angle = temp_bate - roboarm_state->joint1_angle - roboarm_state->joint2_angle;
    if(1)       //满足界限条件，待补充
    {
        return ROBOARM_KINEMATICS_OK;
    }
    //不满足需要清除错误的逆解算，防止误用
    return ROBOARM_KINEMATICS_ERR;
}
