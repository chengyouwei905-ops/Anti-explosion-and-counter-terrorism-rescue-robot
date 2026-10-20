#include "alg_roboarm_kinematics.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

Roboarm_Offset_t roboarm_offset = {0.0, 0.0, 0.0, 0.0};

//初始化机械臂运动学参数，主要包括零点标定的值
//offset表示机械臂在我们认为的0点时（所有杆均竖直指向上方的时候各舵机的position值的大小）
void alg_roboarm_init(void)
{
    //改在app层进行
/*     roboarm_offset.yaw_angle_offset = yaw_angle_offset;
    roboarm_offset.joint1_angle_offset = joint1_angle_offset;
    roboarm_offset.joint2_angle_offset = joint2_angle_offset;
    roboarm_offset.joint3_angle_offset = joint3_angle_offset; */
    return;
}

/* 正解 */
Roboarm_Kinematics_Status_t alg_roboarm_forward(const Roboarm_State_t *roboarm_state, End_Effector_Pos_t *end_effector_pos)
{
    Roboarm_State_t roboarm_state_calculate = *roboarm_state;
    /* angle合法性检查，手动标定,PI / 2为还未标定的值 */
    if (roboarm_state_calculate.yaw_angle < -PI / 2 || roboarm_state_calculate.yaw_angle > PI / 2)
    {
        return ROBOARM_KINEMATICS_ERR;
    }
    if (roboarm_state_calculate.joint1_angle < -PI / 2 || roboarm_state_calculate.joint1_angle > PI / 2)
    {
        return ROBOARM_KINEMATICS_ERR;
    }
    if (roboarm_state_calculate.joint2_angle < -PI / 2 || roboarm_state_calculate.joint2_angle > PI / 2)
    {
        return ROBOARM_KINEMATICS_ERR;
    }
    if (roboarm_state_calculate.joint3_angle < -PI / 2 || roboarm_state_calculate.joint3_angle > PI / 2)
    {
        return ROBOARM_KINEMATICS_ERR;
    }

    end_effector_pos->z = ALG_ROBOARM_KINEMATICS_L0 
    + ALG_ROBOARM_KINEMATICS_L1 * cos(roboarm_state_calculate.joint1_angle) 
    + ALG_ROBOARM_KINEMATICS_L2 * cos(roboarm_state_calculate.joint2_angle) 
    + ALG_ROBOARM_KINEMATICS_L3 * cos(roboarm_state_calculate.joint2_angle + roboarm_state_calculate.joint3_angle)
    + ALG_ROBOARM_KINEMATICS_L4 * cos(PI / 2 - (roboarm_state_calculate.joint2_angle + roboarm_state_calculate.joint3_angle)); //末端执行器中心的高度z
    double r = ALG_ROBOARM_KINEMATICS_L1 * sin(roboarm_state_calculate.joint1_angle)
    + ALG_ROBOARM_KINEMATICS_L2 * sin(roboarm_state_calculate.joint2_angle)
    + ALG_ROBOARM_KINEMATICS_L3 * sin(roboarm_state_calculate.joint2_angle + roboarm_state_calculate.joint3_angle)
    - ALG_ROBOARM_KINEMATICS_L4 * sin(PI / 2 - (roboarm_state_calculate.joint2_angle + roboarm_state_calculate.joint3_angle)); //俯视投影长度r
    end_effector_pos->x = r * sin(roboarm_state_calculate.yaw_angle);
    end_effector_pos->y = r * cos(roboarm_state_calculate.yaw_angle);
    end_effector_pos->pitch =  PI / 2 - (roboarm_state_calculate.joint2_angle + roboarm_state_calculate.joint3_angle);
    //满足界限条件，待补充


    return ROBOARM_KINEMATICS_OK;
}

/* 逆解 */
//只取从上方接触目标点的逆解，从下方弯折的逆解方式忽略
Roboarm_Kinematics_Status_t alg_roboarm_invert(const End_Effector_Pos_t *end_effector_pos, Roboarm_State_t *roboarm_state)
{
    End_Effector_Pos_t end_effector_pos_calculate = *end_effector_pos;

    roboarm_state->yaw_angle = atan2(end_effector_pos_calculate.x, end_effector_pos_calculate.y);
    double r = sqrt(end_effector_pos_calculate.x * end_effector_pos_calculate.x + end_effector_pos_calculate.y * end_effector_pos_calculate.y); //俯视投影长度r

    Roboarm_Rectangular_Plane_Coordinate_System_t coordinate_D = {r, end_effector_pos_calculate.z - ALG_ROBOARM_KINEMATICS_L0};
    Roboarm_Rectangular_Plane_Coordinate_System_t coordinate_C = {coordinate_D.x - ALG_ROBOARM_KINEMATICS_L3 * cos(end_effector_pos_calculate.pitch) + ALG_ROBOARM_KINEMATICS_L4 * sin(end_effector_pos_calculate.pitch), coordinate_D.y - ALG_ROBOARM_KINEMATICS_L3 * sin(end_effector_pos_calculate.pitch) - ALG_ROBOARM_KINEMATICS_L4 * cos(end_effector_pos_calculate.pitch)};
    double L_AC = sqrt(coordinate_C.x * coordinate_C.x + coordinate_C.y * coordinate_C.y);
    if (L_AC < 50 || L_AC > 210)        //满足两边之和大于第三边两边之差小于第三边
    {
        return ROBOARM_KINEMATICS_ERR;
    }
    double temp_alpha = PI - acos((ALG_ROBOARM_KINEMATICS_L1 * ALG_ROBOARM_KINEMATICS_L1 + ALG_ROBOARM_KINEMATICS_L2 * ALG_ROBOARM_KINEMATICS_L2 - L_AC * L_AC) / (2 * ALG_ROBOARM_KINEMATICS_L1 * ALG_ROBOARM_KINEMATICS_L2));
    double temp_beta = atan2(coordinate_C.y, coordinate_C.x);
    double temp_gama = acos((ALG_ROBOARM_KINEMATICS_L1 * ALG_ROBOARM_KINEMATICS_L1 + L_AC * L_AC - ALG_ROBOARM_KINEMATICS_L2 * ALG_ROBOARM_KINEMATICS_L2) / (2 * ALG_ROBOARM_KINEMATICS_L1 * L_AC));
    roboarm_state->joint1_angle = PI / 2 - temp_beta - temp_gama;
    roboarm_state->joint2_angle = temp_alpha + roboarm_state->joint1_angle;
    roboarm_state->joint3_angle = PI / 2 - roboarm_state->joint2_angle - end_effector_pos->pitch;

    /* angle合法性检查，手动标定,PI / 2为还未标定的值 */
    if (roboarm_state->yaw_angle < -PI / 2 || roboarm_state->yaw_angle > PI / 2)
    {
        return ROBOARM_KINEMATICS_ERR;
    }
    if (roboarm_state->joint1_angle < -PI / 2 || roboarm_state->joint1_angle > PI / 2)
    {
        return ROBOARM_KINEMATICS_ERR;
    }
    if (roboarm_state->joint2_angle < -PI / 2 || roboarm_state->joint2_angle > PI / 2)
    {
        return ROBOARM_KINEMATICS_ERR;
    }
    if (roboarm_state->joint3_angle < -PI / 2 || roboarm_state->joint3_angle > PI / 2)
    {
        return ROBOARM_KINEMATICS_ERR;
    }

    return ROBOARM_KINEMATICS_OK;
}
