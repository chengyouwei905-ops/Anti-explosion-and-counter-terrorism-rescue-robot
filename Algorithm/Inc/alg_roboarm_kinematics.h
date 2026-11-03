#ifndef __ALG_ROBOARM_KINEMATICS_H__
#define __ALG_ROBOARM_KINEMATICS_H__

/* 舵机轴 35 齿，转台轴 44 齿。只在正逆解中换算一次。
 * Roboarm_State_t.yaw_angle 是舵机角；几何计算使用转台角。
 */
#define ROBOARM_YAW_OUTPUT_PER_SERVO (35.0 / 44.0)
/* 转台角限制，正逆解共用；舵机原始位置仍必须在 0..1000。 */
#define ROBOARM_YAW_MIN_RAD (-100.0 * 3.14159265358979323846 / 180.0)
#define ROBOARM_YAW_MAX_RAD (100.0 * 3.14159265358979323846 / 180.0)
/* 用户重新审核的关节范围，正逆解共用。 */
#define ROBOARM_JOINT1_MIN_RAD (-78.0 * 3.14159265358979323846 / 180.0)
#define ROBOARM_JOINT1_MAX_RAD (78.0 * 3.14159265358979323846 / 180.0)
#define ROBOARM_JOINT2_MIN_RAD (-60.0 * 3.14159265358979323846 / 180.0)
#define ROBOARM_JOINT2_MAX_RAD (180.0 * 3.14159265358979323846 / 180.0)
#define ROBOARM_JOINT3_MIN_RAD (-70.0 * 3.14159265358979323846 / 180.0)
#define ROBOARM_JOINT3_MAX_RAD (70.0 * 3.14159265358979323846 / 180.0)
/* joint2 - joint1 的连杆差角，2026-11-01 用户校核。 */
#define ROBOARM_JOINT_DIFF_MIN_RAD (20.0 * 3.14159265358979323846 / 180.0)
#define ROBOARM_JOINT_DIFF_MAX_RAD (160.0 * 3.14159265358979323846 / 180.0)

typedef struct{
    double yaw_angle_offset;
    double joint1_angle_offset;
    double joint2_angle_offset;
    double joint3_angle_offset;
} Roboarm_Offset_t;

typedef struct{
    double x;
    double y;
} Roboarm_Rectangular_Plane_Coordinate_System_t;

typedef enum{
    YAW = 1,
    JOINT_1,
    JOINT_2,
    JOINT_3,
    CLAMP
} Joint_Servo_ID_t;

typedef enum{
    ALG_ROBOARM_KINEMATICS_L0 = 50, /* 机械臂第0段长度 关节一到参考平面距离 */
    ALG_ROBOARM_KINEMATICS_L1 = 80, /* 机械臂第1段长度 关节一到关节二距离*/
    ALG_ROBOARM_KINEMATICS_L2 = 130, /* 机械臂第2段长度 关节二到关节三距离*/
    ALG_ROBOARM_KINEMATICS_L3 = 80, /* 机械臂第3段长度 夹爪中心到末端平行连杆的垂直距离*/
    ALG_ROBOARM_KINEMATICS_L4 = 20, /* 机械臂第4段长度 末端连杆的半长*/
} Roboarm_Stick_ID_t;

typedef struct{
    double yaw_angle;
    double joint1_angle;
    double joint2_angle;
    double joint3_angle;
    //clamp夹爪执行关节另外单独控制
} Roboarm_State_t;

typedef struct{
    double x;
    double y;
    double z;
    double pitch;
} End_Effector_Pos_t;

typedef enum{
    ROBOARM_KINEMATICS_ERR = 0,
    ROBOARM_KINEMATICS_OK
} Roboarm_Kinematics_Status_t;

void alg_roboarm_init(void);
Roboarm_Kinematics_Status_t alg_roboarm_forward(const Roboarm_State_t *roboarm_state, End_Effector_Pos_t *end_effector_pos);
Roboarm_Kinematics_Status_t alg_roboarm_invert(const End_Effector_Pos_t *end_effector_pos, Roboarm_State_t *roboarm_state);

#endif
