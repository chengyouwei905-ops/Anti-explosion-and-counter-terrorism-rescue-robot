#ifndef ALG_VISION_ALIGN_H
#define ALG_VISION_ALIGN_H

#include "proto_vision_result.h"
#include "alg_roboarm_kinematics.h"

/* B：底盘中心为原点，X 向右、Y 向前、Z 向上；yaw 逆时针为正。
 * E：K230 当前手眼标定的 TCP 坐标系，不能假定与 B 同向。
 * p_B = rotation * p_E + translation，矩阵按行存储。
 */
typedef struct {
    float rotation[3][3];
    float translation[3];
} VisionTransform_t;

/* 与 K230 calibrate_handeye.py::robot_pose() 完全一致。
 * 机械臂输入 +X 向左，而底盘 B 的 +X 向右；原点按用户约定重合。
 * 指令姿态不是反馈姿态，必须由调用方保证机械臂已经到位并保持不动。
 */
bool alg_vision_transform_from_arm_pose(const End_Effector_Pos_t *pose, VisionTransform_t *out);
bool alg_vision_transform_from_arm_joints(const Roboarm_State_t *joints, VisionTransform_t *out);

typedef enum {
    VISION_YAW_NONE,          /* 球心/中点：只修正平移 */
    VISION_YAW_QR_AXIS,       /* QR 四元数旋转已知的 QR 局部轴 */
    VISION_YAW_TARGET_NORMAL /* 使用带方向的靶面法向，不能自动猜正反 */
} VisionYawMode_t;

typedef struct {
    VisionTransform_t body_from_tcp; /* 采集期间机械臂必须保持此姿态 */
    float desired_reference_body[3]; /* 对准后参考点在 B 系的期望位置 */
    float min_reference_body[3];     /* 观测范围mm；打靶只检查X/Y，Z仅诊断。 */
    float max_reference_body[3];
    VisionYawMode_t yaw_mode;
    float qr_axis[3];                /* 例如 QR 的 +Z 轴，需核实标定定义 */
    float desired_direction_body[3]; /* 期望上述轴/法向在 B 中的方向 */
    bool map_reference_valid;        /* 仅当目标是已知、未被搬动的地图参照物 */
    float reference_map[3];
    float desired_map_yaw;           /* 对准后的地图航向，rad */
    float imu_to_map_yaw;            /* 不使用视觉 yaw 时：地图 yaw = IMU + 此偏置 */
} VisionAlignmentConfig_t;

typedef struct {
    float reference_body[3];
    float chassis_from_reference_body[3]; /* -p_B，以 B 轴表达 */
    float delta_body[2];             /* 当前车体系中的期望底盘平移，X右/Y前 */
    float delta_yaw;                 /* 需要实际转动的角度，不直接篡改 IMU */
    bool map_valid;
    float chassis_map[3];
    float map_yaw;
} VisionAlignment_t;

bool alg_vision_alignment_config_valid(const VisionAlignmentConfig_t *config);
bool alg_vision_alignment_compute(const VisionResult_t *result,
                                  const VisionAlignmentConfig_t *config,
                                  float observation_imu_yaw, VisionAlignment_t *out);
#endif
