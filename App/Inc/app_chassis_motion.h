#ifndef __APP_CHASSIS_MOTION_H__
#define __APP_CHASSIS_MOTION_H__

#include <stdbool.h>
#include "alg_chassis_path.h"

typedef enum {
    MOTION_STATE_IDLE,
    MOTION_STATE_RUNNING
} Chassis_Motion_State_t;

typedef enum {
    MOTION_TYPE_LINEAR_MOVE,
    MOTION_TYPE_CIRCULAR_MOVE,
    MOTION_TYPE_PATH
} Chassis_Motion_Type_t;

typedef enum {
    MOTION_RESULT_IDLE = 0,
    MOTION_RESULT_RUNNING,
    MOTION_RESULT_DONE,
    MOTION_RESULT_CANCELLED,
    MOTION_RESULT_IMU_ERROR
} Chassis_Motion_Result_t;

typedef enum {
    MOTION_START_OK = 0,
    MOTION_START_BUSY,
    MOTION_START_INVALID,
    MOTION_START_NO_IMU
} Chassis_Motion_Start_t;

/* 所有角度为rad，逆时针为正。指令、update和closeloop均在主循环调用。
 * 单次执行一条指令；运行中收到的新指令不接收，先stop再启动新指令。
 * IMU必须已有有效姿态；非法参数不启动运动。
 * IDLE表示参考轨迹结束，尚不提供实际位置到位判定。
 */
void app_chassis_motion_init(void);

/* direction：相对开始时车头的平移方向；0向前，+PI/2向左。
 * 保持开始时航向。distance>=0(mm)，其余参数为mm/s、mm/s^2。
 */
void app_chassis_motion_linear_move(float distance, float direction,
                                    float max_speed, float accel, float decel);

/* orientation：true从当前yaw向左前方走圆弧，false向右前方；起始切线与当前车头一致。
 * radius>0(mm)：按radius*abs(angle)规划弧长，速度/加减速度为mm/s、mm/s^2。
 * radius>0时取abs(angle)为转角大小，转向由orientation决定；车头随圆弧连续转动。
 * radius==0：原地转过angle，忽略orientation；速度/加减速度为rad/s、rad/s^2。
 */
void app_chassis_motion_circular_move(float radius, float angle, bool orientation,
                                      float max_speed, float accel, float decel);

/* 只设置目标。外部每个控制节拍在本函数之后调用closeloop_update，含空闲时。 */
void app_chassis_motion_update(float addtime);
/* 取消规划、平移和角速度前馈归零，保持当前实测航向；无有效姿态则复位到0 rad目标。 */
void app_chassis_motion_stop(void);
Chassis_Motion_State_t app_chassis_motion_get_state(void);
Chassis_Motion_Result_t app_chassis_motion_get_result(void);

/* 连续前进路径：直线与带曲率过渡的转弯，整组零速起止，内部边界不停。
 * 转弯保留名义圆弧的终点/转角，但实际半径随曲率变化。
 * 参数会复制；忙时拒绝且不影响在途路径。所有调用仍在主循环。
 * limits.max_omega应低于闭环限幅，为航向误差修正保留余量。
 */
Chassis_Motion_Start_t app_chassis_motion_path_start(const AlgPathSegment_t *segments,
                                                    uint8_t count,
                                                    const AlgPathLimits_t *limits);
/* 参考状态/预计总时长，不是实际定位。路径完成后仍可读，stop/init后失效。 */
bool app_chassis_motion_path_get_sample(AlgPathSample_t *out);

#endif /* __APP_CHASSIS_MOTION_H__ */
