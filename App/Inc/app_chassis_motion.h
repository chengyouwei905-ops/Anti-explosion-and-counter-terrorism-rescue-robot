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
    MOTION_TYPE_PATH,
    MOTION_TYPE_ARC_TRANSLATION,
    MOTION_TYPE_PATH_TRANSLATION,
    MOTION_TYPE_LINEAR_DISTANCE
} Chassis_Motion_Type_t;

typedef enum {
    MOTION_RESULT_IDLE = 0,
    MOTION_RESULT_RUNNING,
    MOTION_RESULT_DONE,
    MOTION_RESULT_CANCELLED,
    MOTION_RESULT_IMU_ERROR,
    MOTION_RESULT_ODOM_ERROR,
    MOTION_RESULT_STALLED,
    MOTION_RESULT_TIMEOUT
} Chassis_Motion_Result_t;

typedef enum {
    MOTION_START_OK = 0,
    MOTION_START_BUSY,
    MOTION_START_INVALID,
    MOTION_START_NO_IMU,
    MOTION_START_NO_ODOM
} Chassis_Motion_Start_t;

/* 所有角度为rad，逆时针为正。指令、update和closeloop均在主循环调用。
 * 单次执行一条指令；运行中收到的新指令不接收，先stop再启动新指令。
 * IMU必须已有有效姿态；非法参数不启动运动。
 * 时间规划接口的IDLE仅表示参考轨迹结束；里程接口以编码器累计路程到量判断完成。
 */
void app_chassis_motion_init(void);

/* direction：相对开始时车头的平移方向；0向前，+PI/2向左。
 * 保持开始时航向。distance>=0(mm)，其余参数为mm/s、mm/s^2。
 */
void app_chassis_motion_linear_move(float distance, float direction,
                                    float max_speed, float accel, float decel);

/* 仅用于视觉二次纠偏的里程直线移动。方向、单位、IMU保持航向与linear_move一致。
 * 记录get_distance起始值，不清零全局里程；累计路程差达到distance-容差时停止。
 * distance不超过容差时直接DONE，不启动平移。
 * 按剩余路程减速，未走够不会因速度规划时间结束而DONE。
 * 累计路程没有X/Y方向，侧滑/往返也会计入；完成后仍需视觉复测。
 * 返回NO_ODOM表示尚无有效/新鲜编码器反馈；在途反馈过期、清零/回退、
 * 无进展和总超时分别以ODOM_ERROR、STALLED、TIMEOUT结束，不作为成功。
 */
#define APP_CHASSIS_MOTION_DISTANCE_TOLERANCE_MM 20.0f
#define APP_CHASSIS_MOTION_DISTANCE_MIN_SPEED 50.0f /* mm/s，需按实车低速能力标定 */
#define APP_CHASSIS_MOTION_ODOM_MAX_AGE_MS 500U
#define APP_CHASSIS_MOTION_DISTANCE_STALL_S 2.0f
#define APP_CHASSIS_MOTION_DISTANCE_TIMEOUT_MS 30000U
Chassis_Motion_Start_t app_chassis_motion_linear_move_distance(float distance, float direction,
                                                              float max_speed, float accel,
                                                              float decel);

/* orientation：true从当前yaw向左前方走圆弧，false向右前方；起始切线与当前车头一致。
 * radius>0(mm)：按radius*abs(angle)规划弧长，速度/加减速度为mm/s、mm/s^2。
 * radius>0时取abs(angle)为转角大小，转向由orientation决定；车头随圆弧连续转动。
 * radius==0：原地转过angle，忽略orientation；速度/加减速度为rad/s、rad/s^2。
 */
void app_chassis_motion_circular_move(float radius, float angle, bool orientation,
                                      float max_speed, float accel, float decel);

/* 定半径圆弧平移，车头保持指令开始时的实测航向，不随圆弧转动。
 * radius>0(mm)；angle为带符号扫角(rad)，正值使平移方向向左弯，负值向右弯。
 * direction是起始切线相对开始时车头的方向，0向前，+PI/2向左。
 * 例如R200、angle=+PI/2、direction=0：终点在起点左200mm、前200mm。
 * 速度/加减速度为切向mm/s、mm/s^2，沿弧长从零速加速并减速到零。
 * angle=0立即DONE；其余不可规划参数返回INVALID。忙/参数错/IMU无效均不覆盖在途指令。
 * 仍需周期调用update及closeloop_update；DONE仅表示参考轨迹结束。
 */
Chassis_Motion_Start_t app_chassis_motion_arc_translate(float radius, float angle,
                                                       float direction, float max_speed,
                                                       float accel, float decel);

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
/* 相同连续路径几何/速度规划，整组保持开始时车头朝向，仅平移方向随路径变化。
 * 第一段起始切线仍是开始时车头方向；后续直线沿上一圆弧的出口切线移动。
 * 圆弧仍有曲率过渡，并非严格定半径。limits继续约束路径切线的变化。
 * 注意：改变整组车头姿态后，观察姿态/停车参考及后续组的方向需重新核对。
 */
Chassis_Motion_Start_t app_chassis_motion_path_translate_start(const AlgPathSegment_t *segments,
                                                              uint8_t count,
                                                              const AlgPathLimits_t *limits);
/* 参考状态/预计总时长，不是实际定位。路径完成后仍可读，stop/init后失效。
 * 平移路径的yaw/omega/angular_accel描述路径切线，车头目标保持开始时yaw。
 */
bool app_chassis_motion_path_get_sample(AlgPathSample_t *out);

#endif /* __APP_CHASSIS_MOTION_H__ */
