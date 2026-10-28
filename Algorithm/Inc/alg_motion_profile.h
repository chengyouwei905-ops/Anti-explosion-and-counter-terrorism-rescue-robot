#ifndef __ALG_MOTION_PROFILE_H__
#define __ALG_MOTION_PROFILE_H__

#include <stdbool.h>

typedef enum
{
    MOTION_PROFILE_TYPE_TRAPEZOIDAL, /**< 梯形速度曲线 */
    MOTION_PROFILE_TYPE_TRIANGULAR    /**< 三角形速度曲线 */
} MotionProfile_Fx_Type_t;

typedef struct MotionProfile MotionProfile_t;
typedef float (*MotionProfile_Fx_t)(const MotionProfile_t *data, float time);

struct MotionProfile
{
    float max_speed; /**< 最大速度，单位mm/s */
    float accel; /**< 加速度，单位mm/s^2 */
    float decel; /**< 减速度，单位mm/s^2 */
    MotionProfile_Fx_Type_t type; /**< vt图是三角形还是梯形 */
    float distance;  /**< 当前位移量，单位mm */
    float target_distance; /**< 总规划位移；原地转动时各字段使用rad制 */
    float speed;     /**< 当前速度，单位mm/s */
    float time;      /**< 当前时间，单位s */

    float t1;       /**< 加速结束时间点，单位s */
    float t2;       /**< 减速开始时间点，单位s；三角形时等于t1 */

    bool is_finished; /**< 是否完成，true为完成，false为未完成 */
    MotionProfile_Fx_t fx; /**< 输入曲线参数和已运行时间，返回参考速度 */
};

/** @brief 从静止规划到静止；零距离或非法参数保持零速度、已完成状态。 */
void alg_motion_profile_init(float distance, float max_speed, float max_accel, float max_decel);
float alg_motion_profile_get_speed(void);
/** @brief 参考位移（速度曲线积分），不是编码器测得的实际位移。 */
float alg_motion_profile_get_distance(void);
/** @brief addtime为时间增量(s)，非法增量不推进。
 *  @return true仅表示速度规划结束（此时输出零速度），不代表底盘实际到位。
 */
bool alg_motion_profile_update(float addtime);

/* Independent, allocation-free profile used by the continuous path planner.
 * Units may be mm/s or rad/s, but must be consistent. Boundary speeds are
 * nonnegative. Unlike the legacy singleton, multiple plans can coexist.
 */
typedef struct {
    float distance;
    float start_speed;
    float end_speed;
    float peak_speed;
    float accel;
    float decel;
    float accel_time;
    float cruise_time;
    float duration;
    float accel_distance;
    float cruise_distance;
} MotionProfilePlan_t;

/* Rejects infeasible boundary speeds rather than silently changing them. */
bool alg_motion_profile_plan(MotionProfilePlan_t *out, float distance,
                             float start_speed, float end_speed, float max_speed,
                             float accel, float decel);
bool alg_motion_profile_sample(const MotionProfilePlan_t *plan, float time_s,
                               float *distance, float *speed, float *accel);


#endif /* __ALG_MOTION_PROFILE_H__ */
