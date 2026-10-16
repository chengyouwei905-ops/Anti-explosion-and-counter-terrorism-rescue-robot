#ifndef __ALG_MOTION_PROFILE_H__
#define __ALG_MOTION_PROFILE_H__

#include <stdbool.h>

typedef enum
{
    MOTION_PROFILE_TYPE_TRAPEZOIDAL, /**< 梯形速度曲线 */
    MOTION_PROFILE_TYPE_TRIANGULAR    /**< 三角形速度曲线 */
} MotionProfile_Fx_Type_t;

typedef struct
{
    float max_speed; /**< 最大速度，单位mm/s */
    float accel; /**< 加速度，单位mm/s^2 */
    float decel; /**< 减速度，单位mm/s^2 */
    MotionProfile_Fx_Type_t type; /**< vt图是三角形还是梯形 */
    float speed;     /**< 当前速度，单位mm/s */
    float time;      /**< 当前时间，单位s */

    float t1;       /**< 加速结束时间点，单位s */
    float t2;       /**< 减速开始时间点，单位s；三角形时等于t1 */

    bool is_finished; /**< 是否完成，true为完成，false为未完成 */
} MotionProfile_t;

/** @brief 从静止规划到静止；零距离或非法参数保持零速度、已完成状态。 */
void alg_motion_profile_init(float distance, float max_speed, float max_accel, float max_decel);
/** @brief addtime为时间增量(s)，非法增量不推进；speed输出参考速度，可为NULL。
 *  @return true仅表示速度规划结束（此时输出零速度），不代表底盘实际到位。
 */
bool alg_motion_profile_update(float addtime, float *speed);


#endif /* __ALG_MOTION_PROFILE_H__ */
