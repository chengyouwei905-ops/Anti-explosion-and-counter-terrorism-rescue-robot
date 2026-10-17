#include "alg_motion_profile.h"
#include <math.h>
#include <stddef.h>

static float alg_motion_profile_fx(const MotionProfile_t *data, float time);
static MotionProfile_t alg_motion_profile = {
    .is_finished = true, .fx = alg_motion_profile_fx
};

static float alg_motion_profile_fx(const MotionProfile_t *data, float time)
{
    float speed;
    if (data->type == MOTION_PROFILE_TYPE_TRIANGULAR)/* 三角形加减速 */
    {
        if (time < data->t1) /* 加速阶段 */
        {
            speed = data->accel * time;
        }
        else        /* 减速阶段 */
        {
            speed = data->max_speed - data->decel * (time - data->t1);
        }
    }
    else /* data->type == MOTION_PROFILE_TYPE_TRAPEZOIDAL 梯形加减速 */
    {
        if (time < data->t1) /* 加速阶段 */
        {
            speed = data->accel * time;
        }
        else if (time < data->t2) /* 匀速阶段 */
        {
            speed = data->max_speed;
        }
        else /* 减速阶段 */
        {
            speed = data->max_speed - data->decel * (time - data->t2);
        }
    }
    return fmaxf(0.0f, fminf(speed, data->max_speed));
}

void alg_motion_profile_init(float distance, float max_speed, float max_accel, float max_decel)
{
    /* 每次初始化都清除上一次状态；零距离或非法参数保持停止。 */
    alg_motion_profile = (MotionProfile_t){.is_finished = true, .fx = alg_motion_profile_fx};
    if (!isfinite(distance) || !isfinite(max_speed) ||
        !isfinite(max_accel) || !isfinite(max_decel) ||
        distance <= 0.0f || max_speed <= 0.0f ||
        max_accel <= 0.0f || max_decel <= 0.0f)
    {
        return;
    }

    alg_motion_profile.accel = max_accel;
    alg_motion_profile.decel = max_decel;
    alg_motion_profile.target_distance = distance;
    float max_acc_time = max_speed / max_accel;
    float max_dec_time = max_speed / max_decel;
    float meet_acc_mini_distance = 0.5f * max_speed * (max_acc_time + max_dec_time);
    if(distance < meet_acc_mini_distance)   // 距离不足以完成达到设定最大速度后的加速和减速
    {
        /* 修改结构体参数从而构造函数 */
        alg_motion_profile.type = MOTION_PROFILE_TYPE_TRIANGULAR;
        alg_motion_profile.max_speed = sqrtf((2.0f * distance * max_accel * max_decel)/(max_accel + max_decel));//重定义在三角形vt图情况下的最大速度
        alg_motion_profile.t1 = alg_motion_profile.max_speed / max_accel;
        alg_motion_profile.t2 = alg_motion_profile.t1;
    }
    else                                    // 此时vt图为梯形，能够达到设定最大速度
    {
        alg_motion_profile.type = MOTION_PROFILE_TYPE_TRAPEZOIDAL;
        alg_motion_profile.max_speed = max_speed;
        alg_motion_profile.t1 = max_acc_time;
        alg_motion_profile.t2 = ((distance - 0.5f * alg_motion_profile.max_speed * (max_dec_time + max_acc_time) ) / max_speed) + max_acc_time;
    }

    float end_time = alg_motion_profile.t2 + alg_motion_profile.max_speed / max_decel;
    if (!isfinite(alg_motion_profile.max_speed) || !isfinite(end_time) ||
        alg_motion_profile.max_speed <= 0.0f || end_time <= 0.0f)
    {
        alg_motion_profile = (MotionProfile_t){.is_finished = true, .fx = alg_motion_profile_fx};
        return;
    }
    alg_motion_profile.is_finished = false;
}

float alg_motion_profile_get_speed(void)
{
    return alg_motion_profile.speed;
}

float alg_motion_profile_get_distance(void)
{
    return alg_motion_profile.distance;
}

bool alg_motion_profile_update(float addtime)
{
    if (alg_motion_profile.is_finished)
    {
        return true;
    }
    if (!alg_motion_profile.is_finished && isfinite(addtime) && addtime > 0.0f)
    {
        float end_time = alg_motion_profile.t2 + alg_motion_profile.max_speed / alg_motion_profile.decel;
        if (alg_motion_profile.time + addtime >= end_time)
        {
            alg_motion_profile.time = end_time;
            alg_motion_profile.speed = 0.0f;
            alg_motion_profile.distance = alg_motion_profile.target_distance;
            alg_motion_profile.is_finished = true;
            return true;
        }
        else
        {
            alg_motion_profile.time += addtime;
            alg_motion_profile.speed = alg_motion_profile.fx(&alg_motion_profile, alg_motion_profile.time);
        }
    }

    if(alg_motion_profile.type == MOTION_PROFILE_TYPE_TRIANGULAR)
    {
        alg_motion_profile.distance = 0.5f * alg_motion_profile.accel * alg_motion_profile.time * alg_motion_profile.time;
        if(alg_motion_profile.time > alg_motion_profile.t1)
        {
            float t = alg_motion_profile.time - alg_motion_profile.t1;
            alg_motion_profile.distance = 0.5f * alg_motion_profile.accel * alg_motion_profile.t1 * alg_motion_profile.t1;
            alg_motion_profile.distance += (alg_motion_profile.max_speed * t) - (0.5f * alg_motion_profile.decel * t * t);
        }
    }
    else
    {
        if(alg_motion_profile.time < alg_motion_profile.t1)
        {
            alg_motion_profile.distance = 0.5f * alg_motion_profile.accel * alg_motion_profile.time * alg_motion_profile.time;
        }
        else if(alg_motion_profile.time < alg_motion_profile.t2)
        {
            float t = alg_motion_profile.time - alg_motion_profile.t1;
            alg_motion_profile.distance = 0.5f * alg_motion_profile.accel * alg_motion_profile.t1 * alg_motion_profile.t1 + (alg_motion_profile.max_speed * t);
        }
        else
        {
            float t = alg_motion_profile.time - alg_motion_profile.t2;
            float dec_distance = 0.5f * alg_motion_profile.decel * t * t;
            float total_distance = 0.5f * alg_motion_profile.accel * alg_motion_profile.t1 * alg_motion_profile.t1 + (alg_motion_profile.max_speed * (alg_motion_profile.t2 - alg_motion_profile.t1));
            alg_motion_profile.distance = total_distance + (alg_motion_profile.max_speed * t) - dec_distance;
        }
    }

    return alg_motion_profile.is_finished;
}
