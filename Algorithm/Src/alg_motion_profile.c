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

bool alg_motion_profile_plan(MotionProfilePlan_t *out, float distance,
                             float start_speed, float end_speed, float max_speed,
                             float accel, float decel)
{
    if (out == NULL) {
        return false;
    }
    *out = (MotionProfilePlan_t){0};
    if (!isfinite(distance) || distance <= 0.0f ||
        !isfinite(start_speed) || start_speed < 0.0f ||
        !isfinite(end_speed) || end_speed < 0.0f ||
        !isfinite(max_speed) || max_speed <= 0.0f ||
        !isfinite(accel) || accel <= 0.0f ||
        !isfinite(decel) || decel <= 0.0f ||
        start_speed > max_speed || end_speed > max_speed) {
        return false;
    }
    /* Double precision keeps the feasibility check stable at a lookahead bound. */
    double v0sq = (double)start_speed * start_speed;
    double v1sq = (double)end_speed * end_speed;
    double tolerance = 1.0e-5 * distance + 1.0e-4;
    if ((v1sq - v0sq) / (2.0 * accel) > distance + tolerance ||
        (v0sq - v1sq) / (2.0 * decel) > distance + tolerance) {
        return false;
    }
    double peak_sq = (2.0 * accel * decel * distance + decel * v0sq + accel * v1sq) /
                     ((double)accel + decel);
    double peak = fmin((double)max_speed, sqrt(peak_sq));
    peak = fmax(peak, fmax(start_speed, end_speed));
    double sa = fmax(0.0, (peak * peak - v0sq) / (2.0 * accel));
    double sd = fmax(0.0, (peak * peak - v1sq) / (2.0 * decel));
    double sc = fmax(0.0, distance - sa - sd);
    double ta = (peak - start_speed) / accel;
    double tc = sc / peak;
    double duration = ta + tc + (peak - end_speed) / decel;
    if (!isfinite(duration) || duration <= 0.0 || duration > 3600.0) {
        return false;
    }
    *out = (MotionProfilePlan_t){
        .distance = distance, .start_speed = start_speed, .end_speed = end_speed,
        .peak_speed = (float)peak, .accel = accel, .decel = decel,
        .accel_time = (float)ta, .cruise_time = (float)tc,
        .duration = (float)duration, .accel_distance = (float)sa,
        .cruise_distance = (float)sc
    };
    return true;
}

bool alg_motion_profile_sample(const MotionProfilePlan_t *plan, float time_s,
                               float *distance, float *speed, float *accel)
{
    if (plan == NULL || distance == NULL || speed == NULL || accel == NULL ||
        !isfinite(time_s) || time_s < 0.0f || plan->duration <= 0.0f) {
        return false;
    }
    if (time_s >= plan->duration) {
        *distance = plan->distance;
        *speed = plan->end_speed;
        *accel = 0.0f;
    } else if (time_s < plan->accel_time) {
        *distance = plan->start_speed * time_s + 0.5f * plan->accel * time_s * time_s;
        *speed = plan->start_speed + plan->accel * time_s;
        *accel = plan->accel;
    } else if (time_s < plan->accel_time + plan->cruise_time) {
        float t = time_s - plan->accel_time;
        *distance = plan->accel_distance + plan->peak_speed * t;
        *speed = plan->peak_speed;
        *accel = 0.0f;
    } else {
        float t = time_s - plan->accel_time - plan->cruise_time;
        *distance = plan->accel_distance + plan->cruise_distance +
                    plan->peak_speed * t - 0.5f * plan->decel * t * t;
        *speed = plan->peak_speed - plan->decel * t;
        *accel = -plan->decel;
    }
    *distance = fmaxf(0.0f, fminf(*distance, plan->distance));
    *speed = fmaxf(0.0f, fminf(*speed, plan->peak_speed));
    return true;
}
