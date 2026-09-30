/**
  ******************************************************************************
  * @file    alg_ahrs.c
  * @brief   Mahony 姿态解算实现（四元数 + 加速度计重力反馈）
  *
  * 算法出自 Madgwick 的 Mahony AHRS 实现（x-io），与参考例程 Algorithm/MahonyAHRS.c
  * 数学上完全一致，差别只有三点（都有注释说明原因）：
  *   1. 状态收进 Ahrs_State_t 结构体，不用全局变量（以后多路 IMU 不会打架）
  *   2. dt 由调用方传入实测值，不写死 sampleFreq = 1000
  *   3. 归一化用 1/sqrtf() 而不是快速平方根倒数（H7 有硬件 VSQRT，更准且同样快）
  ******************************************************************************
  */

#include "alg_ahrs.h"
#include <math.h>

/** @brief 默认 2 × Kp，即 Kp = 0.5（与参考例程一致） */
#define ALG_AHRS_DEFAULT_TWO_KP   1.0f
/** @brief 默认 2 × Ki = 0：不做零偏估计 */
#define ALG_AHRS_DEFAULT_TWO_KI   0.0f
/** @brief dt 上限，秒。超过认为是断点/被打断造成的异常间隔，丢弃该次 */
#define ALG_AHRS_MAX_DT_S         0.1f

void alg_ahrs_init(Ahrs_State_t *ahrs)
{
    if (ahrs == NULL) {
        return;
    }

    /* 零姿态 = 没有旋转 */
    ahrs->quat[0] = 1.0f;
    ahrs->quat[1] = 0.0f;
    ahrs->quat[2] = 0.0f;
    ahrs->quat[3] = 0.0f;

    ahrs->integral_fb[0] = 0.0f;
    ahrs->integral_fb[1] = 0.0f;
    ahrs->integral_fb[2] = 0.0f;

    ahrs->two_kp      = ALG_AHRS_DEFAULT_TWO_KP;
    ahrs->two_ki      = ALG_AHRS_DEFAULT_TWO_KI;
    ahrs->sample_freq = 0.0f;
}

void alg_ahrs_set_gain(Ahrs_State_t *ahrs, float two_kp, float two_ki)
{
    if (ahrs == NULL) {
        return;
    }
    ahrs->two_kp = two_kp;
    ahrs->two_ki = two_ki;
}

void alg_ahrs_update(Ahrs_State_t *ahrs, const float gyro_radps[3],
                     const float accel[3], float dt_s)
{
    float q0, q1, q2, q3;
    float qa, qb, qc;
    float gx, gy, gz;
    float ax, ay, az;
    float half_vx, half_vy, half_vz;
    float half_ex, half_ey, half_ez;
    float recip;

    if ((ahrs == NULL) || (gyro_radps == NULL) || (accel == NULL)) {
        return;
    }
    /* dt 非法时直接丢弃：宁可少算一拍，也不能把垃圾灌进积分器 */
    if ((dt_s <= 0.0f) || (dt_s > ALG_AHRS_MAX_DT_S)) {
        return;
    }

    ahrs->sample_freq = 1.0f / dt_s;

    q0 = ahrs->quat[0];
    q1 = ahrs->quat[1];
    q2 = ahrs->quat[2];
    q3 = ahrs->quat[3];

    gx = gyro_radps[0];
    gy = gyro_radps[1];
    gz = gyro_radps[2];

    ax = accel[0];
    ay = accel[1];
    az = accel[2];

    /* ---------- 第 2~4 步：由重力方向求误差，再用 PI 修正角速度 ---------- */
    /* 加速度计读数全 0 视为无效，本轮只靠陀螺外推（避免除零产生 NaN） */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        /* 加速度归一化：只要方向，不要大小（所以运动加速度的大小不影响它） */
        recip = 1.0f / sqrtf((ax * ax) + (ay * ay) + (az * az));
        ax *= recip;
        ay *= recip;
        az *= recip;

        /* 由当前四元数推算"重力在机体系应该指向哪"
         * 即 R(机体系->导航系) 第三行的一半，乘 2 就是 2(q1q3-q0q2) 那一行 */
        half_vx = (q1 * q3) - (q0 * q2);
        half_vy = (q0 * q1) + (q2 * q3);
        half_vz = (q0 * q0) - 0.5f + (q3 * q3);

        /* 误差 = 实测方向 × 估计方向。
         * 叉积模长 = sin(夹角)，方向 = 转轴，正好是"要绕哪根轴转多少"，
         * 量纲与角速度一致，可以直接叠加到陀螺读数上 */
        half_ex = (ay * half_vz) - (az * half_vy);
        half_ey = (az * half_vx) - (ax * half_vz);
        half_ez = (ax * half_vy) - (ay * half_vx);

        if (ahrs->two_ki > 0.0f) {
            /* 积分项：误差长期同号说明存在固定零偏，积分起来把它抵消掉 */
            ahrs->integral_fb[0] += ahrs->two_ki * half_ex * dt_s;
            ahrs->integral_fb[1] += ahrs->two_ki * half_ey * dt_s;
            ahrs->integral_fb[2] += ahrs->two_ki * half_ez * dt_s;

            gx += ahrs->integral_fb[0];
            gy += ahrs->integral_fb[1];
            gz += ahrs->integral_fb[2];
        } else {
            /* Ki = 0 时保持积分器归零，防止以后开启 Ki 瞬间带入历史累积量 */
            ahrs->integral_fb[0] = 0.0f;
            ahrs->integral_fb[1] = 0.0f;
            ahrs->integral_fb[2] = 0.0f;
        }

        /* 比例项：按误差直接纠正 */
        gx += ahrs->two_kp * half_ex;
        gy += ahrs->two_kp * half_ey;
        gz += ahrs->two_kp * half_ez;
    }

    /* ---------- 第 1 步：四元数积分 ---------- */
    /* 先把公共因子 ½·dt 乘进去 */
    gx *= 0.5f * dt_s;
    gy *= 0.5f * dt_s;
    gz *= 0.5f * dt_s;

    /* qa/qb/qc 保存 q0/q1/q2 的旧值，q3 在算完前三个分量前不能被改 */
    qa = q0;
    qb = q1;
    qc = q2;

    q0 = q0 + ((-qb * gx) - (qc * gy) - (q3 * gz));
    q1 = q1 + ((qa * gx) + (qc * gz) - (q3 * gy));
    q2 = q2 + ((qa * gy) - (qb * gz) + (q3 * gx));
    q3 = q3 + ((qa * gz) + (qb * gy) - (qc * gx));

    /* ---------- 第 5 步：归一化 ---------- */
    /* 不归一化的话数值误差会让 |q| 慢慢偏离 1，姿态跟着漂 */
    recip = 1.0f / sqrtf((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
    q0 *= recip;
    q1 *= recip;
    q2 *= recip;
    q3 *= recip;

    ahrs->quat[0] = q0;
    ahrs->quat[1] = q1;
    ahrs->quat[2] = q2;
    ahrs->quat[3] = q3;
}

void alg_ahrs_get_euler(const Ahrs_State_t *ahrs, float euler_rad[3])
{
    float q0, q1, q2, q3;

    if ((ahrs == NULL) || (euler_rad == NULL)) {
        return;
    }

    q0 = ahrs->quat[0];
    q1 = ahrs->quat[1];
    q2 = ahrs->quat[2];
    q3 = ahrs->quat[3];

    euler_rad[AHRS_ROLL] = atan2f((2.0f * ((q0 * q1) + (q2 * q3))),
                                  (2.0f * ((q0 * q0) + (q3 * q3))) - 1.0f);

    /* asinf 的入参必须夹到 [-1, 1]：归一化后的舍入误差可能让它略微越界，
     * 越界会让 asinf 返回 NaN 并污染整条控制链 */
    euler_rad[AHRS_PITCH] = asinf(CLAMP(-2.0f * ((q1 * q3) - (q0 * q2)), -1.0f, 1.0f));

    euler_rad[AHRS_YAW] = atan2f((2.0f * ((q0 * q3) + (q1 * q2))),
                                 (2.0f * ((q0 * q0) + (q1 * q1))) - 1.0f);
}

void alg_ahrs_get_quat(const Ahrs_State_t *ahrs, float quat[4])
{
    if ((ahrs == NULL) || (quat == NULL)) {
        return;
    }

    quat[0] = ahrs->quat[0];
    quat[1] = ahrs->quat[1];
    quat[2] = ahrs->quat[2];
    quat[3] = ahrs->quat[3];
}
