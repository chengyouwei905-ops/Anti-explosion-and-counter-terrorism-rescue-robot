/**
  ******************************************************************************
  * @file    alg_ahrs.h
  * @brief   Mahony 姿态解算（四元数 + 加速度计重力反馈），纯计算、不碰硬件
  *
  * 原理（五步）：
  *   1. 陀螺积分          q += ½ · q ⊗ ω · dt
  *   2. 由 q 推算重力方向  v = R(body->earth) 第三行 / 2
  *   3. 与加速度计实测方向叉乘求误差  e = a × v
  *   4. PI 反馈修正角速度  ω += twoKp·e + ∫ twoKi·e·dt
  *   5. 归一化 q
  *
  * ⚠️ 输入单位约定：
  *      gyro_radps 必须是 **rad/s**（改量程要同步改 BSP 里的换算系数）
  *      accel      单位无所谓（内部会归一化），只取其方向
  *
  * ⚠️ 已知局限：
  *      - 没有磁力计，**yaw 没有绝对参考**。上电瞬间 yaw = 0，之后靠陀螺积分，
  *        会缓慢漂移；roll/pitch 则被加速度计长期拉回真实值。
  *      - 急加速/剧烈振动时加速度计测的是"重力 + 运动加速度"，反馈会把运动
  *        加速度误当姿态误差处理，倾角会被带偏。
  *
  * 注：本文件有意不使用磁力计版本（本板没有磁力计），只保留纯 IMU 解算。
  ******************************************************************************
  */

#ifndef __ALG_AHRS_H__
#define __ALG_AHRS_H__

#include <stdbool.h>
#include "common_def.h"

/** @brief 欧拉角下标，euler_rad[] 的元素顺序 */
typedef enum {
    AHRS_ROLL  = 0,   /**< 横滚 φ，绕 x 轴，右倾为正 */
    AHRS_PITCH = 1,   /**< 俯仰 θ，绕 y 轴，抬头为正 */
    AHRS_YAW   = 2,   /**< 偏航 ψ，绕 z 轴，俯视逆时针（左转）为正 */
} Ahrs_Euler_ID_t;

/** @brief 姿态解算状态 */
typedef struct {
    float quat[4];          /**< 姿态四元数 [w, x, y, z]：机体系 -> 导航系 */
    float integral_fb[3];   /**< 积分反馈项（等效陀螺零偏估计），单位 rad/s */
    float two_kp;           /**< 2 × 比例增益，见 alg_ahrs_set_gain() */
    float two_ki;           /**< 2 × 积分增益，见 alg_ahrs_set_gain() */
    float sample_freq;      /**< 最近一次解算的等效采样率 Hz，只读，可用于观察 */
} Ahrs_State_t;

/**
  * @brief  初始化姿态解算状态（四元数复位为"零姿态"、增益取默认值）
  * @param  ahrs 状态对象
  */
void alg_ahrs_init(Ahrs_State_t *ahrs);

/**
  * @brief  设置增益
  * @param  two_kp 2 × Kp，默认 1.0f（即 Kp = 0.5）。越大跟随越快、抗振越差
  * @param  two_ki 2 × Ki，默认 0.0f。设为 0 表示不做零偏估计（不会积分饱和，
  *                但陀螺零偏会让倾角长期存在一个固定偏差，典型值 0.005f 左右可消掉）
  */
void alg_ahrs_set_gain(Ahrs_State_t *ahrs, float two_kp, float two_ki);

/**
  * @brief  一次姿态解算（每个控制周期调用一次）
  * @param  ahrs        状态对象
  * @param  gyro_radps  角速度 [x, y, z]，单位 rad/s
  * @param  accel       加速度 [x, y, z]，单位任意；传全 0 表示本轮不可信，跳过反馈
  * @param  dt_s        距上次调用的时间间隔，单位 **秒**，必须由调用方用实测时间算
  * @note   dt_s <= 0 或过大（> 0.1s，说明被打断过）时会直接丢弃本次调用
  */
void alg_ahrs_update(Ahrs_State_t *ahrs, const float gyro_radps[3],
                     const float accel[3], float dt_s);

/**
  * @brief  取欧拉角
  * @param  ahrs      状态对象
  * @param  euler_rad 输出 [roll, pitch, yaw]，单位 **弧度**
  *                   转角度制乘 57.29578f；yaw 范围 (-π, π]，跨 ±π 会跳变
  */
void alg_ahrs_get_euler(const Ahrs_State_t *ahrs, float euler_rad[3]);

/**
  * @brief  取四元数（需要做插值/发给上位机时用）
  * @param  quat 输出 [w, x, y, z]，注意 q 与 -q 表示同一姿态
  */
void alg_ahrs_get_quat(const Ahrs_State_t *ahrs, float quat[4]);

#endif /* __ALG_AHRS_H__ */
