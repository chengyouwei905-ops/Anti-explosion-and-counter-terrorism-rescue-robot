/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   IMU 应用层实现
  *
  * 每轮 poll 做的事：
  *   1. 用 DWT 周期计数器实测与上次的间隔 dt（分辨率约 15.6ns，不是写死 1ms）
  *   2. bsp_imu_read() 读加速度/角速度/温度
  *   3. 角速度减掉开机标定的零偏
  *   4. alg_ahrs_update() 做 Mahony 解算
  *
  * 关于零偏：本层在开机时静止采样一次，把陀螺零偏量出来直接减掉，
  *          所以 alg_ahrs 的积分增益 two_ki 保持 0 —— 不让积分器去猜零偏，
  *          因为它分不清"零偏"和"真实的持续转动"，容易在急加速时被带偏。
  ******************************************************************************
  */

#include "app_imu.h"
#include "bsp_imu.h"
#include "bsp_dwt.h"

/** @brief 开机零偏标定的采样次数（配合下面的 1ms 间隔，约 1 秒） */
#define APP_IMU_CALI_SAMPLES      1000U
/** @brief 标定采样间隔（ms），BMI088 陀螺 ODR 是 1kHz，刚好每次拿到新数据 */
#define APP_IMU_CALI_INTERVAL_MS  1U
/** @brief 标定期间允许的零偏抖动峰峰值（rad/s）。超过就说明板子在被晃，标定作废 */
#define APP_IMU_CALI_MAX_SPREAD   0.05f

/**
  * @brief 两次采样之间的最小间隔（秒）
  * @note  陀螺 ODR 是 1000Hz，比 1ms 更密地读只会拿到重复样本、白耗 SPI 带宽。
  *        想把解算频率提到 1kHz 以上：把这里调小，**同时**改 bsp_imu.c 里
  *        陀螺的 ODR（GYRO_BANDWIDTH 那一条）
  */
#define APP_IMU_MIN_DT_S          0.001f

static bool          s_ready;
static bool          s_calibrated;
static Ahrs_State_t  s_ahrs;
static float         s_gyro_bias[3];
static float         s_gyro[3];      /**< 已减零偏的角速度，rad/s */
static float         s_accel[3];     /**< 加速度，m/s^2 */
static float         s_temp;
static uint32_t      s_last_cycle;   /**< 上次采样的 DWT 周期计数 */
static uint32_t      s_err_count;

/**
  * @brief 开机陀螺零偏标定：静止采样取平均
  * @note  同时用峰峰值判断板子是否真的静止，静止是标定的前提
  */
static ret_code_t imu_calibrate_gyro(void)
{
    bsp_imu_data_t data;
    float       sum[3]  = { 0.0f, 0.0f, 0.0f };
    float       vmin[3] = { 0.0f, 0.0f, 0.0f };
    float       vmax[3] = { 0.0f, 0.0f, 0.0f };
    uint32_t    i;
    uint32_t    k;
    ret_code_t  ret;

    for (i = 0U; i < APP_IMU_CALI_SAMPLES; i++) {
        ret = bsp_imu_read(&data);
        if (ret != RET_OK) {
            return ret;
        }

        for (k = 0U; k < 3U; k++) {
            float v = data.gyro_radps[k];

            if (i == 0U) {
                vmin[k] = v;
                vmax[k] = v;
            } else {
                if (v < vmin[k]) { vmin[k] = v; }
                if (v > vmax[k]) { vmax[k] = v; }
            }
            sum[k] += v;
        }

        HAL_Delay(APP_IMU_CALI_INTERVAL_MS);
    }

    for (k = 0U; k < 3U; k++) {
        if ((vmax[k] - vmin[k]) > APP_IMU_CALI_MAX_SPREAD) {
            return RET_ERROR;   /* 标定期间板子在动，这一组数据不能用 */
        }
        s_gyro_bias[k] = sum[k] / (float)APP_IMU_CALI_SAMPLES;
    }

    return RET_OK;
}

int app_imu_init(SPI_HandleTypeDef *hspi)
{
    ret_code_t ret;

    s_ready      = false;
    s_calibrated = false;
    s_temp       = 0.0f;
    s_err_count  = 0U;

    for (uint32_t k = 0U; k < 3U; k++) {
        s_gyro_bias[k] = 0.0f;
        s_gyro[k]      = 0.0f;
        s_accel[k]     = 0.0f;
    }

    /* 1) 跑 BSP：SPI2+DMA 通信、BMI088 初始化（含 WHO_AM_I 校验）*/
    ret = bsp_imu_init(hspi);
    if (ret != RET_OK) {
        return ret;
    }

    /* 2) 复位姿态解算：q = [1,0,0,0]，即"此刻"为姿态零点 */
    alg_ahrs_init(&s_ahrs);
    /* two_ki 保持默认 0：零偏已由下面的标定处理，不需要积分器再兜底 */

    /* 3) 启动 DWT 周期计数器，作为 dt 的时间基准 */
    /*    ⚠️ 必须在 SystemClock_Config() 之后，它要用 SystemCoreClock 算换算比例 */
    bsp_dwt_init();
    if (!bsp_dwt_is_ready()) {
        return RET_ERROR;
    }

    /* 4) 标定陀螺零偏（失败不致命，bias 保持 0）*/
    s_calibrated = (imu_calibrate_gyro() == RET_OK);

    s_last_cycle = bsp_dwt_get_cycles();
    s_ready      = true;

    return RET_OK;
}

int app_imu_poll(void)
{
    bsp_imu_data_t data;
    uint32_t       now_cycle;
    float          dt_s;
    uint32_t       k;
    ret_code_t     ret;

    if (!s_ready) {
        return RET_NOTREADY;
    }

    /* ---- 1) 用 DWT 实测 dt（分辨率 15.6ns，远优于 HAL_GetTick 的 1ms）----
     *        无符号减法能自然处理 CYCCNT 回绕，只要单次间隔 < 67s */
    now_cycle = bsp_dwt_get_cycles();
    dt_s      = bsp_dwt_cycles_to_s(now_cycle - s_last_cycle);
    if (dt_s < APP_IMU_MIN_DT_S) {
        return RET_BUSY;        /* 比传感器 ODR 还密地读，只会拿到重复样本 */
    }
    s_last_cycle = now_cycle;

    /* ---- 2) 读传感器 ---- */
    ret = bsp_imu_read(&data);
    if (ret != RET_OK) {
        s_err_count++;
        return ret;     /* 读失败就保持上一次的姿态，绝不把垃圾喂进解算器 */
    }

    /* ---- 3) 减零偏 ---- */
    for (k = 0U; k < 3U; k++) {
        s_gyro[k]  = data.gyro_radps[k] - s_gyro_bias[k];
        s_accel[k] = data.accel_mps2[k];
    }
    s_temp = data.temp_c;

    /* ---- 4) 姿态解算：dt 用实测值，所以主循环节奏不齐也不影响积分正确性 ---- */
    alg_ahrs_update(&s_ahrs, s_gyro, s_accel, dt_s);

    return RET_OK;
}

void app_imu_get_euler(float euler_rad[3])
{
    if (euler_rad == NULL) {
        return;
    }
    if (!s_ready) {
        euler_rad[AHRS_ROLL]  = 0.0f;
        euler_rad[AHRS_PITCH] = 0.0f;
        euler_rad[AHRS_YAW]   = 0.0f;
        return;
    }

    alg_ahrs_get_euler(&s_ahrs, euler_rad);
}

void app_imu_get_gyro(float gyro_radps[3])
{
    if (gyro_radps == NULL) {
        return;
    }
    for (uint32_t k = 0U; k < 3U; k++) {
        gyro_radps[k] = s_gyro[k];
    }
}

void app_imu_get_accel(float accel_mps2[3])
{
    if (accel_mps2 == NULL) {
        return;
    }
    for (uint32_t k = 0U; k < 3U; k++) {
        accel_mps2[k] = s_accel[k];
    }
}

float app_imu_get_temp(void)
{
    return s_temp;
}

bool app_imu_is_ready(void)
{
    return s_ready;
}

bool app_imu_is_calibrated(void)
{
    return s_calibrated;
}

uint32_t app_imu_get_error_count(void)
{
    return s_err_count;
}
