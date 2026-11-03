/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   IMU 应用层实现
  *
  * 三块职责，互不耦合：
  *   1. 节拍：app_imu_poll() 用 DWT 实测 dt 自限速，**调用频率不敏感**
  *   2. 解算：bsp_imu_read() -> 减零偏及温漂 -> alg_ahrs_update()
  *   3. 发布：每拍把结果写进双精度快照 s_snap；读取侧 app_imu_get_xxx() 复制快照
  *
  * 发布/读取约定（刻意保持最简单）：
  *   - 只有 app_imu_poll() 写 s_snap，其它任何地方一律只读
  *   - 读到的可能是上一拍的旧值（最多旧一个 poll 周期，约 1ms）——本项目可以接受
  *   - double 是 64 位，32 位 MCU 不能保证单次读写原子性。
  *     发布和复制快照时短暂关中断，并恢复原 PRIMASK；计算和传感器通信不在临界区。
  *   - 主循环/普通中断均可读取，不自旋等待；NMI/HardFault 中不要调用读取接口。
  *   - 快照里的 yaw 是**连续角**：跨 ±180° 不跳变（179° -> 180° -> 181°），会一直
  *     累加，不再限制在 (-180°, 180°]；roll / pitch 仍是 (-180°, 180°]
  *
  * 调用频率约定（app_imu_poll）：
  *   - 太快（间隔 < APP_IMU_MIN_DT_S）：返回 RET_IDLE，不算错误，也不会丢数据
  *     （陀螺 ODR 1kHz，比它更密地读只会拿到重复样本、白耗 SPI 带宽）
  *   - 太慢：dt 是实测值，积分本身照样正确；只有超过 APP_IMU_MAX_DT_S 才按上限
  *     夹住，防止断点恢复 / 长时间阻塞那种异常间隔把积分带飞
  *   - 姿态只在"读传感器失败"时才停止更新，与调用节奏无关
  *
  * 关于零偏：本层在开机时静止采样一次，把陀螺零偏量出来直接减掉，
  *          所以 alg_ahrs 的积分增益 two_ki 保持 0 —— 不让积分器去猜零偏，
  *          因为它分不清"零偏"和"真实的持续转动"，容易在急加速时被带偏。
  ******************************************************************************
  */

#include "app_imu.h"
#include "bsp_imu.h"
#include "bsp_dwt.h"

/** @brief 开机零偏标定的采样次数（配合下面的 1ms 间隔，至少约 8 秒） */
#define APP_IMU_CALI_SAMPLES      8000U     //此处给8s的采样时间，保证低零飘
/** @brief 标定采样间隔（ms），BMI088 陀螺 ODR 是 1kHz，刚好每次拿到新数据 */
#define APP_IMU_CALI_INTERVAL_MS  1U
/** @brief 标定期间允许的零偏抖动峰峰值（rad/s）。超过就说明板子在被晃，标定作废 */
#define APP_IMU_CALI_MAX_SPREAD   0.05

/**
  * @brief 三轴零偏温度系数，单位 (rad/s)/℃；0 表示禁用该轴，按本机静止实测值填写
  * @note  K = (高温零偏 - 低温零偏) / (高温 - 低温)，各轴可正可负。
  *        补偿时减去 K * (当前温度 - 标定基准温度)；若三轴共用一个 K，填相同值。
  */
#ifndef APP_IMU_GYRO_TEMP_COEFF_X
#define APP_IMU_GYRO_TEMP_COEFF_X  0.0
#endif
#ifndef APP_IMU_GYRO_TEMP_COEFF_Y
#define APP_IMU_GYRO_TEMP_COEFF_Y  0.0
#endif
#ifndef APP_IMU_GYRO_TEMP_COEFF_Z
#define APP_IMU_GYRO_TEMP_COEFF_Z  0.000004
#endif

/**
  * @brief 两次采样之间的最小间隔（秒）—— 限速用，比它更密地读只会拿到重复样本
  * @note  陀螺 ODR 是 1000Hz。想把解算频率提到 1kHz 以上：把这里调小，**同时**
  *        改 bsp_imu.c 里陀螺的 ODR（GYRO_BANDWIDTH 那一条）
  */
#define APP_IMU_MIN_DT_S          0.001
/**
  * @brief 单拍 dt 上限（秒）—— 夹取用
  * @note  实测 dt 超过它就按它算。目的是兜住断点恢复 / 长时间阻塞那种异常间隔，
  *        而不是"因为调用得慢就不更新姿态"：1s 以内都按真实 dt 积分，不会冻结。
  */
#define APP_IMU_MAX_DT_S          1.0

/** @brief π / 2π：把两拍之间的 yaw 增量归一化到 (-π, π] 用 */
#define IMU_PI                    3.14159265358979323846
#define IMU_TWO_PI                6.28318530717958647692

/**
  * @brief 发布给读取侧的姿态快照
  * @note  volatile 保证实际访问内存；64 位字段的一致性由短临界区保证。
  */
typedef struct
{
    double euler[3];    /**< [roll, pitch, yaw]，弧度；**yaw 是连续角**，见下 */
    double quat[4];     /**< [w, x, y, z] */
    double gyro[3];     /**< 已减零偏及温漂的角速度，rad/s */
    double accel[3];    /**< 加速度，m/s^2 */
    double temp_c;      /**< 芯片温度，℃ */
    double rate_hz;     /**< 最近一拍的等效采样率，Hz */
    bool  valid;       /**< false = 还没有有效姿态 */
} app_imu_snapshot_t;

static bool          s_ready;
static bool          s_calibrated;
static Ahrs_State_t  s_ahrs;
static double         s_gyro_bias[3];       /**< 基准温度对应的三轴零偏，rad/s */
static double         s_gyro_ref_temp_c;    /**< 标定全部有效样本的平均温度，℃ */
static double         s_gyro_cali_start_temp_c; /**< 标定第一个有效样本的温度，℃ */
static double         s_gyro_cali_end_temp_c;   /**< 标定最后一个有效样本的温度，℃ */
static double         s_gyro_cali_mean_temp_c;  /**< 标定全部样本的平均温度，℃ */
static const double   s_gyro_temp_coeff[3] = {
    APP_IMU_GYRO_TEMP_COEFF_X,
    APP_IMU_GYRO_TEMP_COEFF_Y,
    APP_IMU_GYRO_TEMP_COEFF_Z
};
static uint32_t      s_last_cycle;   /**< 上次采样的 DWT 周期计数 */
static uint32_t      s_err_count;

/**
  * @brief 连续 yaw 的累计状态
  * @note  alg_ahrs 给出的 yaw 被包在 (-π, π] 内，跨 ±180° 会跳变；本层在发布前把它
  *        累计成连续角（179° -> 180° -> 181°）。累计是有状态的，所以只能在唯一的
  *        写入者 app_imu_poll() 里做，读取侧才能保持无状态
  * @note  这么做依赖一个前提：两拍之间的转动小于 180°（正常 1ms 节拍下最多约 0.5°，
  *        完全安全）。只有在"长时间不调 poll（>0.3s）且期间转过半圈以上"时，
  *        增量才会判错方向 —— 属于极端情况，正常使用不会碰到
  */
static double         s_yaw_cont;         /**< 连续 yaw（弧度），会一直累加 */
static double         s_yaw_prev;         /**< 上一拍解算出的 (-π, π] yaw */
static bool          s_yaw_cont_ready;   /**< false = 还没有基准拍 */

static volatile app_imu_snapshot_t s_snap;   /**< 最新一次解算结果，读取侧直接读 */

/* 只保护快照搬运。保存/恢复 PRIMASK，避免在原本关中断的调用者中意外开中断。 */
static void imu_publish_snapshot(const app_imu_snapshot_t *snapshot)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_snap = *snapshot;
    __set_PRIMASK(primask);
}

static bool imu_read_vector(const volatile double *values, double *out, uint32_t count)
{
    if (out == NULL) {
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool valid = s_snap.valid;
    for (uint32_t k = 0U; k < count; k++) {
        out[k] = valid ? values[k] : 0.0;
    }
    __set_PRIMASK(primask);
    return valid;
}

static double imu_read_scalar(const volatile double *value)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    double result = s_snap.valid ? *value : 0.0;
    __set_PRIMASK(primask);
    return result;
}

/**
  * @brief 开机陀螺零偏标定：静止采样取平均
  * @note  同时用峰峰值判断板子是否真的静止，静止是标定的前提
  * @note  温度与陀螺使用同一批样本取算术平均，均温作为温漂补偿基准。
  *        首尾温度仅保留用于观察，不参与补偿计算。
  */
static ret_code_t imu_calibrate_gyro(void)
{
    bsp_imu_data_t data;
    double       sum[3]  = { 0.0, 0.0, 0.0 };
    double       vmin[3] = { 0.0, 0.0, 0.0 };
    double       vmax[3] = { 0.0, 0.0, 0.0 };
    double       temp_sum = 0.0;
    double       temp_start_c = 0.0;
    double       temp_end_c = 0.0;
    uint32_t    i;
    uint32_t    k;
    ret_code_t  ret;

    for (i = 0U; i < APP_IMU_CALI_SAMPLES; i++) {
        ret = bsp_imu_read(&data);
        if (ret != RET_OK) {
            return ret;
        }

        if (i == 0U) {
            temp_start_c = data.temp_c;
        }
        temp_end_c = data.temp_c;

        for (k = 0U; k < 3U; k++) {
            double v = data.gyro_radps[k];

            if (i == 0U) {
                vmin[k] = v;
                vmax[k] = v;
            } else {
                if (v < vmin[k]) { vmin[k] = v; }
                if (v > vmax[k]) { vmax[k] = v; }
            }
            sum[k] += v;
        }
        temp_sum += data.temp_c;

        HAL_Delay(APP_IMU_CALI_INTERVAL_MS);
    }

    for (k = 0U; k < 3U; k++) {
        if ((vmax[k] - vmin[k]) > APP_IMU_CALI_MAX_SPREAD) {
            return RET_ERROR;   /* 标定期间板子在动，这一组数据不能用 */
        }
    }
    /* 全部轴通过后才提交温度和零偏，失败时不保留部分标定结果。 */
    s_gyro_cali_start_temp_c = temp_start_c;
    s_gyro_cali_end_temp_c   = temp_end_c;
    s_gyro_cali_mean_temp_c  = temp_sum / (double)APP_IMU_CALI_SAMPLES;
    s_gyro_ref_temp_c       = s_gyro_cali_mean_temp_c;

    /* 平均零偏与平均温度直接配对；运行时 gyro = raw - bias_mean - K * (T - T_mean)。 */
    for (k = 0U; k < 3U; k++) {
        s_gyro_bias[k] = sum[k] / (double)APP_IMU_CALI_SAMPLES;
    }

    return RET_OK;
}

int app_imu_init(SPI_HandleTypeDef *hspi)
{
    ret_code_t ret;
    app_imu_snapshot_t initial = { .quat = { 1.0, 0.0, 0.0, 0.0 } };

    s_ready          = false;
    s_calibrated     = false;
    s_gyro_ref_temp_c = 0.0;
    s_gyro_cali_start_temp_c = 0.0;
    s_gyro_cali_end_temp_c   = 0.0;
    s_gyro_cali_mean_temp_c  = 0.0;
    s_err_count      = 0U;
    s_yaw_cont       = 0.0;
    s_yaw_prev       = 0.0;
    s_yaw_cont_ready = false;

    /* 快照清零：还没解算出姿态之前，读取侧拿到的是全 0 + valid = false */
    for (uint32_t k = 0U; k < 3U; k++) {
        s_gyro_bias[k] = 0.0;
    }
    imu_publish_snapshot(&initial);

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

    /* 4) 标定陀螺零偏并记录基准温度（失败不致命，bias 保持 0，禁用温漂补偿）*/
    s_calibrated = (imu_calibrate_gyro() == RET_OK);

    s_last_cycle = bsp_dwt_get_cycles();
    s_ready      = true;

    return RET_OK;
}

/**
  * @brief 把角度归一化到 (-π, π]
  * @note  只用于处理"两拍之间的 yaw 增量"：输入必定落在 (-2π, 2π) 内，
  *        所以一次加减就够，不需要反复缠绕
  */
static double wrap_pi(double a)
{
    if (a > IMU_PI) {
        a -= IMU_TWO_PI;
    }
    if (a <= -IMU_PI) {
        a += IMU_TWO_PI;
    }
    return a;
}

int app_imu_poll(void)
{
    bsp_imu_data_t data;
    app_imu_snapshot_t next = { 0 };
    double          gyro[3];
    double          euler[3];
    uint32_t       now_cycle;
    double          dt_s;
    double          temp_delta_c;
    uint32_t       k;
    ret_code_t     ret;

    if (!s_ready) {
        return RET_NOTREADY;
    }

    /* ---- 1) 用 DWT 实测 dt（分辨率 15.6ns，远优于 HAL_GetTick 的 1ms）----
     *        无符号减法能自然处理 CYCCNT 回绕 */
    now_cycle = bsp_dwt_get_cycles();
    dt_s      = bsp_dwt_cycles_to_s(now_cycle - s_last_cycle);

    /* 调得太快：本轮直接跳过。既不算错误，也不会丢数据（传感器本来就没新样本） */
    if (dt_s < APP_IMU_MIN_DT_S) {
        return RET_IDLE;
    }
    s_last_cycle = now_cycle;

    /* 调得太慢：dt 就是实测值，照常积分即可。只在间隔离谱（断点恢复、长时间
     * 阻塞）时才按上限夹住，避免单拍积分把姿态带飞 —— 而不是不更新姿态 */
    dt_s = CLAMP(dt_s, APP_IMU_MIN_DT_S, APP_IMU_MAX_DT_S);

    /* ---- 2) 读传感器（data.temp_c 为本次读取的当前温度）---- */
    ret = bsp_imu_read(&data);
    if (ret != RET_OK) {
        s_err_count++;
        return ret;     /* 读失败就保持上一次的姿态，绝不把垃圾喂进解算器 */
    }

    /* ---- 3) 减零偏及温漂；温差保留符号，升温/降温都能补偿 ---- */
    temp_delta_c = s_calibrated ? (data.temp_c - s_gyro_ref_temp_c) : 0.0;
    for (k = 0U; k < 3U; k++) {
        gyro[k] = data.gyro_radps[k] - s_gyro_bias[k]
                - s_gyro_temp_coeff[k] * temp_delta_c;
    }

    /* ---- 4) 姿态解算：dt 用实测值，所以调用节奏不齐也不影响积分正确性 ---- */
    alg_ahrs_update(&s_ahrs, gyro, data.accel_mps2, dt_s);

    /* ---- 5) 发布：先把 yaw 连续化，再直接写快照；读取侧下一瞬间就能取到最新值 ----
     *      连续化必须在写入侧做（唯一写者、约 1kHz）：1ms 内最多转约 0.5°，
     *      所以两拍增量必定落在 (-π, π] 内，方向不会判错。
     *      先完成局部双精度计算，最后在短临界区内统一发布 */
    alg_ahrs_get_euler(&s_ahrs, euler);

    if (!s_yaw_cont_ready) {
        s_yaw_cont       = euler[AHRS_YAW];     /* 第一帧：以此刻朝向为基准 */
        s_yaw_prev       = euler[AHRS_YAW];
        s_yaw_cont_ready = true;
    } else {
        s_yaw_cont += wrap_pi(euler[AHRS_YAW] - s_yaw_prev);
        s_yaw_prev  = euler[AHRS_YAW];
    }
    euler[AHRS_YAW] = s_yaw_cont;               /* 快照里存的是连续角 */

    for (k = 0U; k < 3U; k++) {
        next.euler[k] = euler[k];
        next.gyro[k]  = gyro[k];
        next.accel[k] = data.accel_mps2[k];
    }

    alg_ahrs_get_quat(&s_ahrs, next.quat);

    next.temp_c  = data.temp_c;
    next.rate_hz = 1.0 / dt_s;
    next.valid   = true;
    imu_publish_snapshot(&next);

    return RET_OK;
}

bool app_imu_get_euler(double euler_rad[3])
{
    return imu_read_vector(s_snap.euler, euler_rad, 3U);
}

bool app_imu_get_quat(double quat[4])
{
    return imu_read_vector(s_snap.quat, quat, 4U);
}

bool app_imu_get_gyro(double gyro_radps[3])
{
    return imu_read_vector(s_snap.gyro, gyro_radps, 3U);
}

bool app_imu_get_accel(double accel_mps2[3])
{
    return imu_read_vector(s_snap.accel, accel_mps2, 3U);
}

double app_imu_get_temp(void)
{
    return imu_read_scalar(&s_snap.temp_c);
}

double app_imu_get_rate_hz(void)
{
    return imu_read_scalar(&s_snap.rate_hz);
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

void app_imu_get_init_fail(uint8_t *dev, uint8_t *reg, uint8_t *expect, uint8_t *actual)
{
    bsp_imu_get_init_fail(dev, reg, expect, actual);
}
