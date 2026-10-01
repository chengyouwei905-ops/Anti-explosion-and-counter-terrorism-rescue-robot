/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   IMU 应用层实现
  *
  * 三块职责，互不耦合：
  *   1. 节拍：app_imu_poll() 用 DWT 实测 dt 自限速，**调用频率不敏感**
  *   2. 解算：bsp_imu_read() -> 减零偏 -> alg_ahrs_update()
  *   3. 发布：每拍算好一条快照，用 seqlock 发布；读取侧 app_imu_get_xxx()
  *            无锁、无线序要求，随便哪个上下文调都行
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

/** @brief 开机零偏标定的采样次数（配合下面的 1ms 间隔，约 1 秒） */
#define APP_IMU_CALI_SAMPLES      1000U
/** @brief 标定采样间隔（ms），BMI088 陀螺 ODR 是 1kHz，刚好每次拿到新数据 */
#define APP_IMU_CALI_INTERVAL_MS  1U
/** @brief 标定期间允许的零偏抖动峰峰值（rad/s）。超过就说明板子在被晃，标定作废 */
#define APP_IMU_CALI_MAX_SPREAD   0.05f

/**
  * @brief 两次采样之间的最小间隔（秒）—— 限速用，比它更密地读只会拿到重复样本
  * @note  陀螺 ODR 是 1000Hz。想把解算频率提到 1kHz 以上：把这里调小，**同时**
  *        改 bsp_imu.c 里陀螺的 ODR（GYRO_BANDWIDTH 那一条）
  */
#define APP_IMU_MIN_DT_S          0.001f
/**
  * @brief 单拍 dt 上限（秒）—— 夹取用
  * @note  实测 dt 超过它就按它算。目的是兜住断点恢复 / 长时间阻塞那种异常间隔，
  *        而不是"因为调用得慢就不更新姿态"：1s 以内都按真实 dt 积分，不会冻结。
  */
#define APP_IMU_MAX_DT_S          1.0f

/** @brief 编译器屏障，防止快照的写入顺序被优化打乱 */
#define IMU_BARRIER()             __asm volatile ("" ::: "memory")

/**
  * @brief 发布给读取侧的姿态快照（整条一起更新，保证读到的是同一拍的数据）
  * @note  写入只发生在 app_imu_poll()；读取侧只读这里，不碰 s_ahrs
  */
typedef struct
{
    float euler[3];    /**< [roll, pitch, yaw]，弧度 */
    float quat[4];     /**< [w, x, y, z] */
    float gyro[3];     /**< 已减零偏的角速度，rad/s */
    float accel[3];    /**< 加速度，m/s^2 */
    float temp_c;      /**< 芯片温度，℃ */
    float rate_hz;     /**< 最近一拍的等效采样率，Hz */
    bool  valid;       /**< false = 还没有有效姿态 */
} app_imu_snapshot_t;

static bool          s_ready;
static bool          s_calibrated;
static Ahrs_State_t  s_ahrs;
static float         s_gyro_bias[3];
static uint32_t      s_last_cycle;   /**< 上次采样的 DWT 周期计数 */
static uint32_t      s_err_count;

static app_imu_snapshot_t s_snap;
static volatile uint32_t  s_snap_seq;   /**< seqlock 序号：奇数 = 正在写 */

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
    s_err_count  = 0U;
    s_snap_seq   = 0U;

    /* 快照清零：还没解算出姿态之前，读取侧拿到的是全 0 + valid = false */
    for (uint32_t k = 0U; k < 3U; k++) {
        s_gyro_bias[k]  = 0.0f;
        s_snap.euler[k] = 0.0f;
        s_snap.gyro[k]  = 0.0f;
        s_snap.accel[k] = 0.0f;
    }
    for (uint32_t k = 0U; k < 4U; k++) {
        s_snap.quat[k] = 0.0f;
    }
    s_snap.quat[0] = 1.0f;
    s_snap.temp_c  = 0.0f;
    s_snap.rate_hz = 0.0f;
    s_snap.valid   = false;

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
    float          gyro[3];
    float          euler[3];
    uint32_t       now_cycle;
    float          dt_s;
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

    /* ---- 2) 读传感器 ---- */
    ret = bsp_imu_read(&data);
    if (ret != RET_OK) {
        s_err_count++;
        return ret;     /* 读失败就保持上一次的姿态，绝不把垃圾喂进解算器 */
    }

    /* ---- 3) 减零偏 ---- */
    for (k = 0U; k < 3U; k++) {
        gyro[k] = data.gyro_radps[k] - s_gyro_bias[k];
    }

    /* ---- 4) 姿态解算：dt 用实测值，所以调用节奏不齐也不影响积分正确性 ---- */
    alg_ahrs_update(&s_ahrs, gyro, data.accel_mps2, dt_s);

    /* ---- 5) 发布快照：整条一起写，读取侧绝不会读到"半新半旧"的姿态 ---- */
    alg_ahrs_get_euler(&s_ahrs, euler);

    s_snap_seq++;                       /* 奇数：写入中 */
    IMU_BARRIER();
    for (k = 0U; k < 3U; k++) {
        s_snap.euler[k] = euler[k];
        s_snap.gyro[k]  = gyro[k];
        s_snap.accel[k] = data.accel_mps2[k];
    }
    alg_ahrs_get_quat(&s_ahrs, s_snap.quat);
    s_snap.temp_c  = data.temp_c;
    s_snap.rate_hz = 1.0f / dt_s;
    s_snap.valid   = true;
    IMU_BARRIER();
    s_snap_seq++;                       /* 偶数：写完 */

    return RET_OK;
}

/**
  * @brief 读一份一致的快照（seqlock，读取侧无锁）
  * @note  写入方只有 app_imu_poll()。若拷贝期间被打断（序号变了）就重来一次，
  *        代价只是多拷 60 多个字节，比关中断更划算
  * @retval true 数据有效
  */
static bool imu_snapshot_read(app_imu_snapshot_t *out)
{
    const volatile uint8_t *src = (const volatile uint8_t *)&s_snap;
    uint8_t                *dst = (uint8_t *)out;
    uint32_t                s0;
    uint32_t                s1;
    uint32_t                i;

    for (;;) {
        s0 = s_snap_seq;
        if ((s0 & 1U) != 0U) {
            continue;               /* 写者正在写，等它写完 */
        }
        IMU_BARRIER();
        for (i = 0U; i < sizeof(s_snap); i++) {
            dst[i] = src[i];
        }
        IMU_BARRIER();
        s1 = s_snap_seq;
        if (s0 == s1) {
            break;
        }
    }

    return out->valid;
}

bool app_imu_get_euler(float euler_rad[3])
{
    app_imu_snapshot_t snap;

    if (euler_rad == NULL) {
        return false;
    }
    if (!imu_snapshot_read(&snap)) {
        euler_rad[AHRS_ROLL]  = 0.0f;
        euler_rad[AHRS_PITCH] = 0.0f;
        euler_rad[AHRS_YAW]   = 0.0f;
        return false;
    }

    euler_rad[AHRS_ROLL]  = snap.euler[AHRS_ROLL];
    euler_rad[AHRS_PITCH] = snap.euler[AHRS_PITCH];
    euler_rad[AHRS_YAW]   = snap.euler[AHRS_YAW];
    return true;
}

bool app_imu_get_quat(float quat[4])
{
    app_imu_snapshot_t snap;

    if (quat == NULL) {
        return false;
    }
    if (!imu_snapshot_read(&snap)) {
        for (uint32_t k = 0U; k < 4U; k++) {
            quat[k] = 0.0f;
        }
        return false;
    }

    for (uint32_t k = 0U; k < 4U; k++) {
        quat[k] = snap.quat[k];
    }
    return true;
}

bool app_imu_get_gyro(float gyro_radps[3])
{
    app_imu_snapshot_t snap;

    if (gyro_radps == NULL) {
        return false;
    }
    if (!imu_snapshot_read(&snap)) {
        for (uint32_t k = 0U; k < 3U; k++) {
            gyro_radps[k] = 0.0f;
        }
        return false;
    }

    for (uint32_t k = 0U; k < 3U; k++) {
        gyro_radps[k] = snap.gyro[k];
    }
    return true;
}

bool app_imu_get_accel(float accel_mps2[3])
{
    app_imu_snapshot_t snap;

    if (accel_mps2 == NULL) {
        return false;
    }
    if (!imu_snapshot_read(&snap)) {
        for (uint32_t k = 0U; k < 3U; k++) {
            accel_mps2[k] = 0.0f;
        }
        return false;
    }

    for (uint32_t k = 0U; k < 3U; k++) {
        accel_mps2[k] = snap.accel[k];
    }
    return true;
}

float app_imu_get_temp(void)
{
    app_imu_snapshot_t snap;

    return imu_snapshot_read(&snap) ? snap.temp_c : 0.0f;
}

float app_imu_get_rate_hz(void)
{
    app_imu_snapshot_t snap;

    return imu_snapshot_read(&snap) ? snap.rate_hz : 0.0f;
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
