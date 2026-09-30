/**
  ******************************************************************************
  * @file    bsp_imu.c
  * @brief   BMI088 六轴 IMU 驱动：SPI2 + DMA 全双工收发 + 寄存器读写
  *
  * 数据流：
  *   bsp_imu_read()
  *     -> 加速度 0x12 起 6 字节 / 陀螺仪 0x00 起 8 字节 / 温度 0x22 起 2 字节
  *     -> 换算成 m/s^2、rad/s、℃
  *
  * BMI088 的 SPI 读时序有两个坑（本文件已处理）：
  *   1. 加速度计：地址字节后必须再跟 1 个 dummy 字节，数据从第 3 个字节开始；
  *      陀螺仪：地址字节后直接就是数据。
  *   2. 读地址需要在寄存器地址上加 0x80。
  ******************************************************************************
  */

#include "bsp_imu.h"
#include "main.h"   /* CubeMX 生成的 SPI2_CS0_Pin / SPI2_CS1_Pin 等宏 */

/* ========================= BMI088 寄存器地址 ========================= */

/* --- 加速度计 --- */
#define IMU_ACC_CHIP_ID            0x00U    /**< WHO_AM_I */
#define IMU_ACC_DATA               0x12U    /**< ACCEL_XOUT_L，连续 6 字节 */
#define IMU_ACC_TEMP_M             0x22U    /**< 温度高字节，连续 2 字节 */
#define IMU_ACC_CONF               0x40U
#define IMU_ACC_RANGE              0x41U
#define IMU_ACC_INT1_IO_CTRL       0x53U
#define IMU_ACC_INT_MAP_DATA       0x58U
#define IMU_ACC_PWR_CONF           0x7CU
#define IMU_ACC_PWR_CTRL           0x7DU
#define IMU_ACC_SOFTRESET          0x7EU

/* --- 陀螺仪 --- */
#define IMU_GYRO_CHIP_ID           0x00U    /**< WHO_AM_I */
#define IMU_GYRO_BASE              0x00U    /**< 从 WHO_AM_I 起连续 8 字节 = ID + 状态 + 6 字节角速度 */
#define IMU_GYRO_RANGE             0x0FU
#define IMU_GYRO_BANDWIDTH         0x10U
#define IMU_GYRO_LPM1              0x11U
#define IMU_GYRO_SOFTRESET         0x14U
#define IMU_GYRO_CTRL              0x15U
#define IMU_GYRO_INT3_INT4_IO_CONF 0x16U
#define IMU_GYRO_INT3_INT4_IO_MAP  0x18U

/* --- 期望的 WHO_AM_I 值 --- */
#define IMU_ACC_CHIP_ID_VALUE      0x1EU
#define IMU_GYRO_CHIP_ID_VALUE     0x0FU

#define IMU_SOFTRESET_VALUE        0xB6U

/* ========================= 灵敏度换算系数 ========================= */
/* 加速度 ±3g：原始值 -> m/s^2（沿用参考例程的官方系数） */
#define IMU_ACCEL_3G_SEN           0.0008974358974f
/* 角速度 ±2000dps：原始值 -> rad/s */
#define IMU_GYRO_2000_SEN          0.00106526443603169529841533860381f
/* 温度：原始值 * 0.125 + 23.0 */
#define IMU_TEMP_FACTOR            0.125f
#define IMU_TEMP_OFFSET            23.0f

/* ========================= 时序参数 ========================= */
#define IMU_LONG_DELAY_TIME        80U     /**< 软复位后等待，单位 ms */
#define IMU_COM_WAIT_SENSOR_TIME   150U    /**< 每次寄存器操作后的等待，单位 us */
#define IMU_SPI_TIMEOUT_MS         10U     /**< 单次 SPI 传输超时，单位 ms */

/* ========================= 片选映射 ========================= */
/* ⚠️ 若实测发现加速度计/陀螺仪片选接反了，只交换这两组宏即可 */
#define IMU_ACCEL_CS_PORT          SPI2_CS0_GPIO_Port
#define IMU_ACCEL_CS_PIN           SPI2_CS0_Pin
#define IMU_GYRO_CS_PORT           SPI2_CS1_GPIO_Port
#define IMU_GYRO_CS_PIN            SPI2_CS1_Pin

/** @brief 单次 SPI 传输缓冲上限（最长 8 字节数据 + 2 字节头部） */
#define IMU_XFER_BUF_SIZE          16U

/* ========================= 内部类型 ========================= */

typedef enum
{
    IMU_DEV_ACCEL = 0,  /**< 加速度计 */
    IMU_DEV_GYRO  = 1,  /**< 陀螺仪   */
} imu_dev_t;

/** @brief 配置寄存器 地址-值 对，用于初始化表 */
typedef struct
{
    uint8_t reg;
    uint8_t val;
} imu_reg_val_t;

/* ========================= 加速度计初始化表 ========================= */
/*
 * ACC_CONF (0x40) = 0x80                     bit7 = 必须置位的标志位
 *                 | (acc_bwp << 4)           bits[6:4] 过采样/带宽档位
 *                 |  acc_odr                 bits[3:0] 输出数据率(刷新率)
 *
 *   acc_odr : 0x05 = 12.5Hz   0x06 = 25Hz    0x07 = 50Hz    0x08 = 100Hz
 *             0x09 = 200Hz    0x0A = 400Hz   0x0B = 800Hz   0x0C = 1600Hz
 *             ⚠️ 选 1600Hz 时 acc_bwp 只能是 OSR4(0x00)
 *   acc_bwp : 0x00 = OSR4     0x10 = OSR2     0x20 = Normal(归一化)
 *
 * 当前：0x80 | 0x20(Normal) | 0x0B(800Hz) = 0xAB
 */
static const imu_reg_val_t s_accel_init_seq[] =
{
    { IMU_ACC_PWR_CTRL,     0x04U },   /* 加速度计上电 */
    { IMU_ACC_PWR_CONF,     0x00U },   /* 退出挂起，进入正常模式 */
    { IMU_ACC_CONF,         0xABU },   /* 800Hz ODR  ← 想改加速度刷新率就改这里 */
    { IMU_ACC_RANGE,        0x00U },   /* ±3g */
    { IMU_ACC_INT1_IO_CTRL, 0x08U },   /* INT1 使能、推挽、低有效 */
    { IMU_ACC_INT_MAP_DATA, 0x04U },   /* 数据就绪映射到 INT1 */
};

/* ========================= 陀螺仪初始化表 ========================= */
/*
 * GYRO_BANDWIDTH (0x10) = 0x80(必须置位的标志位) | 档位
 *
 *   档位（输出数据率_滤波带宽）：
 *     0x00 = 2000Hz_532Hz   0x01 = 2000Hz_230Hz
 *     0x02 = 1000Hz_116Hz   0x03 =  400Hz_47Hz
 *     0x04 =  200Hz_23Hz    0x05 =  100Hz_12Hz
 *     0x06 =  200Hz_64Hz    0x07 =  100Hz_32Hz
 *
 * 当前：0x80 | 0x02 = 0x82，即 ODR 1000Hz、带宽 116Hz
 *       —— 和 app_imu_poll() 约 1kHz 的调用率对齐，每次都能拿到一帧新数据
 */
static const imu_reg_val_t s_gyro_init_seq[] =
{
    { IMU_GYRO_RANGE,             0x00U },  /* ±2000 dps */
    { IMU_GYRO_BANDWIDTH,         0x82U },  /* 0x80 | 1000Hz ODR / 116Hz */
    { IMU_GYRO_LPM1,              0x00U },  /* 正常模式 */
    { IMU_GYRO_CTRL,              0x80U },  /* 打开数据就绪 */
    { IMU_GYRO_INT3_INT4_IO_CONF, 0x00U },  /* INT3 推挽、低有效 */
    { IMU_GYRO_INT3_INT4_IO_MAP,  0x01U },  /* 数据就绪映射到 INT3 */
};

/* ========================= 内部状态 ========================= */

static SPI_HandleTypeDef *s_hspi = NULL;

static uint8_t s_tx[IMU_XFER_BUF_SIZE];
static uint8_t s_rx[IMU_XFER_BUF_SIZE];

static volatile bool s_xfer_done  = false;
static volatile bool s_xfer_error = false;
static bool          s_busy       = false;

static bool    s_ready        = false;
static uint8_t s_acc_chip_id  = 0U;
static uint8_t s_gyro_chip_id = 0U;

/* 初始化失败现场，方便在调试器里直接看 */
static uint8_t s_fail_dev    = 0xFFU;
static uint8_t s_fail_reg    = 0xFFU;
static uint8_t s_fail_expect = 0U;
static uint8_t s_fail_actual = 0U;

/* ========================= 延时（不占用定时器，直接读 SysTick） ========================= */

static void imu_delay_us(uint32_t us)
{
    uint32_t reload  = SysTick->LOAD + 1U;
    uint32_t cycles  = us * (SystemCoreClock / 1000000U);
    uint32_t prev    = SysTick->VAL;
    uint32_t elapsed = 0U;

    while (elapsed < cycles)
    {
        uint32_t now = SysTick->VAL;
        elapsed += (prev >= now) ? (prev - now) : ((prev + reload) - now);
        prev = now;
    }
}

static void imu_delay_ms(uint32_t ms)
{
    while (ms-- > 0U)
    {
        imu_delay_us(1000U);
    }
}

/* ========================= 片选控制 ========================= */

static void imu_cs_write(imu_dev_t dev, GPIO_PinState state)
{
    if (dev == IMU_DEV_ACCEL)
    {
        HAL_GPIO_WritePin(IMU_ACCEL_CS_PORT, IMU_ACCEL_CS_PIN, state);
    }
    else
    {
        HAL_GPIO_WritePin(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN, state);
    }
}

/* ========================= SPI2 + DMA 收发 ========================= */

/**
  * @brief SPI2 全双工 DMA 收发（同步等待完成）
  * @note  片选由调用方在传输前后自己拉低/拉高，这里只管数据
  */
static ret_code_t imu_spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout_ms)
{
    uint32_t start;

    if ((s_hspi == NULL) || (len == 0U) || (len > IMU_XFER_BUF_SIZE))
    {
        return RET_PARAM;
    }
    if (s_busy)
    {
        return RET_BUSY;
    }

    s_busy       = true;
    s_xfer_done  = false;
    s_xfer_error = false;

    if (HAL_SPI_TransmitReceive_DMA(s_hspi, tx, rx, len) != HAL_OK)
    {
        s_busy = false;
        return RET_ERROR;
    }

    start = HAL_GetTick();
    while ((!s_xfer_done) && (!s_xfer_error))
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            (void)HAL_SPI_Abort(s_hspi);
            s_busy = false;
            return RET_TIMEOUT;
        }
    }

    s_busy = false;
    return s_xfer_error ? RET_ERROR : RET_OK;
}

/**
  * @brief SPI 传输完成回调（独占实现，其它文件不要再写）
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        s_xfer_done = true;
    }
}

/**
  * @brief SPI 错误回调（独占实现，其它文件不要再写）
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        s_xfer_error = true;
    }
}

/* ========================= 寄存器读写 ========================= */

/** @brief 读操作前面要发的头部字节数：加速度计多 1 个 dummy 字节 */
static uint16_t imu_header_len(imu_dev_t dev)
{
    return (dev == IMU_DEV_ACCEL) ? 2U : 1U;
}

/**
  * @brief 连续读寄存器
  * @param  dev 器件
  * @param  reg 起始寄存器地址（内部会自动加 0x80 读标志）
  * @param  out 数据输出缓冲
  * @param  len 读取字节数
  */
static ret_code_t imu_read_regs(imu_dev_t dev, uint8_t reg, uint8_t *out, uint16_t len)
{
    uint16_t hdr   = imu_header_len(dev);
    uint16_t total = (uint16_t)(hdr + len);
    uint16_t i;
    ret_code_t ret;

    if ((out == NULL) || (total > IMU_XFER_BUF_SIZE))
    {
        return RET_PARAM;
    }

    for (i = 0U; i < hdr; i++)
    {
        s_tx[i] = (uint8_t)(reg | 0x80U);   /* 读标志 */
    }
    for (i = hdr; i < total; i++)
    {
        s_tx[i] = 0x55U;                    /* 读阶段的填充字节，内容无所谓 */
    }

    imu_cs_write(dev, GPIO_PIN_RESET);
    ret = imu_spi_transfer(s_tx, s_rx, total, IMU_SPI_TIMEOUT_MS);
    imu_cs_write(dev, GPIO_PIN_SET);

    if (ret != RET_OK)
    {
        return ret;
    }

    for (i = 0U; i < len; i++)
    {
        out[i] = s_rx[hdr + i];
    }

    return RET_OK;
}

/**
  * @brief 写单个寄存器
  */
static ret_code_t imu_write_reg(imu_dev_t dev, uint8_t reg, uint8_t val)
{
    ret_code_t ret;

    s_tx[0] = reg;      /* 写操作地址不加 0x80 */
    s_tx[1] = val;

    imu_cs_write(dev, GPIO_PIN_RESET);
    ret = imu_spi_transfer(s_tx, s_rx, 2U, IMU_SPI_TIMEOUT_MS);
    imu_cs_write(dev, GPIO_PIN_SET);

    return ret;
}

/* ========================= 初始化 ========================= */

static void imu_record_fail(imu_dev_t dev, uint8_t reg, uint8_t expect, uint8_t actual)
{
    s_fail_dev    = (uint8_t)dev;
    s_fail_reg    = reg;
    s_fail_expect = expect;
    s_fail_actual = actual;
}

/**
  * @brief 读 WHO_AM_I（连读两次，第一次丢弃 —— 上电后首字节经常是脏的）
  */
static ret_code_t imu_read_chip_id(imu_dev_t dev, uint8_t *id)
{
    ret_code_t ret;

    ret = imu_read_regs(dev, 0x00U, id, 1U);
    if (ret != RET_OK)
    {
        return ret;
    }
    imu_delay_us(IMU_COM_WAIT_SENSOR_TIME);

    ret = imu_read_regs(dev, 0x00U, id, 1U);
    if (ret != RET_OK)
    {
        return ret;
    }
    imu_delay_us(IMU_COM_WAIT_SENSOR_TIME);

    return RET_OK;
}

/**
  * @brief 写一个配置寄存器并回读校验
  */
static ret_code_t imu_write_reg_checked(imu_dev_t dev, uint8_t reg, uint8_t val)
{
    uint8_t    rb = 0U;
    ret_code_t ret;

    ret = imu_write_reg(dev, reg, val);
    if (ret != RET_OK)
    {
        imu_record_fail(dev, reg, val, 0U);
        return ret;
    }
    imu_delay_us(IMU_COM_WAIT_SENSOR_TIME);

    ret = imu_read_regs(dev, reg, &rb, 1U);
    if (ret != RET_OK)
    {
        imu_record_fail(dev, reg, val, 0U);
        return ret;
    }

    if (rb != val)
    {
        imu_record_fail(dev, reg, val, rb);
        return RET_ERROR;
    }

    return RET_OK;
}

/**
  * @brief 单颗器件（加速度计或陀螺仪）的上电初始化
  */
static ret_code_t imu_init_device(imu_dev_t dev,
                                  uint8_t expect_id, uint8_t *id_out,
                                  const imu_reg_val_t *seq, uint32_t seq_len)
{
    const uint8_t softreset_reg = (dev == IMU_DEV_ACCEL) ? IMU_ACC_SOFTRESET : IMU_GYRO_SOFTRESET;
    uint8_t    id = 0U;
    uint32_t   i;
    ret_code_t ret;

    /* 1) 复位前先确认 SPI 通（值不校验，只要求能收发） */
    ret = imu_read_chip_id(dev, &id);
    if (ret != RET_OK)
    {
        imu_record_fail(dev, 0x00U, expect_id, 0U);
        return ret;
    }

    /* 2) 软复位 */
    ret = imu_write_reg(dev, softreset_reg, IMU_SOFTRESET_VALUE);
    if (ret != RET_OK)
    {
        imu_record_fail(dev, softreset_reg, IMU_SOFTRESET_VALUE, 0U);
        return ret;
    }
    imu_delay_ms(IMU_LONG_DELAY_TIME);

    /* 3) 复位后校验 WHO_AM_I */
    ret = imu_read_chip_id(dev, &id);
    if (ret != RET_OK)
    {
        imu_record_fail(dev, 0x00U, expect_id, 0U);
        return ret;
    }
    if (id != expect_id)
    {
        imu_record_fail(dev, 0x00U, expect_id, id);
        return RET_ERROR;
    }

    /* 4) 逐条写配置寄存器并回读校验 */
    for (i = 0U; i < seq_len; i++)
    {
        ret = imu_write_reg_checked(dev, seq[i].reg, seq[i].val);
        if (ret != RET_OK)
        {
            return ret;
        }
    }

    *id_out = id;
    return RET_OK;
}

ret_code_t bsp_imu_init(SPI_HandleTypeDef *hspi)
{
    ret_code_t ret;

    if (hspi == NULL)
    {
        return RET_PARAM;
    }

    s_hspi        = hspi;
    s_ready       = false;
    s_acc_chip_id = 0U;
    s_gyro_chip_id = 0U;
    s_fail_dev    = 0xFFU;
    s_fail_reg    = 0xFFU;
    s_fail_expect = 0U;
    s_fail_actual = 0U;

    /* 两路片选先都拉高（不选中） */
    imu_cs_write(IMU_DEV_ACCEL, GPIO_PIN_SET);
    imu_cs_write(IMU_DEV_GYRO, GPIO_PIN_SET);

    ret = imu_init_device(IMU_DEV_ACCEL, IMU_ACC_CHIP_ID_VALUE, &s_acc_chip_id,
                          s_accel_init_seq, (uint32_t)ARRAY_SIZE(s_accel_init_seq));
    if (ret != RET_OK)
    {
        return ret;
    }

    ret = imu_init_device(IMU_DEV_GYRO, IMU_GYRO_CHIP_ID_VALUE, &s_gyro_chip_id,
                          s_gyro_init_seq, (uint32_t)ARRAY_SIZE(s_gyro_init_seq));
    if (ret != RET_OK)
    {
        return ret;
    }

    s_ready = true;
    return RET_OK;
}

bool bsp_imu_is_ready(void)
{
    return s_ready;
}

/* ========================= 读数据 ========================= */

ret_code_t bsp_imu_read(bsp_imu_data_t *out)
{
    uint8_t    buf[8];
    int16_t    raw;
    uint32_t   i;
    ret_code_t ret;

    if (out == NULL)
    {
        return RET_PARAM;
    }
    if (!s_ready)
    {
        return RET_NOTREADY;
    }

    /* --- 加速度：0x12 起连续 6 字节，小端，先低后高 --- */
    ret = imu_read_regs(IMU_DEV_ACCEL, IMU_ACC_DATA, buf, 6U);
    if (ret != RET_OK)
    {
        return ret;
    }
    for (i = 0U; i < 3U; i++)
    {
        raw = (int16_t)(((uint16_t)buf[2U * i + 1U] << 8) | (uint16_t)buf[2U * i]);
        out->accel_mps2[i] = (float)raw * IMU_ACCEL_3G_SEN;
    }

    /* --- 陀螺仪：0x00 起连续 8 字节 = WHO_AM_I + 状态 + 6 字节角速度 --- */
    ret = imu_read_regs(IMU_DEV_GYRO, IMU_GYRO_BASE, buf, 8U);
    if (ret != RET_OK)
    {
        return ret;
    }
    if (buf[0] != IMU_GYRO_CHIP_ID_VALUE)
    {
        return RET_ERROR;
    }
    for (i = 0U; i < 3U; i++)
    {
        raw = (int16_t)(((uint16_t)buf[2U * i + 3U] << 8) | (uint16_t)buf[2U * i + 2U]);
        out->gyro_radps[i] = (float)raw * IMU_GYRO_2000_SEN;
    }

    /* --- 温度：0x22/0x23，11 位有符号数 --- */
    ret = imu_read_regs(IMU_DEV_ACCEL, IMU_ACC_TEMP_M, buf, 2U);
    if (ret != RET_OK)
    {
        return ret;
    }
    raw = (int16_t)(((uint16_t)buf[0] << 3) | (uint16_t)(buf[1] >> 5));
    if (raw > 1023)
    {
        raw = (int16_t)(raw - 2048);
    }
    out->temp_c = (float)raw * IMU_TEMP_FACTOR + IMU_TEMP_OFFSET;

    return RET_OK;
}

/* ========================= 调试接口 ========================= */

uint8_t bsp_imu_get_accel_chip_id(void)
{
    return s_acc_chip_id;
}

uint8_t bsp_imu_get_gyro_chip_id(void)
{
    return s_gyro_chip_id;
}

void bsp_imu_get_init_fail(uint8_t *dev, uint8_t *reg, uint8_t *expect, uint8_t *actual)
{
    if (dev != NULL)    { *dev = s_fail_dev; }
    if (reg != NULL)    { *reg = s_fail_reg; }
    if (expect != NULL) { *expect = s_fail_expect; }
    if (actual != NULL) { *actual = s_fail_actual; }
}
