#include "app_imu_hwt101.h"
#include "proto_hwt101.h"

#define HWT101_PI 3.14159265358979323846
#define HWT101_CHUNK_SIZE 64U

typedef struct {
    double yaw;
    uint32_t timestamp_ms;
    bool valid;
} hwt101_snapshot_t;

static volatile hwt101_snapshot_t s_snap;
static bool s_inited, s_yaw_started;
static int16_t s_previous_raw;
static int64_t s_continuous_raw;
static uint32_t s_last_poll_ms, s_rx_overflow;

/* 仅供调试器观察，不增加驱动接口。运行后暂停，区分未收到字节和未解出角度。 */
volatile uint32_t hwt101_rx_bytes;
volatile uint32_t hwt101_angle_frames;

static void hwt101_publish(bool valid, uint32_t now)
{
    hwt101_snapshot_t next = {
        APP_IMU_HWT101_YAW_SIGN * (double)s_continuous_raw * (HWT101_PI / 32768.0),
        now, valid
    };
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_snap = next;
    __set_PRIMASK(primask);
}

static void hwt101_on_yaw(int16_t raw)
{
    hwt101_angle_frames++;
    if (!s_yaw_started) {
        s_yaw_started = true;
        s_continuous_raw = 0;
    } else {
        /* 原始32768计数=180度，65536计数=360度。
         * +179 -> -179的差为-358度，补360后只累加+2度；反向同理。
         * 边界两侧反复跳变时增减相抵，不会凭空累加整圈。 */
        int32_t delta = (int32_t)raw - (int32_t)s_previous_raw;
        if (delta > 32768) { delta -= 65536; }
        else if (delta < -32768) { delta += 65536; }
        s_continuous_raw += delta;
    }
    s_previous_raw = raw;
    hwt101_publish(true, HAL_GetTick());
}

int app_imu_hwt101_init(UART_HandleTypeDef *huart)
{
    s_inited = false;
    s_yaw_started = false;
    s_previous_raw = 0;
    s_continuous_raw = 0;
    hwt101_rx_bytes = 0U;
    hwt101_angle_frames = 0U;
    s_last_poll_ms = HAL_GetTick();
    hwt101_publish(false, s_last_poll_ms);
    if (huart == NULL) {
        return RET_PARAM;
    }
    int ret = bsp_uart_attach(BSP_UART_HWT101, huart);
    if (ret != RET_OK) {
        return ret;
    }
    proto_hwt101_init(hwt101_on_yaw);
    s_rx_overflow = bsp_uart_get_rx_overflow(BSP_UART_HWT101);
    ret = bsp_uart_start_rx(BSP_UART_HWT101);
    s_inited = (ret == RET_OK);
    return ret;
}

int app_imu_hwt101_poll(void)
{
    if (!s_inited) {
        return RET_NOTREADY;
    }
    uint32_t now = HAL_GetTick();
    uint32_t overflow = bsp_uart_get_rx_overflow(BSP_UART_HWT101);
    bool overrun = overflow != s_rx_overflow;
    bool delayed = (uint32_t)(now - s_last_poll_ms) > APP_IMU_HWT101_TIMEOUT_MS;
    s_last_poll_ms = now;
    s_rx_overflow = overflow;
    uint16_t remaining = bsp_uart_available(BSP_UART_HWT101);
    uint16_t frames = 0U;
    uint8_t buf[HWT101_CHUNK_SIZE];
    if (overrun || delayed) {
        proto_hwt101_reset_rx();
        hwt101_publish(false, now);
    }
    /* 只消费本轮开始时的积压；ISR继续写head，这里只通过read推进tail。
     * 停顿/溢出后丢弃积压，避免把旧帧盖上当前时间当成新反馈。 */
    while (remaining > 0U) {
        uint16_t request = MIN_OF(remaining, (uint16_t)sizeof(buf));
        uint16_t got = bsp_uart_read(BSP_UART_HWT101, buf, request);
        if (got == 0U) {
            break;
        }
        hwt101_rx_bytes += got;
        remaining = (uint16_t)(remaining - got);
        if (!overrun && !delayed) {
            frames = (uint16_t)(frames + proto_hwt101_feed(buf, got));
        }
    }
    if (overrun) { return RET_ERROR; }
    if (delayed) { return RET_TIMEOUT; }
    if (frames > 0U) {
        return RET_OK;
    }
    double yaw;
    return app_imu_hwt101_getangle(&yaw) ? RET_IDLE : RET_TIMEOUT;
}

bool app_imu_hwt101_getangle(double *angle_rad)
{
    if (angle_rad == NULL) {
        return false;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    hwt101_snapshot_t snap = s_snap;
    __set_PRIMASK(primask);
    bool valid = snap.valid &&
        ((uint32_t)(HAL_GetTick() - snap.timestamp_ms) <= APP_IMU_HWT101_TIMEOUT_MS);
    *angle_rad = valid ? snap.yaw : 0.0;
    return valid;
}

