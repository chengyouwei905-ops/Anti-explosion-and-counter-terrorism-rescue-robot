/* Real UART ISR -> BSP ring -> poll -> protocol -> yaw -> motion/closed loop. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_imu_hwt101.h"
#include "app_chassis_closeloop.h"
#include "app_chassis_motion.h"
#include "app_chassis.h"

#define PI 3.14159265358979323846
uint32_t test_hwt101_primask;
static uint32_t tick_ms;
static uint8_t *rx_byte;
static UART_HandleTypeDef uart;
static HAL_StatusTypeDef rx_result = HAL_OK;
static float vx, vy, omega;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
uint32_t HAL_GetTick(void) { return tick_ms; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
    assert(huart == &uart && len == 1U);
    rx_byte = data;
    return rx_result;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n, uint32_t t)
{
    (void)h; (void)d; (void)n; (void)t;
    assert(!"HWT101 driver must not block on transmit"); return HAL_ERROR;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, const uint8_t *d, uint16_t n)
{
    (void)h; (void)d; (void)n;
    assert(!"receive-only driver must not change sensor configuration"); return HAL_ERROR;
}
int app_chassis_move(float x, float y, float w) { vx=x;vy=y;omega=w;return RET_OK; }
bool app_chassis_get_distance_sample(Chassis_DistanceSample_t *out)
{ (void)out; return false; }

static void inject(const uint8_t *data, unsigned len)
{
    for (unsigned i=0U; i<len; i++) {
        *rx_byte=data[i];
        HAL_UART_RxCpltCallback(&uart);
    }
}
static void frame(uint8_t out[11], uint8_t type, int16_t yaw)
{
    memset(out, 0, 11U);
    out[0]=0x55;out[1]=type;
    /* HWT101: four reserved zeros, signed YawL/YawH, version VL/VH. */
    out[8]=0x55;out[9]=0xA1; /* Version bytes are included in the checksum. */
    le_i16_put(out+6,yaw);
    for (unsigned i=0U;i<10U;i++) out[10]=(uint8_t)(out[10]+out[i]);
}
static void angle(int16_t raw)
{
    uint8_t data[11];frame(data,0x53,raw);inject(data,11);
    assert(app_imu_hwt101_poll()==RET_OK);
}
static void near_yaw(double expected)
{
    double yaw=100.0;
    assert(app_chassis_closeloop_get_yaw(&yaw));
    assert(fabs(yaw-expected)<1e-10);
}
static void init(uint32_t now)
{
    tick_ms=now;rx_result=HAL_OK;
    assert(app_imu_hwt101_init(&uart)==RET_OK);
    app_chassis_motion_init();
}
static void test_stream(void)
{
    uint8_t data[11];double yaw=123;
    assert(app_imu_hwt101_poll()==RET_NOTREADY);
    assert(!app_imu_hwt101_getangle(&yaw) && yaw==0.0);
    assert(app_imu_hwt101_init(NULL)==RET_PARAM);
    rx_result=HAL_ERROR;
    assert(app_imu_hwt101_init(&uart)==RET_ERROR);
    assert(app_imu_hwt101_poll()==RET_NOTREADY);
    init(0U);
    app_chassis_closeloop_set_target(1,2,3);
    app_chassis_closeloop_update();
    assert(vx==0 && vy==0 && omega==0);
    assert(!app_imu_hwt101_getangle(NULL));
    frame(data,0x53,32000);
    inject(data,5);(void)app_imu_hwt101_poll();
    assert(!app_imu_hwt101_getangle(&yaw));
    inject(data+5,6);assert(app_imu_hwt101_poll()==RET_OK);near_yaw(0);
    test_hwt101_primask=1U;
    angle(-32000);near_yaw(APP_IMU_HWT101_YAW_SIGN*1536.0*PI/32768.0);
    assert(test_hwt101_primask==1U);
    test_hwt101_primask=0U;
    angle(32000);near_yaw(0);
    assert(test_hwt101_primask==0U);
    /* CRC failure must leave the last valid value intact. */
    frame(data,0x53,0);data[10]^=1U;inject(data,11);
    assert(app_imu_hwt101_poll()==RET_IDLE);near_yaw(0);
    uint8_t noise[]={0x11,0x22,0x55,0x53,0x01};inject(noise,sizeof(noise));
    frame(data,0x53,32100);inject(data,11);
    assert(app_imu_hwt101_poll()==RET_OK);near_yaw(APP_IMU_HWT101_YAW_SIGN*100.0*PI/32768.0);
    /* Lost byte followed by a good frame, split over many poll calls. */
    frame(data,0x53,32200);inject(data,7);inject(data+8,3);
    frame(data,0x53,32300);
    for(unsigned i=0;i<11;i++) { inject(data+i,1);(void)app_imu_hwt101_poll(); }
    near_yaw(APP_IMU_HWT101_YAW_SIGN*300.0*PI/32768.0);
    uart.ErrorCode=1U;HAL_UART_ErrorCallback(&uart);
    assert(uart.ErrorCode==HAL_UART_ERROR_NONE);
    angle(32400);near_yaw(APP_IMU_HWT101_YAW_SIGN*400.0*PI/32768.0);
}
static void test_continuous(void)
{
    init(0);angle(0);
    const int16_t positive[]={16384,-32768,-16384,0};
    for(unsigned turn=0;turn<4;turn++) {
        /* All four frames may be consumed in a single poll. */
        for(unsigned j=0;j<4;j++) { uint8_t d[11];frame(d,0x53,positive[j]);inject(d,11); }
        assert(app_imu_hwt101_poll()==RET_OK);
        near_yaw(APP_IMU_HWT101_YAW_SIGN*(turn+1)*2.0*PI);
    }
    const int16_t negative[]={-16384,-32768,16384,0};
    for(unsigned turn=0;turn<8;turn++) {
        for(unsigned j=0;j<4;j++) angle(negative[j]);
        near_yaw(APP_IMU_HWT101_YAW_SIGN*(3.0-(double)turn)*2.0*PI);
    }
    /* Thousands of ISR writes exercise actual ring-index wraparound. */
    for(unsigned i=0;i<1000;i++) angle(0);
    assert(bsp_uart_get_rx_overflow(BSP_UART_HWT101)==0U);
}
static void test_timeout_overflow(void)
{
    double yaw;
    init(UINT32_MAX-200U);angle(0);angle(100);
    tick_ms+=APP_IMU_HWT101_TIMEOUT_MS;
    near_yaw(APP_IMU_HWT101_YAW_SIGN*100.0*PI/32768.0);
    (void)app_imu_hwt101_poll();
    tick_ms++;
    assert(!app_imu_hwt101_getangle(&yaw) && yaw==0.0);
    /* A valid gyro packet is not a fresh yaw packet. */
    uint8_t d[11];frame(d,0x52,200);inject(d,11);
    assert(app_imu_hwt101_poll()==RET_TIMEOUT);
    assert(!app_imu_hwt101_getangle(&yaw));
    angle(200);near_yaw(APP_IMU_HWT101_YAW_SIGN*200.0*PI/32768.0);
    /* Old buffered angles after a stalled main loop must be discarded. */
    frame(d,0x53,300);inject(d,11);
    tick_ms+=APP_IMU_HWT101_TIMEOUT_MS+1U;
    assert(app_imu_hwt101_poll()==RET_TIMEOUT);
    assert(!app_imu_hwt101_getangle(&yaw));
    angle(400);near_yaw(APP_IMU_HWT101_YAW_SIGN*400.0*PI/32768.0);
    for(unsigned i=0;i<60;i++) inject(d,11);
    assert(bsp_uart_get_rx_overflow(BSP_UART_HWT101)>0U);
    assert(app_imu_hwt101_poll()==RET_ERROR);
    assert(!app_imu_hwt101_getangle(&yaw));
    assert(bsp_uart_available(BSP_UART_HWT101)==0U);
    angle(500);near_yaw(APP_IMU_HWT101_YAW_SIGN*500.0*PI/32768.0);
}
static void test_boundary_jitter(void)
{
    init(0);angle(0);angle(16384);angle(32767);
    app_chassis_closeloop_init();
    app_chassis_closeloop_set_target(0,0,(float)(APP_IMU_HWT101_YAW_SIGN*PI));
    /* Repeated +/-180-degree reports must not accumulate full turns or kick PID. */
    for(unsigned i=0;i<2000;i++) {
        angle((i&1U)?32767:-32768);
        near_yaw(APP_IMU_HWT101_YAW_SIGN*((i&1U)?32767.0:32768.0)*PI/32768.0);
        app_chassis_closeloop_update();
        assert(fabsf(omega)<0.02f);
    }
    angle(16384);angle(0);near_yaw(0);
    /* Strictly greater than 180: an exact -180 delta is not flipped to +180. */
    init(0);angle(0);angle(-32768);near_yaw(-APP_IMU_HWT101_YAW_SIGN*PI);
    for(unsigned i=0;i<2000;i++) {
        angle((i&1U)?-32768:32767);
        near_yaw(APP_IMU_HWT101_YAW_SIGN*((i&1U)?-32768.0:-32769.0)*PI/32768.0);
    }
    angle(0);near_yaw(0);
}

static void test_motion_feedback(void)
{
    init(0);
    assert(app_chassis_motion_arc_translate(100,1,0,80,100,100)==MOTION_START_NO_IMU);
    angle(16000);angle(16100);
    double yaw;assert(app_chassis_closeloop_get_yaw(&yaw));
    /* A zero-error target must use the same HWT101 baseline as feedback. */
    app_chassis_closeloop_init();
    app_chassis_closeloop_set_target(20,30,(float)yaw);
    app_chassis_closeloop_update();
    assert(vx==20 && vy==30 && omega==0);
    app_chassis_motion_linear_move(100,0,80,100,100);
    app_chassis_motion_update(.01f);app_chassis_closeloop_update();
    assert(vy>0 && fabsf(vx)<1e-5f && omega==0);
    /* Multi-turn targets keep their requested direction. */
    app_chassis_closeloop_set_target(0,0,(float)(yaw+2*PI));
    app_chassis_closeloop_update();assert(omega>0);
    tick_ms+=APP_IMU_HWT101_TIMEOUT_MS+1U;
    app_chassis_motion_update(.01f);app_chassis_closeloop_update();
    assert(vx==0 && vy==0 && omega==0);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_IMU_ERROR);
}

int main(void)
{
    test_stream();test_continuous();test_boundary_jitter();test_timeout_overflow();test_motion_feedback();
    puts("PASS: HWT101 ISR/ring/poll, fragmented/corrupt frames, continuous yaw, timeout/overflow and chassis feedback");
    return 0;
}
