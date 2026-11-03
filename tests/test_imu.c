#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Exercise nonzero slopes, including a negative axis, using the real IMU app. */
#define APP_IMU_GYRO_TEMP_COEFF_X  0.001
#define APP_IMU_GYRO_TEMP_COEFF_Y -0.002
#define APP_IMU_GYRO_TEMP_COEFF_Z  0.003
#include "../App/Src/app_imu.c"

uint32_t test_imu_primask;
typedef enum { CAL_OK, CAL_MOVING, CAL_READ_ERROR, CAL_NONLINEAR_HEATING } cal_mode_t;
static const double slopes[3] = { 0.001, -0.002, 0.003 };
static const double no_motion[3] = { 0.0, 0.0, 0.0 };
static double ref_temp;
static double test_bias[3];
static uint32_t cal_reads;
static uint32_t cycles;
static bool calibrating;
static cal_mode_t cal_mode;
static ret_code_t read_result;
static bsp_imu_data_t sample;

ret_code_t bsp_imu_init(SPI_HandleTypeDef *hspi)
{
    return hspi != NULL ? RET_OK : RET_PARAM;
}

ret_code_t bsp_imu_read(bsp_imu_data_t *out)
{
    if (calibrating) {
        uint32_t n = cal_reads++;
        if (cal_mode == CAL_READ_ERROR && n == 2U) {
            calibrating = false;
            return RET_ERROR;
        }
        /* Physical average temperature is ref_temp. Nonlinear heating makes
         * the endpoint midpoint differ from the average of all samples. */
        double delta = n < APP_IMU_CALI_SAMPLES / 2U ? -5.0 : 5.0;
        if (cal_mode == CAL_NONLINEAR_HEATING) {
            delta = n < APP_IMU_CALI_SAMPLES / 4U ? -6.0 : 2.0;
        }
        *out = (bsp_imu_data_t){ .accel_mps2 = { 0.0, 0.0, 9.81 },
                                 .temp_c = ref_temp + delta };
        for (uint32_t axis = 0U; axis < 3U; axis++) {
            out->gyro_radps[axis] = test_bias[axis] + slopes[axis] * delta;
        }
        if (cal_mode == CAL_MOVING && n == APP_IMU_CALI_SAMPLES / 2U) {
            out->gyro_radps[2] += 0.2;
        }
        if (cal_reads == APP_IMU_CALI_SAMPLES) {
            calibrating = false;
        }
        return RET_OK;
    }
    *out = sample;
    return read_result;
}

void bsp_imu_get_init_fail(uint8_t *dev, uint8_t *reg, uint8_t *expect, uint8_t *actual)
{
    (void)dev; (void)reg; (void)expect; (void)actual;
}

void HAL_Delay(uint32_t ms) { (void)ms; }
void bsp_dwt_init(void) { cycles = 0U; }
bool bsp_dwt_is_ready(void) { return true; }
uint32_t bsp_dwt_get_cycles(void) { cycles += 2000U; return cycles; }
double bsp_dwt_cycles_to_s(uint32_t elapsed) { return (double)elapsed * 0.000001; }

static void init_imu(double mean_temp, const double bias[3], cal_mode_t mode)
{
    SPI_HandleTypeDef hspi = { 0U };
    ref_temp = mean_temp;
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        test_bias[axis] = bias[axis];
    }
    cal_reads = 0U;
    cal_mode = mode;
    calibrating = true;
    read_result = RET_OK;
    assert(app_imu_init(&hspi) == RET_OK);
    assert(app_imu_is_ready());
    assert(app_imu_is_calibrated() == (mode == CAL_OK || mode == CAL_NONLINEAR_HEATING));
    assert(app_imu_get_error_count() == 0U);
}

static void expect_gyro(const double expected[3])
{
    double gyro[3];
    assert(app_imu_get_gyro(gyro));
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        assert(fabs(gyro[axis] - expected[axis]) < 1e-13);
    }
}

static void poll_at(double temp, const double motion[3])
{
    sample = (bsp_imu_data_t){ .accel_mps2 = { 0.0, 0.0, 9.81 }, .temp_c = temp };
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        sample.gyro_radps[axis] = test_bias[axis] + slopes[axis] * (temp - ref_temp)
                              + motion[axis];
    }
    assert(app_imu_poll() == RET_OK);
    expect_gyro(motion);
    assert(app_imu_get_temp() == temp);
}

int main(void)
{
    const double bias[3] = { 0.01, -0.02, 0.03 };
    const double new_bias[3] = { -0.02, 0.01, -0.01 };
    const double motion[3] = { 0.2, -0.3, 0.4 };
    double euler[3];

    assert(app_imu_poll() == RET_NOTREADY);
    init_imu(25.0, bias, CAL_OK);
    assert(!app_imu_get_euler(euler));
    poll_at(25.0, no_motion);
    poll_at(35.0, no_motion);
    poll_at(15.0, no_motion);
    poll_at(35.0, no_motion);  /* Baseline stays fixed across polls. */
    assert(app_imu_get_euler(euler));
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        assert(fabs(euler[axis]) < 0.00001);
    }
    poll_at(35.0, motion);  /* Real rotation survives compensation. */

    read_result = RET_ERROR;
    sample.temp_c = 80.0;
    assert(app_imu_poll() == RET_ERROR);
    expect_gyro(motion);
    assert(app_imu_get_temp() == 35.0);
    assert(app_imu_get_error_count() == 1U);

    init_imu(45.0, new_bias, CAL_OK);  /* Both bias and temperature are replaced. */
    poll_at(55.0, no_motion);
    poll_at(35.0, motion);

    init_imu(30.0, bias, CAL_NONLINEAR_HEATING);
    assert(s_gyro_cali_start_temp_c == 24.0);
    assert(s_gyro_cali_end_temp_c == 32.0);
    assert(s_gyro_ref_temp_c == 30.0);
    assert(s_gyro_cali_mean_temp_c == 30.0);
    /* Nonlinear heating uses all samples, not the endpoints' midpoint (28 C). */
    poll_at(28.0, no_motion);
    poll_at(30.0, no_motion);
    poll_at(40.0, motion);
    poll_at(20.0, motion);

    for (cal_mode_t mode = CAL_MOVING; mode <= CAL_READ_ERROR; mode++) {
        init_imu(25.0, bias, mode);
        assert(s_gyro_cali_start_temp_c == 0.0);
        assert(s_gyro_cali_end_temp_c == 0.0);
        assert(s_gyro_cali_mean_temp_c == 0.0);
        assert(s_gyro_ref_temp_c == 0.0);
        /* A failed re-init calibration must use neither the old bias nor temperature. */
        sample = (bsp_imu_data_t){ .gyro_radps = { 0.1, -0.2, 0.3 }, .temp_c = 60.0 };
        assert(app_imu_poll() == RET_OK);
        expect_gyro(sample.gyro_radps);
    }

    /* Tiny biases with many significant digits must survive 8000-sample sums. */
    const double tiny_bias[3] = { 0.0010652644360316953, -0.000008123456789123,
                                 0.000000123456789123 };
    init_imu(30.0, tiny_bias, CAL_NONLINEAR_HEATING);
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        assert(fabs(s_gyro_bias[axis] - tiny_bias[axis]) < 1e-14);
    }
    poll_at(37.125, no_motion);

    /* A large continuous heading must retain small updates, including +/-pi wraps. */
    init_imu(25.0, no_motion, CAL_OK);
    s_yaw_cont = 10000.0;
    s_yaw_prev = 0.0;
    s_yaw_cont_ready = true;
    const double turning[3] = { 0.0, 0.0, 0.2 };
    for (uint32_t i = 0U; i < 20000U; i++) {
        poll_at(25.0, turning);
    }
    assert(app_imu_get_euler(euler));
    double expected_yaw = 10000.0 + 20000.0 * 2.0 * atan(0.5 * turning[2] * 0.002);
    assert(fabs(euler[AHRS_YAW] - expected_yaw) < 2e-8);

    /* Snapshot operations must preserve a caller's existing interrupt mask. */
    test_imu_primask = 1U;
    assert(app_imu_get_euler(euler));
    assert(app_imu_get_temp() == 25.0);
    assert(app_imu_poll() == RET_OK);
    assert(test_imu_primask == 1U);
    test_imu_primask = 0U;
    assert(app_imu_get_quat(NULL) == false);
    assert(app_imu_get_gyro(NULL) == false);
    assert(app_imu_get_accel(NULL) == false);
    assert(app_imu_get_rate_hz() > 499.999999999);
    assert(test_imu_primask == 0U);

    puts("imu tests passed");
    return 0;
}
