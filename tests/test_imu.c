#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Exercise nonzero slopes, including a negative axis, using the real IMU app. */
#define APP_IMU_GYRO_TEMP_COEFF_X  0.001f
#define APP_IMU_GYRO_TEMP_COEFF_Y -0.002f
#define APP_IMU_GYRO_TEMP_COEFF_Z  0.003f
#include "../App/Src/app_imu.c"

typedef enum { CAL_OK, CAL_MOVING, CAL_READ_ERROR, CAL_NONLINEAR_HEATING } cal_mode_t;
static const float slopes[3] = { 0.001f, -0.002f, 0.003f };
static const float no_motion[3] = { 0.0f, 0.0f, 0.0f };
static float ref_temp;
static float test_bias[3];
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
        float delta = n < APP_IMU_CALI_SAMPLES / 2U ? -5.0f : 5.0f;
        if (cal_mode == CAL_NONLINEAR_HEATING) {
            delta = n < APP_IMU_CALI_SAMPLES / 4U ? -6.0f : 2.0f;
        }
        *out = (bsp_imu_data_t){ .accel_mps2 = { 0.0f, 0.0f, 9.81f },
                                 .temp_c = ref_temp + delta };
        for (uint32_t axis = 0U; axis < 3U; axis++) {
            out->gyro_radps[axis] = test_bias[axis] + slopes[axis] * delta;
        }
        if (cal_mode == CAL_MOVING && n == APP_IMU_CALI_SAMPLES / 2U) {
            out->gyro_radps[2] += 0.2f;
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
float bsp_dwt_cycles_to_s(uint32_t elapsed) { return (float)elapsed * 0.000001f; }

static void init_imu(float mean_temp, const float bias[3], cal_mode_t mode)
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

static void expect_gyro(const float expected[3])
{
    float gyro[3];
    assert(app_imu_get_gyro(gyro));
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        assert(fabsf(gyro[axis] - expected[axis]) < 0.00002f);
    }
}

static void poll_at(float temp, const float motion[3])
{
    sample = (bsp_imu_data_t){ .accel_mps2 = { 0.0f, 0.0f, 9.81f }, .temp_c = temp };
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
    const float bias[3] = { 0.01f, -0.02f, 0.03f };
    const float new_bias[3] = { -0.02f, 0.01f, -0.01f };
    const float motion[3] = { 0.2f, -0.3f, 0.4f };
    float euler[3];

    assert(app_imu_poll() == RET_NOTREADY);
    init_imu(25.0f, bias, CAL_OK);
    assert(!app_imu_get_euler(euler));
    poll_at(25.0f, no_motion);
    poll_at(35.0f, no_motion);
    poll_at(15.0f, no_motion);
    poll_at(35.0f, no_motion);  /* Baseline stays fixed across polls. */
    assert(app_imu_get_euler(euler));
    for (uint32_t axis = 0U; axis < 3U; axis++) {
        assert(fabsf(euler[axis]) < 0.00001f);
    }
    poll_at(35.0f, motion);  /* Real rotation survives compensation. */

    read_result = RET_ERROR;
    sample.temp_c = 80.0f;
    assert(app_imu_poll() == RET_ERROR);
    expect_gyro(motion);
    assert(app_imu_get_temp() == 35.0f);
    assert(app_imu_get_error_count() == 1U);

    init_imu(45.0f, new_bias, CAL_OK);  /* Both bias and temperature are replaced. */
    poll_at(55.0f, no_motion);
    poll_at(35.0f, motion);

    init_imu(30.0f, bias, CAL_NONLINEAR_HEATING);
    assert(s_gyro_cali_start_temp_c == 24.0f);
    assert(s_gyro_cali_end_temp_c == 32.0f);
    assert(s_gyro_ref_temp_c == 28.0f);
    assert(s_gyro_cali_mean_temp_c == 30.0f);
    /* Public outputs must remain correct even though mean and reference differ. */
    poll_at(28.0f, no_motion);
    poll_at(30.0f, no_motion);
    poll_at(40.0f, motion);
    poll_at(20.0f, motion);

    for (cal_mode_t mode = CAL_MOVING; mode <= CAL_READ_ERROR; mode++) {
        init_imu(25.0f, bias, mode);
        assert(s_gyro_cali_start_temp_c == 0.0f);
        assert(s_gyro_cali_end_temp_c == 0.0f);
        assert(s_gyro_cali_mean_temp_c == 0.0f);
        assert(s_gyro_ref_temp_c == 0.0f);
        /* A failed re-init calibration must use neither the old bias nor temperature. */
        sample = (bsp_imu_data_t){ .gyro_radps = { 0.1f, -0.2f, 0.3f }, .temp_c = 60.0f };
        assert(app_imu_poll() == RET_OK);
        expect_gyro(sample.gyro_radps);
    }

    puts("imu tests passed");
    return 0;
}
