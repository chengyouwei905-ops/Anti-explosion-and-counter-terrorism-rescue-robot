#include <assert.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "app_chassis_motion.h"
#include "app_chassis_closeloop.h"
#include "app_chassis.h"
#include "alg_motion_profile.h"

#define PI 3.14159265358979323846f
static bool imu_valid = true;
static float imu_yaw;
static float out_vx, out_vy, out_omega;
static unsigned sends;
static float world_x, world_y;
static float encoder_distance, speed_scale = 1.0f;
static bool encoder_ready = true, encoder_frozen;
static uint32_t encoder_age, encoder_reset;

float app_chassis_get_distance(void) { return encoder_distance; }
bool app_chassis_get_distance_sample(Chassis_DistanceSample_t *out)
{
    if (!encoder_ready || out == NULL) return false;
    *out = (Chassis_DistanceSample_t){app_chassis_get_distance(), encoder_age, encoder_reset, 0};
    return true;
}

bool app_imu_get_euler(double euler[3])
{
    euler[0] = euler[1] = 0.0f;
    euler[2] = imu_yaw;
    return imu_valid;
}

int app_chassis_move(float vx, float vy, float omega)
{
    assert(isfinite(vx) && isfinite(vy) && isfinite(omega));
    out_vx = vx;
    out_vy = vy;
    out_omega = omega;
    ++sends;
    return 0;
}

static void near_value(float actual, float expected, float tolerance)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "actual=%g expected=%g tolerance=%g\n", actual, expected, tolerance);
        assert(0);
    }
}

static void reset(float yaw)
{
    imu_valid = true;
    imu_yaw = yaw;
    world_x = world_y = 0.0f;
    encoder_distance = 1000.0f; /* A command must preserve the global accumulated mileage. */
    speed_scale = 1.0f;
    encoder_ready = true; encoder_frozen = false; encoder_age = encoder_reset = 0U;
    app_chassis_motion_init();
    app_chassis_closeloop_update();
    near_value(out_vx, 0.0f, 1e-6f);
    near_value(out_vy, 0.0f, 1e-6f);
    near_value(out_omega, 0.0f, 1e-6f);
}

/* Ideal wheel-speed tracking and IMU, without hardware/slip/communication delay. */
static void tick(float dt)
{
    unsigned previous_sends = sends;
    app_chassis_motion_update(dt);
    assert(sends == previous_sends); /* motion only sets targets */
    app_chassis_closeloop_update();
    assert(sends == previous_sends + 1U);
    float midpoint_yaw = imu_yaw + 0.5f * out_omega * dt;
    world_x += (cosf(midpoint_yaw) * out_vx - sinf(midpoint_yaw) * out_vy) * dt * speed_scale;
    world_y += (sinf(midpoint_yaw) * out_vx + cosf(midpoint_yaw) * out_vy) * dt * speed_scale;
    if (!encoder_frozen) encoder_distance += hypotf(out_vx, out_vy) * dt * speed_scale;
    imu_yaw += out_omega * dt;
}

static void finish(void)
{
    unsigned iterations = 0;
    while (app_chassis_motion_get_state() == MOTION_STATE_RUNNING) {
        tick(0.001f);
        assert(++iterations < 100000U);
    }
    near_value(out_vx, 0.0f, 1e-6f);
    near_value(out_vy, 0.0f, 1e-6f);
    /* Reference completion permits the yaw loop to finish settling. */
    for (unsigned i = 0; i < 2000U; ++i) {
        tick(0.001f);
    }
    near_value(out_omega, 0.0f, 0.003f);
}

static void test_profile(void)
{
    alg_motion_profile_init(25.0f, 100.0f, 100.0f, 100.0f);
    assert(!alg_motion_profile_update(0.5f));
    near_value(alg_motion_profile_get_speed(), 50.0f, 1e-5f);
    near_value(alg_motion_profile_get_distance(), 12.5f, 1e-5f);
    assert(!alg_motion_profile_update(0.25f));
    near_value(alg_motion_profile_get_speed(), 25.0f, 1e-5f);
    near_value(alg_motion_profile_get_distance(), 21.875f, 1e-5f);
    assert(!alg_motion_profile_update(NAN));
    assert(!alg_motion_profile_update(-1.0f));
    near_value(alg_motion_profile_get_distance(), 21.875f, 1e-5f);
    assert(alg_motion_profile_update(10.0f));
    near_value(alg_motion_profile_get_speed(), 0.0f, 1e-5f);
    near_value(alg_motion_profile_get_distance(), 25.0f, 1e-5f);
    assert(alg_motion_profile_update(1.0f));

    /* Asymmetric triangular profile: peak=50, acceleration=1s, deceleration=0.5s. */
    alg_motion_profile_init(37.5f, 100.0f, 50.0f, 100.0f);
    assert(!alg_motion_profile_update(1.25f));
    near_value(alg_motion_profile_get_distance(), 34.375f, 1e-5f);
    assert(alg_motion_profile_update(0.25f));
    near_value(alg_motion_profile_get_distance(), 37.5f, 1e-5f);

    alg_motion_profile_init(200.0f, 100.0f, 100.0f, 100.0f);
    assert(!alg_motion_profile_update(1.5f));
    near_value(alg_motion_profile_get_distance(), 100.0f, 1e-5f);
    assert(!alg_motion_profile_update(1.0f));
    near_value(alg_motion_profile_get_distance(), 187.5f, 1e-5f);
    assert(alg_motion_profile_update(0.5f));
    near_value(alg_motion_profile_get_distance(), 200.0f, 1e-5f);
    alg_motion_profile_init(100.0f, 100.0f, 0.0f, 100.0f);
    assert(alg_motion_profile_update(0.01f));
    near_value(alg_motion_profile_get_speed(), 0.0f, 1e-5f);
    near_value(alg_motion_profile_get_distance(), 0.0f, 1e-5f);
}

static void test_motion(void)
{
    reset(0.7f);
    app_chassis_motion_linear_move(200.0f, PI / 2.0f, 100.0f, 100.0f, 100.0f);
    finish();
    near_value(world_x, -200.0f * cosf(0.7f), 0.15f);
    near_value(world_y, -200.0f * sinf(0.7f), 0.15f);
    near_value(imu_yaw, 0.7f, 1e-4f);

    reset(0.0f);
    app_chassis_motion_linear_move(25.0f, -PI / 2.0f, 100.0f, 100.0f, 100.0f);
    finish();
    near_value(world_x, 25.0f, 0.05f);
    near_value(world_y, 0.0f, 0.01f);

    for (int sign = -1; sign <= 1; sign += 2) {
        reset(0.0f);
        app_chassis_motion_circular_move(100.0f, PI / 2.0f, sign > 0, 50.0f, 50.0f, 50.0f);
        app_chassis_closeloop_update();
        near_value(out_omega, 0.0f, 1e-6f); /* No heading jump when accepting the command. */
        tick(0.01f);
        near_value(out_vx, 0.0f, 1e-4f);
        near_value(out_vy, 0.5f, 1e-5f); /* Start forwards at the measured yaw. */
        assert(sign * out_omega > 0.0f && sign * out_omega < 0.02f); /* No fast initial turn. */
        finish();
        near_value(world_x, -sign * 100.0f, 0.15f);
        near_value(world_y, 100.0f, 0.15f);
        near_value(imu_yaw, sign * PI / 2.0f, 0.001f);
        /* The next straight command must hold the NEW heading, not an old arc target. */
        float start_x = world_x, start_y = world_y;
        app_chassis_motion_linear_move(25.0f, 0.0f, 50.0f, 50.0f, 50.0f);
        finish();
        near_value(world_x - start_x, -sign * 25.0f, 0.1f);
        near_value(world_y - start_y, 0.0f, 0.1f);
    }

    /* The boolean selects the turn even with a nonzero yaw or negative input angle. */
    for (int sign = -1; sign <= 1; sign += 2) {
        for (int angle_sign = -1; angle_sign <= 1; angle_sign += 2) {
            reset(0.4f);
            app_chassis_motion_circular_move(100.0f, angle_sign * PI / 2.0f, sign > 0,
                                              50.0f, 50.0f, 50.0f);
            tick(0.01f);
            near_value(out_vx, 0.0f, 1e-4f);
            near_value(out_vy, 0.5f, 1e-5f);
            assert(sign * out_omega > 0.0f && sign * out_omega < 0.02f);
            finish();
            near_value(world_x, -sign * 100.0f * cosf(0.4f) - 100.0f * sinf(0.4f), 0.3f);
            near_value(world_y, -sign * 100.0f * sinf(0.4f) + 100.0f * cosf(0.4f), 0.3f);
            near_value(imu_yaw, 0.4f + sign * PI / 2.0f, 0.001f);
        }
    }

    for (int sign = -1; sign <= 1; sign += 2) {
        reset(0.8f);
        app_chassis_motion_circular_move(0.0f, sign * 2.5f * PI, sign > 0, 1.0f, 2.0f, 2.0f);
        finish();
        near_value(world_x, 0.0f, 1e-6f);
        near_value(world_y, 0.0f, 1e-6f);
        near_value(imu_yaw, 0.8f + sign * 2.5f * PI, 0.001f);
    }

    reset(1.0f);
    app_chassis_motion_circular_move(0.0f, 0.0f, true, 1.0f, 1.0f, 1.0f);
    tick(0.01f);
    near_value(out_omega, 0.0f, 1e-6f);
    app_chassis_motion_circular_move(-1.0f, 1.0f, false, 1.0f, 1.0f, 1.0f);
    assert(app_chassis_motion_get_state() == MOTION_STATE_IDLE);

    app_chassis_motion_linear_move(100.0f, 0.0f, 50.0f, 50.0f, 50.0f);
    tick(0.1f);
    near_value(out_vy, 5.0f, 1e-4f);
    app_chassis_motion_circular_move(0.0f, 1.0f, false, 1.0f, 1.0f, 1.0f); /* busy: ignored */
    tick(0.1f);
    near_value(out_vy, 10.0f, 1e-4f);
    app_chassis_motion_stop();
    tick(0.01f);
    near_value(out_vx, 0.0f, 1e-6f);
    near_value(out_vy, 0.0f, 1e-6f);
    near_value(out_omega, 0.0f, 1e-6f);

    imu_valid = false;
    app_chassis_motion_linear_move(100.0f, 0.0f, 50.0f, 50.0f, 50.0f);
    assert(app_chassis_motion_get_state() == MOTION_STATE_IDLE);
    tick(0.01f);
    near_value(out_vy, 0.0f, 1e-6f);

    reset(0.0f);
    app_chassis_motion_circular_move(100.0f, PI, true, 50.0f, 50.0f, 50.0f);
    tick(0.1f);
    imu_valid = false;
    tick(0.01f);
    assert(app_chassis_motion_get_state() == MOTION_STATE_IDLE);
    near_value(out_vy, 0.0f, 1e-6f);
    near_value(out_omega, 0.0f, 1e-6f);
    imu_valid = true;
    tick(0.01f);
    near_value(out_omega, -1.2f, 1e-5f); /* PD restart is limited after the missing IMU. */
}

static void test_arc_translation(void)
{
    const float headings[] = {0.0f, 0.7f, -2.0f, 7.0f};
    const float directions[] = {0.0f, PI / 2.0f, -PI / 2.0f, PI / 3.0f};
    const float angles[] = {PI / 2.0f, -PI / 2.0f, PI, -PI, 2.0f * PI, -0.35f};
    for (unsigned h = 0U; h < sizeof(headings) / sizeof(headings[0]); ++h) {
        for (unsigned d = 0U; d < sizeof(directions) / sizeof(directions[0]); ++d) {
            for (unsigned a = 0U; a < sizeof(angles) / sizeof(angles[0]); ++a) {
                reset(headings[h]);
                assert(app_chassis_motion_arc_translate(200.0f, angles[a], directions[d],
                                                        80.0f, 120.0f, 70.0f) == MOTION_START_OK);
                assert(app_chassis_motion_get_result() == MOTION_RESULT_RUNNING);
                /* Independent circle geometry in the fixed frame. */
                double tangent = (double)headings[h] + directions[d];
                double sign = angles[a] < 0.0f ? -1.0 : 1.0;
                float center_x = (float)(-sign * 200.0 * cos(tangent));
                float center_y = (float)(-sign * 200.0 * sin(tangent));
                unsigned ticks = 0U;
                while (app_chassis_motion_get_state() == MOTION_STATE_RUNNING) {
                    tick(0.001f);
                    near_value(imu_yaw, headings[h], 1e-5f);
                    near_value(out_omega, 0.0f, 1e-6f);
                    near_value(hypotf(world_x - center_x, world_y - center_y), 200.0f, 0.25f);
                    assert(hypotf(out_vx, out_vy) <= 80.001f);
                    assert(++ticks < 100000U);
                }
                assert(app_chassis_motion_get_result() == MOTION_RESULT_DONE);
                near_value(world_x, (float)(sign * 200.0 * (cos(tangent + angles[a]) - cos(tangent))), 0.25f);
                near_value(world_y, (float)(sign * 200.0 * (sin(tangent + angles[a]) - sin(tangent))), 0.25f);
                near_value(out_vx, 0.0f, 1e-6f);
                near_value(out_vy, 0.0f, 1e-6f);
            }
        }
    }

    /* A yaw disturbance changes body-frame components, not the world tangent. */
    reset(0.4f);
    assert(app_chassis_motion_arc_translate(200, PI / 2, 0, 80, 120, 70) == MOTION_START_OK);
    tick(0.1f);
    float expected_x = cosf(0.4f) * out_vx - sinf(0.4f) * out_vy;
    float expected_y = sinf(0.4f) * out_vx + cosf(0.4f) * out_vy;
    reset(0.4f);
    assert(app_chassis_motion_arc_translate(200, PI / 2, 0, 80, 120, 70) == MOTION_START_OK);
    imu_yaw = 0.5f;
    app_chassis_motion_update(0.1f);
    app_chassis_closeloop_update();
    near_value(cosf(imu_yaw) * out_vx - sinf(imu_yaw) * out_vy, expected_x, 1e-4f);
    near_value(sinf(imu_yaw) * out_vx + cosf(imu_yaw) * out_vy, expected_y, 1e-4f);
    assert(out_omega < 0.0f); /* Correct back to 0.4 rad, not toward the tangent. */

    reset(0.6f);
    assert(app_chassis_motion_arc_translate(200, PI / 2, 0, 80, 120, 70) == MOTION_START_OK);
    finish();
    float x = world_x, y = world_y;
    app_chassis_motion_linear_move(25, 0, 50, 50, 50);
    finish();
    near_value(world_x - x, -25.0f * sinf(0.6f), 0.05f);
    near_value(world_y - y, 25.0f * cosf(0.6f), 0.05f);
    near_value(imu_yaw, 0.6f, 1e-5f);
}

static void test_arc_translation_lifecycle(void)
{
    const float bad[][6] = {
        {0, 1, 0, 50, 50, 50}, {-1, 1, 0, 50, 50, 50},
        {NAN, 1, 0, 50, 50, 50}, {INFINITY, 1, 0, 50, 50, 50},
        {200, NAN, 0, 50, 50, 50}, {200, INFINITY, 0, 50, 50, 50},
        {200, 1, NAN, 50, 50, 50}, {200, 1, INFINITY, 50, 50, 50},
        {200, 1, 0, 0, 50, 50}, {200, 1, 0, NAN, 50, 50},
        {200, 1, 0, INFINITY, 50, 50}, {200, 1, 0, 50, -1, 50},
        {200, 1, 0, 50, INFINITY, 50}, {200, 1, 0, 50, 50, 0},
        {200, 1, 0, 50, 50, NAN}, {FLT_MAX, PI, 0, 50, 50, 50},
        {FLT_MIN, FLT_MIN, 0, 50, 50, 50}, {200, 1, 0, FLT_MIN, 50, 50}
    };
    reset(0.4f);
    for (unsigned i = 0U; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        assert(app_chassis_motion_arc_translate(bad[i][0], bad[i][1], bad[i][2],
                                                bad[i][3], bad[i][4], bad[i][5]) == MOTION_START_INVALID);
        assert(app_chassis_motion_get_state() == MOTION_STATE_IDLE);
        assert(app_chassis_motion_get_result() == MOTION_RESULT_IDLE);
    }
    imu_valid = false;
    assert(app_chassis_motion_arc_translate(200, 1, 0, 50, 50, 50) == MOTION_START_NO_IMU);
    imu_valid = true;
    imu_yaw = NAN;
    assert(app_chassis_motion_arc_translate(200, 1, 0, 50, 50, 50) == MOTION_START_NO_IMU);
    imu_yaw = 0.4f;
    assert(app_chassis_motion_arc_translate(200, 0, PI / 2, 50, 50, 50) == MOTION_START_OK);
    assert(app_chassis_motion_get_state() == MOTION_STATE_IDLE);
    assert(app_chassis_motion_get_result() == MOTION_RESULT_DONE);
    tick(0.01f);
    near_value(out_vx, 0, 1e-6f); near_value(out_vy, 0, 1e-6f); near_value(out_omega, 0, 1e-6f);

    assert(app_chassis_motion_arc_translate(200, PI / 2, 0, 50, 50, 50) == MOTION_START_OK);
    unsigned before = sends;
    app_chassis_motion_update(NAN);
    app_chassis_motion_update(0);
    app_chassis_motion_update(-1);
    assert(sends == before);
    tick(0.1f);
    near_value(hypotf(out_vx, out_vy), 5, 1e-5f);
    assert(app_chassis_motion_arc_translate(NAN, NAN, NAN, NAN, NAN, NAN) == MOTION_START_BUSY);
    assert(app_chassis_motion_path_start(NULL, 0, NULL) == MOTION_START_BUSY);
    assert(app_chassis_motion_path_translate_start(NULL, 0, NULL) == MOTION_START_BUSY);
    app_chassis_motion_linear_move(999, 0, 100, 100, 100); /* Busy: old API must not overwrite. */
    finish();
    near_value(world_x, -200 * cosf(0.4f) - 200 * sinf(0.4f), 0.25f);
    near_value(world_y, -200 * sinf(0.4f) + 200 * cosf(0.4f), 0.25f);

    assert(app_chassis_motion_arc_translate(200, -PI / 2, 0, 50, 50, 50) == MOTION_START_OK);
    tick(0.1f);
    app_chassis_motion_stop(); tick(0.01f);
    assert(app_chassis_motion_get_result() == MOTION_RESULT_CANCELLED);
    near_value(out_vx, 0, 1e-6f); near_value(out_vy, 0, 1e-6f); near_value(out_omega, 0, 1e-6f);

    assert(app_chassis_motion_arc_translate(200, 1, 0, 50, 50, 50) == MOTION_START_OK);
    imu_valid = false; tick(0.01f);
    assert(app_chassis_motion_get_result() == MOTION_RESULT_IMU_ERROR);
    assert(app_chassis_motion_get_state() == MOTION_STATE_IDLE);
    near_value(out_vx, 0, 1e-6f); near_value(out_vy, 0, 1e-6f); near_value(out_omega, 0, 1e-6f);
    reset(0.4f);
    assert(app_chassis_motion_arc_translate(200, 1, 0, 50, 50, 50) == MOTION_START_OK);
    tick(1000); /* A large timestep must stop with the original heading. */
    assert(app_chassis_motion_get_result() == MOTION_RESULT_DONE);
    near_value(out_vx, 0, 1e-6f); near_value(out_vy, 0, 1e-6f); near_value(out_omega, 0, 1e-6f);
}

static void test_distance_linear(void)
{
    const float headings[]={0,.7f,7};
    const float directions[]={0,PI/2,-PI/2,PI,PI/3};
    for(unsigned h=0;h<sizeof(headings)/sizeof(headings[0]);++h) {
        for(unsigned d=0;d<sizeof(directions)/sizeof(directions[0]);++d) {
            reset(headings[h]);speed_scale=.2f;
            assert(app_chassis_motion_linear_move_distance(25,directions[d],40,100,100)==MOTION_START_OK);
            assert(app_chassis_motion_arc_translate(200,PI/2,0,40,100,100)==MOTION_START_BUSY);
            unsigned ticks=0;
            while(app_chassis_motion_get_state()==MOTION_STATE_RUNNING) {
                tick(.01f);
                near_value(imu_yaw,headings[h],1e-5f);
                near_value(out_omega,0,1e-6f);
                assert(hypotf(out_vx,out_vy)<=40.001f);
                assert(++ticks<3000);
            }
            assert(app_chassis_motion_get_result()==MOTION_RESULT_DONE);
            float travel=hypotf(world_x,world_y);
            near_value(travel,5,.3f); /* 25mm request permits 20mm remaining. */
            near_value(world_x,-travel*sinf(headings[h]+directions[d]),.01f);
            near_value(world_y,travel*cosf(headings[h]+directions[d]),.01f);
            assert(encoder_distance>=1005 && encoder_distance<1005.5f);
            near_value(out_vx,0,1e-6f);near_value(out_vy,0,1e-6f);
        }
    }
    /* A startup dead zone must not consume the requested distance. */
    reset(.4f);speed_scale=0;
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_OK);
    for(unsigned i=0;i<100;++i)tick(.01f);
    assert(app_chassis_motion_get_state()==MOTION_STATE_RUNNING && encoder_distance==1000);
    speed_scale=.2f;finish();
    assert(app_chassis_motion_get_result()==MOTION_RESULT_DONE);
    near_value(hypotf(world_x,world_y),5,.3f);
    /* Original time-based API retains its old behavior under the same speed loss. */
    reset(.4f);speed_scale=.2f;
    app_chassis_motion_linear_move(25,0,40,100,100);finish();
    near_value(hypotf(world_x,world_y),5,.1f);

    reset(.4f);
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_OK);
    imu_yaw+=.05f;app_chassis_motion_update(.01f);app_chassis_closeloop_update();
    near_value(cosf(imu_yaw)*out_vx-sinf(imu_yaw)*out_vy,-sinf(.4f),1e-5f);
    near_value(sinf(imu_yaw)*out_vx+cosf(imu_yaw)*out_vy,cosf(.4f),1e-5f);
    assert(out_omega<0); /* The IMU still closes heading, independently of mileage. */
}

static void test_distance_linear_failures(void)
{
    reset(0);
    assert(app_chassis_motion_linear_move_distance(-1,0,40,100,100)==MOTION_START_INVALID);
    assert(app_chassis_motion_linear_move_distance(NAN,0,40,100,100)==MOTION_START_INVALID);
    assert(app_chassis_motion_linear_move_distance(25,INFINITY,40,100,100)==MOTION_START_INVALID);
    assert(app_chassis_motion_linear_move_distance(25,0,0,100,100)==MOTION_START_INVALID);
    assert(app_chassis_motion_linear_move_distance(25,0,40,-1,100)==MOTION_START_INVALID);
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,NAN)==MOTION_START_INVALID);
    assert(app_chassis_motion_linear_move_distance(FLT_MAX,0,40,100,100)==MOTION_START_INVALID);
    encoder_ready=false;
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_NO_ODOM);
    encoder_ready=true;encoder_age=501;
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_NO_ODOM);
    encoder_age=0;encoder_distance=NAN;
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_NO_ODOM);
    encoder_distance=1000;imu_valid=false;
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_NO_IMU);
    imu_valid=true;
    assert(app_chassis_motion_linear_move_distance(0,0,40,100,100)==MOTION_START_OK);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_DONE);
    assert(app_chassis_motion_linear_move_distance(20,0,40,100,100)==MOTION_START_OK);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_DONE && encoder_distance==1000);

    for(unsigned fault=0;fault<6;++fault) {
        reset(0);
        assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_OK);
        tick(.1f);
        switch(fault) {
        case 0:encoder_age=501;break;
        case 1:encoder_ready=false;break;
        case 2:++encoder_reset;break;
        case 3:encoder_distance=999.0f;break;
        case 4:encoder_distance=NAN;break;
        default:imu_valid=false;break;
        }
        tick(.01f);
        assert(app_chassis_motion_get_state()==MOTION_STATE_IDLE);
        assert(app_chassis_motion_get_result()==(fault==5?MOTION_RESULT_IMU_ERROR:MOTION_RESULT_ODOM_ERROR));
        near_value(out_vx,0,1e-6f);near_value(out_vy,0,1e-6f);
    }
    reset(0);encoder_frozen=true;
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_OK);
    for(unsigned i=0;i<210;++i)tick(.01f);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_STALLED);
    near_value(out_vx,0,1e-6f);near_value(out_vy,0,1e-6f);
    reset(0);encoder_frozen=true;
    assert(app_chassis_motion_linear_move_distance(100,0,40,100,100)==MOTION_START_OK);
    /* Small, real progress avoids STALLED; the overall timeout still applies. */
    for(unsigned i=0;i<31;++i) { encoder_distance+=.11f;tick(1); }
    assert(app_chassis_motion_get_result()==MOTION_RESULT_TIMEOUT);
    reset(.4f);
    assert(app_chassis_motion_linear_move_distance(25,0,40,100,100)==MOTION_START_OK);
    app_chassis_motion_update(NAN);app_chassis_motion_update(-1);
    tick(.1f);near_value(hypotf(out_vx,out_vy),10,1e-5f);
    app_chassis_motion_stop();tick(.01f);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_CANCELLED);
    near_value(out_vx,0,1e-6f);near_value(out_vy,0,1e-6f);
}

static void test_closed_loop(void)
{
    imu_valid = true;
    imu_yaw = 0.1f;
    app_chassis_closeloop_init();
    app_chassis_closeloop_update(); /* no set_target required: hold the default 0 rad */
    near_value(out_vx, 0.0f, 1e-6f);
    near_value(out_vy, 0.0f, 1e-6f);
    near_value(out_omega, -1.2f, 1e-5f); /* First error step includes D and reaches the limit. */
    app_chassis_closeloop_update();
    near_value(out_omega, -0.4f, 1e-5f); /* Constant 0.1 rad error: D=0, tuned Kp=4. */
    reset(1.2f);
    app_chassis_closeloop_set_target_ff(0.0f, 0.0f, 1.2f, -0.4f);
    app_chassis_closeloop_update();
    near_value(out_omega, -0.4f, 1e-5f); /* no rad/degree mismatch */
    app_chassis_closeloop_set_target(0.0f, 0.0f, 1.2f);
    app_chassis_closeloop_update();
    near_value(out_omega, 0.0f, 1e-6f); /* feedforward cleared */
    app_chassis_closeloop_set_target_ff(0.0f, 0.0f, 10.0f, 10.0f);
    app_chassis_closeloop_update();
    near_value(out_omega, 1.2f, 1e-6f);
    app_chassis_closeloop_set_target(NAN, 0.0f, 0.0f);
    app_chassis_closeloop_update();
    near_value(out_vx, 0.0f, 1e-6f);
    near_value(out_omega, 1.2f, 1e-6f); /* invalid input leaves the previous target unchanged */
}

int main(void)
{
    test_profile();
    test_motion();
    test_arc_translation();
    test_arc_translation_lifecycle();
    test_distance_linear();
    test_distance_linear_failures();
    test_closed_loop();
    puts("PASS: profile position/speed, lines, yaw-following arcs, fixed-heading arc translation, validation, lifecycle and radian closed loop.");
    return 0;
}
