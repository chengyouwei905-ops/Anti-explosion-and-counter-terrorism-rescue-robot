#include <assert.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include "app_chassis_motion.h"
#include "app_chassis_closeloop.h"
#include "alg_motion_profile.h"

#define PI 3.14159265358979323846f
static bool imu_valid = true;
static float imu_yaw;
static float out_vx, out_vy, out_omega;
static unsigned sends;
static float world_x, world_y;

bool app_imu_get_euler(float euler[3])
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
    world_x += (cosf(midpoint_yaw) * out_vx - sinf(midpoint_yaw) * out_vy) * dt;
    world_y += (sinf(midpoint_yaw) * out_vx + cosf(midpoint_yaw) * out_vy) * dt;
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
    test_closed_loop();
    puts("PASS: profile position/speed, omnidirectional lines, signed arcs, relative headings, pivots, sequencing, stop and radian closed loop.");
    return 0;
}
