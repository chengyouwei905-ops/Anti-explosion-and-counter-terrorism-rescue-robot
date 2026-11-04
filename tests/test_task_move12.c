/* Reuse the real task/motion simulation without the old full-course coordinates. */
#define main task_path_fixture_main
#include "test_task_path.c"
#undef main

static void start_move12(uint32_t base, float yaw)
{
    reset(base);
    imu_yaw = yaw;
    task_running = true;
    task_state = TASK_EXPLOSIVE;
    tick(true); /* Completion of bomb work starts MOVE12 through task_poll. */
    assert(task_state == TASK_MOVE12);
    assert(app_chassis_motion_get_state() == MOTION_STATE_RUNNING);
}

static void test_move12(uint32_t base, float yaw)
{
    start_move12(base, yaw);
    while (app_chassis_motion_get_result() == MOTION_RESULT_RUNNING) {
        tick(true);
        assert(task_state == TASK_MOVE12 && elapsed_ms < 10000U);
        assert(fabsf(imu_yaw - yaw) < 1e-5f);
    }
    assert(app_chassis_motion_get_result() == MOTION_RESULT_DONE);
    /* Project the simulated displacement onto the starting right/forward axes. */
    float dx = px - 3350.0f, dy = py - 1750.0f;
    assert(fabsf(cosf(yaw) * dx + sinf(yaw) * dy - TASK_MOVE12_RIGHT_MM) < 0.2f);
    assert(fabsf(-sinf(yaw) * dx + cosf(yaw) * dy - task_route[11].distance) < 0.2f);
    assert(hypotf(out_vx, out_vy) < 1e-5f);
    uint32_t done_ms = elapsed_ms;
    while (task_state == TASK_MOVE12) {
        tick(true);
        assert(elapsed_ms - done_ms < 1100U);
    }
    assert(task_state == TASK_CS && elapsed_ms - done_ms >= 1000U);
}

static void test_final_offsets(uint32_t base, float yaw, bool grouped)
{
    reset(base);
    imu_yaw = yaw;
    task_running = true;
    task_state = grouped ? TASK_CS : TASK_RESCUE;
    tick(true);
    assert(task_state == (grouped ? TASK_MOVE13 : TASK_MOVE16));
    unsigned seams = 0U;
    AlgPathSample_t before, after;
    assert(app_chassis_motion_path_get_sample(&before));
    while (app_chassis_motion_get_result() == MOTION_RESULT_RUNNING) {
        tick(true);
        assert(task_state != TASK_FAULT && elapsed_ms < 20000U);
        assert(app_chassis_motion_path_get_sample(&after));
        if (after.segment != before.segment) {
            ++seams;
            assert(before.speed > 1.0f && after.speed > 1.0f);
        }
        before = after;
    }
    assert(app_chassis_motion_get_result() == MOTION_RESULT_DONE);
    assert(seams == (grouped ? 2U : 0U));
    float expected_right = grouped ? 200.0f + task_route[14].distance : 5.0f;
    float expected_forward = grouped ? task_route[12].distance + 200.0f - 5.0f :
                                       task_route[15].distance;
    /* Independent endpoint geometry: right of the final heading is backwards
     * relative to the group start after MOVE14's right-angle turn. */
    assert(fabsf(cosf(yaw) * after.x + sinf(yaw) * after.y - expected_right) < 0.02f);
    assert(fabsf(-sinf(yaw) * after.x + cosf(yaw) * after.y - expected_forward) < 0.02f);
    float dx = px - 3350.0f, dy = py - 1750.0f;
    assert(fabsf(cosf(yaw) * dx + sinf(yaw) * dy - expected_right) < 2.0f);
    assert(fabsf(-sinf(yaw) * dx + cosf(yaw) * dy - expected_forward) < 2.0f);
    uint32_t done_ms = elapsed_ms;
    Task_State_t final_move = grouped ? TASK_MOVE15 : TASK_MOVE16;
    while (task_state == final_move) {
        tick(true);
        assert(elapsed_ms - done_ms < 1100U);
    }
    assert(task_state == (grouped ? TASK_RESCUE : TASK_END));
    assert(elapsed_ms - done_ms >= 1000U);
    assert(fabsf(imu_yaw - (grouped ? yaw - PI / 2.0f : yaw)) < 0.002f);
    assert(hypotf(out_vx, out_vy) < 1e-5f);
}

int main(void)
{
    const float headings[] = {0.0f, 0.7f, PI / 2.0f, -1.0f};
    for (unsigned i = 0; i < sizeof(headings) / sizeof(headings[0]); ++i) {
        test_move12(0U, headings[i]);
        test_move12(UINT32_MAX - 1000U, headings[i]);
        test_final_offsets(0U, headings[i], true);
        test_final_offsets(UINT32_MAX - 1000U, headings[i], true);
        test_final_offsets(0U, headings[i], false);
        test_final_offsets(UINT32_MAX - 1000U, headings[i], false);
    }
    start_move12(0U, 0.0f);
    command("TOTALSTOP!"); tick(true);
    assert(task_state == TASK_IDLE && app_chassis_motion_get_state() == MOTION_STATE_IDLE);
    start_move12(0U, 0.0f);
    imu_valid = false;
    while (elapsed_ms < 20U) tick(true);
    assert(task_state == TASK_FAULT && app_chassis_motion_get_state() == MOTION_STATE_IDLE);
    start_move12(UINT32_MAX - 1000U, 0.0f);
    while (task_state != TASK_FAULT && elapsed_ms < 10000U) tick(false);
    assert(task_state == TASK_FAULT); /* Missing control updates must time out. */
    puts("PASS: configured MOVE12 offset; MOVE15/16 right 5 mm, preserved forward distances/headings, moving seams, 1 s dwell, rollover, stop, IMU and timeout");
    return 0;
}
