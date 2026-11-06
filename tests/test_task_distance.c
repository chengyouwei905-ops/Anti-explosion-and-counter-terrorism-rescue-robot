/* Historical filename retained. The production correction now uses the timed
 * linear API; the standalone mileage API stays covered in test_chassis_motion. */
#define main full_route_fixture_main
#include "test_task_vision.c"
#undef main

static void begin_translation_correction(void)
{
    enter_work(WORK_QR,false);
    px+=60;py-=45; /* Exactly 75mm planar error; no visual yaw correction. */
    until_phase(WORK_WAIT_MOTION,4000);
    assert(app_task_work_get_status()->corrections==1);
}

int main(void)
{
    for(unsigned rollover=0;rollover<2;++rollover) {
        reset(rollover?UINT32_MAX-1000U:0);
        encoder_ready=false;encoder_age=501; /* Timed correction must not require odometry. */
        begin_translation_correction();
        until_phase(WORK_COMPLETE,8000);
        assert(hypotf(px-anchor[WORK_QR][0],py-anchor[WORK_QR][1])<=20);
        assert(app_task_work_get_status()->corrections==1 && send_count[0x10]==2);
        assert(encoder_distance>1074 && encoder_distance<1076);
    }
    /* Poor physical tracking ends each timed move, then vision requests another correction. */
    reset(0);translation_scale=.1f;encoder_ready=false;
    begin_translation_correction();
    until_phase(WORK_WAIT_LOCATE,4000);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_DONE);
    assert(hypotf(px-anchor[WORK_QR][0],py-anchor[WORK_QR][1])>20);
    until_retry(WORK_ERR_ALIGNMENT,20000);
    assert(app_task_work_get_status()->corrections==0);
    translation_scale=1;
    until_phase(WORK_COMPLETE,10000);
    assert(hypotf(px-anchor[WORK_QR][0],py-anchor[WORK_QR][1])<=20);
    /* Restored planned-time watchdog, rather than the old 31-second mileage wait. */
    reset(0);begin_translation_correction();
    uint32_t started=elapsed;
    while(task_state!=TASK_FAULT && elapsed-started<6000U)step(false);
    assert(task_state==TASK_FAULT && app_task_work_get_status()->error==WORK_ERR_MOTION);
    reset(0);begin_translation_correction();command("TOTALSTOP!");step(true);
    assert(task_state==TASK_IDLE && !laser && app_chassis_motion_get_state()==MOTION_STATE_IDLE);
    /* Scan moves, bucket transport and return still work without encoder feedback. */
    reset(0);encoder_ready=false;enter_work(WORK_BOMB,false);
    until_phase(WORK_COMPLETE,60000);
    assert(pick_calls==1 && place_calls==1 && app_task_work_get_status()->corrections==0);
    test_height_diagnostics();test_stop_and_errors();test_bucket_transport();
    puts("PASS: timed correction without odometry, visual remeasurement, correction-batch retry, planned-time watchdog, rollover and STOP");
    return 0;
}
