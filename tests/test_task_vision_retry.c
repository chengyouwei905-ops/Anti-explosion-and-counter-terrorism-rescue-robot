/* Real task/work/UART/protocol/motion; exercise recovery without K230 CANCEL. */
#define main legacy_task_fixture_main
#include "test_task_vision.c"
#undef main

static uint8_t work_command(TaskWorkKind_t kind,bool scan)
{
    const uint8_t measure[]={VISION_CMD_QR,VISION_CMD_BOMB_LOCATE,
        VISION_CMD_TARGET_LOCATE,VISION_CMD_RESCUE_LOCATE};
    const uint8_t scans[]={VISION_CMD_QR,VISION_CMD_BOMB_SCAN,
        VISION_CMD_TARGET_SCAN,VISION_CMD_RESCUE_SCAN};
    return scan ? scans[kind] : measure[kind];
}

static void pending_work(TaskWorkKind_t kind,bool scan,uint32_t base)
{
    reset(base);scene_slot=1;enter_work(kind,false);
    if(scan) automatic_reply=false;
    until_phase(scan ? WORK_WAIT_SCAN : WORK_WAIT_LOCATE,10000);
    automatic_reply=false;
}

static void assert_retry(TaskWorkError_t reason)
{
    until_phase(WORK_RETRY_VISION,20);
    const TaskWorkStatus_t *status=app_task_work_get_status();
    assert(task_running && task_state!=TASK_FAULT && status->error==WORK_ERR_NONE);
    assert(status->last_retry_error==reason && status->vision_retries>0);
    assert(!status->request_pending && !status->alignment_valid && !status->cancel_pending);
    assert(!laser && arm_stops==0 && send_count[VISION_CMD_CANCEL]==0);
}

static void successful_retry(TaskWorkKind_t kind,bool scan,uint8_t old_seq)
{
    unsigned count=send_count[work_command(kind,scan)];
    float x=px,y=py;
    /* Late success after rejection must never trigger laser/grasp/movement. */
    uint8_t payload[64];uint8_t size=make_scene_result(payload);
    deliver(VISION_CMD_RESULT,payload,size);
    for(unsigned i=0;i<100;++i)step(true);
    assert(hypotf(px-x,py-y)<.01f && !laser && pick_calls==0);
    assert(send_count[work_command(kind,scan)]==count);
    until_phase(scan ? WORK_WAIT_SCAN : WORK_WAIT_LOCATE,1000);
    assert(tx_seq!=old_seq && send_count[work_command(kind,scan)]==count+1);
    result_header(payload,tx_cmd,old_seq,VISION_FAILURE,9);
    deliver(VISION_CMD_RESULT,payload,15);step(true);
    assert(app_task_work_get_status()->request_pending && task_running);
    automatic_reply=true;scene_samples=5;
    until_phase(WORK_COMPLETE,40000);
    assert(send_count[VISION_CMD_CANCEL]==0 && arm_stops==0);
    const Task_State_t next[]={TASK_MOVE2,TASK_MOVE12,TASK_MOVE13,TASK_MOVE16};
    step(true);
    assert(task_state==next[kind] && app_chassis_motion_get_state()==MOTION_STATE_RUNNING);
}

static void failed_measurements(void)
{
    const uint8_t statuses[]={VISION_FAILURE,VISION_CANCELLED};
    const uint8_t reasons[]={3,9,19,1};
    for(unsigned kind=0;kind<WORK_COUNT;++kind) {
        for(unsigned scan=0;scan<2;++scan) {
            if(kind==WORK_QR && scan) continue;
            for(unsigned i=0;i<sizeof(reasons);++i) {
                pending_work((TaskWorkKind_t)kind,scan,i%2 ? UINT32_MAX-1000U : 0);
                uint8_t old_seq=tx_seq,payload[64];
                result_header(payload,tx_cmd,tx_seq,statuses[i==3],reasons[i]);
                payload[6]=kind==WORK_QR ? 0 : 1;
                deliver(VISION_CMD_RESULT,payload,15);
                assert_retry(WORK_ERR_VISION);
                assert(app_task_work_get_status()->vision_reason==reasons[i]);
                successful_retry((TaskWorkKind_t)kind,scan,old_seq);
            }
        }
    }
}

static void timeout_and_heading(void)
{
    for(unsigned kind=0;kind<WORK_COUNT;++kind) {
        for(unsigned scan=0;scan<2;++scan) {
            if(kind==WORK_QR && scan) continue;
            pending_work((TaskWorkKind_t)kind,scan,UINT32_MAX-1000U);
            uint8_t old_seq=tx_seq;
            float x=px,y=py;
            until_phase(WORK_RETRY_VISION,66000);
            assert_retry(WORK_ERR_TIMEOUT);
            assert(hypotf(px-x,py-y)<.01f);
            successful_retry((TaskWorkKind_t)kind,scan,old_seq);

            if(kind==WORK_QR)continue; /* QR task content is independent of observation yaw. */
            pending_work((TaskWorkKind_t)kind,scan,0);
            old_seq=tx_seq;
            yaw+=.05f;
            assert_retry(WORK_ERR_OBSERVATION_MOVED);
            successful_retry((TaskWorkKind_t)kind,scan,old_seq);
        }
    }
}

static void invalid_results_and_alignment(void)
{
    for(unsigned fault=0;fault<6;++fault) {
        pending_work(WORK_TARGET,false,0);
        uint8_t payload[64];uint8_t old_seq=tx_seq;
        if(fault==5)px+=400; /* Outside the correction envelope; no motion allowed. */
        uint8_t size=make_scene_result(payload);
        if(fault==0)payload[7]=2;
        if(fault==1)put_float(payload+size-4,30);
        if(fault==2)payload[6]=2;
        if(fault==3)payload[4]&=(uint8_t)~VISION_CONTROL_READY;
        if(fault==4)put_float(payload+15,100000);
        deliver(VISION_CMD_RESULT,payload,size);
        if(fault==3) {
            /* A structurally invalid success is ignored until the request timeout. */
            step(true);assert(app_task_work_get_status()->request_pending);
            until_phase(WORK_RETRY_VISION,66000);
        }
        assert_retry(fault==5 ? WORK_ERR_ALIGNMENT : fault==3 ? WORK_ERR_TIMEOUT : WORK_ERR_DATA);
        if(fault==5)px-=400;
        successful_retry(WORK_TARGET,false,old_seq);
    }
    reset(0);scene_slot=1;enter_work(WORK_TARGET,false);px+=60;py-=45;translation_scale=.1f;
    until_phase(WORK_RETRY_VISION,40000);
    assert_retry(WORK_ERR_ALIGNMENT);
    assert(app_task_work_get_status()->corrections==0);
    translation_scale=1;
    until_phase(WORK_LASER,40000);
    assert(hypotf(px-anchor[WORK_TARGET][0],py-anchor[WORK_TARGET][1])<=20);
    assert(send_count[VISION_CMD_CANCEL]==0);
    /* 5-frame results up to the current K230 20mm scatter limit remain actionable. */
    const float scatters[]={16.01f,20.0f,20.01f};
    for(unsigned i=0;i<sizeof(scatters)/sizeof(scatters[0]);++i) {
        pending_work(WORK_TARGET,false,0);
        uint8_t payload[64];scene_samples=5;
        uint8_t size=make_scene_result(payload);
        put_float(payload+size-4,scatters[i]);
        deliver(VISION_CMD_RESULT,payload,size);
        if(i==2)assert_retry(WORK_ERR_DATA);
        else {
            until_phase(WORK_COMPLETE,5000);
            assert(app_task_work_get_status()->vision_retries==0);
        }
    }
}

static void transmission_and_local_stop(void)
{
    for(unsigned i=0;i<2;++i) {
        reset(0);automatic_reply=false;
        force_tx=i ? HAL_BUSY : HAL_ERROR;
        enter_work(WORK_QR,false);
        until_phase(WORK_RETRY_VISION,4000);
        assert_retry(WORK_ERR_TX);
        for(unsigned j=0;j<3000;++j)step(true);
        assert(task_running && task_state!=TASK_FAULT && send_count[VISION_CMD_CANCEL]==0);
        force_tx=HAL_OK;automatic_reply=true;
        until_phase(WORK_COMPLETE,10000);
    }
    /* Both public send APIs refuse CANCEL without transmitting or consuming SEQ. */
    reset(0);automatic_reply=false;
    unsigned sent=0;
    uint8_t seq=199;
    for(unsigned i=0;i<256;++i)sent+=send_count[i];
    assert(app_vision_send_with_seq(VISION_CMD_CANCEL,NULL,0,&seq)==RET_PARAM && seq==199);
    assert(app_vision_send(VISION_CMD_CANCEL,NULL,0)==RET_PARAM);
    unsigned after=0;for(unsigned i=0;i<256;++i)after+=send_count[i];
    assert(after==sent && send_count[VISION_CMD_CANCEL]==0);

    pending_work(WORK_TARGET,false,0);
    command("TOTALSTOP!");step(true);
    assert(task_state==TASK_IDLE && !task_running && !laser && arm_stops==1);
    assert(!app_task_work_cancel_pending() && send_count[VISION_CMD_CANCEL]==0);
    command("TOTALSTART!");step(true);
    assert(task_state==TASK_MOVE1 && task_running && send_count[VISION_CMD_CANCEL]==0);

    pending_work(WORK_QR,false,0);imu_ok=false;until_fault(20);
    for(unsigned i=0;i<30;++i)step(true);
    assert(app_task_work_get_status()->error==WORK_ERR_IMU && send_count[VISION_CMD_CANCEL]==0);
    reset(0);enter_work(WORK_QR,false);arm_wait=true;until_fault(5000);
    for(unsigned i=0;i<30;++i)step(true);
    assert(app_task_work_get_status()->error==WORK_ERR_ARM && send_count[VISION_CMD_CANCEL]==0);
}

int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    failed_measurements();timeout_and_heading();invalid_results_and_alignment();
    transmission_and_local_stop();
    puts("PASS: all work kinds retry failed/cancelled/timeout/bad-data/heading/TX/alignment observations, reject stale replies, complete and start next MOVE; local STOP/fault never transmits CANCEL");
    return 0;
}
