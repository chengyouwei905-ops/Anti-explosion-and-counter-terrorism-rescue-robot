/* A QR task-code ACK must start MOVE2 without any vision alignment or arm travel. */
#define main legacy_task_fixture_main
#include "test_task_vision.c"
#undef main

static void pending_qr(uint32_t base)
{
    reset(base);automatic_reply=false;enter_work(WORK_QR,false);
    px+=400;py-=300; /* Deliberately outside the old QR localization envelope. */
    until_phase(WORK_WAIT_LOCATE,4000);
}

static void acknowledge(void)
{
    uint8_t payload[64];result_header(payload,VISION_CMD_QR,tx_seq,VISION_SUCCESS,0);
    deliver(VISION_CMD_RESULT,payload,15);
}

static void qr_success(uint32_t base)
{
    pending_qr(base);
    float x=px,y=py;
    yaw+=.05f; /* Reading task content does not require a fixed observation yaw. */
    uint32_t started=elapsed;
    acknowledge();acknowledge();step(true);
    assert(app_task_work_get_status()->phase==WORK_QR_OPEN_CLAMP && clamp_open_calls==0);
    assert(!app_task_work_get_status()->alignment_valid && app_task_work_get_status()->corrections==0);
    assert(hypotf(px-x,py-y)<.01f && task_running && !laser);
    step(true);
    assert(task_state==TASK_MOVE2 && app_chassis_motion_get_state()==MOTION_STATE_RUNNING);
    assert(app_task_work_get_status()->phase==WORK_COMPLETE && clamp_open_calls==1);
    for(unsigned i=0;i<100;++i)step(true);
    assert(clamp_open_calls==1 && clamp_open_attempts==1); /* No repeated open while moving. */
    assert(elapsed-started<=102U && send_count[VISION_CMD_QR]==1 && send_count[VISION_CMD_CANCEL]==0);
}

static void strict_ack_and_stale_results(void)
{
    VisionResult_t decoded;
    uint8_t payload[64];result_header(payload,VISION_CMD_QR,42,VISION_SUCCESS,0);
    assert(proto_vision_decode_result(payload,15,&decoded));
    assert(proto_vision_result_qr_ready(&decoded));
    assert(!proto_vision_result_control_ready(&decoded,3,20));
    assert(!proto_vision_decode_result(payload,16,&decoded));
    const uint8_t offsets[]={4,6,7,14};
    for(unsigned i=0;i<sizeof(offsets);++i) {
        result_header(payload,VISION_CMD_QR,42,VISION_SUCCESS,0);payload[offsets[i]]=1;
        assert(!proto_vision_decode_result(payload,15,&decoded));
    }
    result_header(payload,VISION_CMD_BOMB_LOCATE,42,VISION_SUCCESS,0);
    assert(!proto_vision_decode_result(payload,15,&decoded));

    pending_qr(0);
    result_header(payload,VISION_CMD_QR,(uint8_t)(tx_seq+1),VISION_SUCCESS,0);
    deliver(VISION_CMD_RESULT,payload,15);step(true);
    assert(app_task_work_get_status()->request_pending && task_state==TASK_SCAN);
    result_header(payload,VISION_CMD_BOMB_SCAN,tx_seq,VISION_SUCCESS,0);payload[6]=1;
    deliver(VISION_CMD_RESULT,payload,15);step(true);
    assert(app_task_work_get_status()->request_pending && task_state==TASK_SCAN);
    acknowledge();step(true);assert(app_task_work_get_status()->phase==WORK_QR_OPEN_CLAMP);
    step(true);assert(task_state==TASK_MOVE2 && clamp_open_calls==1);

    pending_qr(0);
    /* Legacy XYZ packets are never used to move the chassis during QR. */
    result_header(payload,VISION_CMD_QR,tx_seq,VISION_SUCCESS,0);payload[4]=3;payload[7]=5;
    put_float(payload+15,100000);put_float(payload+19,-100000);put_float(payload+23,100000);
    put_float(payload+27,.5f);
    deliver(VISION_CMD_RESULT,payload,31);until_retry(WORK_ERR_DATA,20);
    assert(app_task_work_get_status()->corrections==0);
    assert(clamp_open_calls==0);
    until_phase(WORK_WAIT_LOCATE,1000);acknowledge();until_phase(WORK_COMPLETE,20);
}

static void retry_and_total_stop(void)
{
    pending_qr(UINT32_MAX-1000U);
    until_retry(WORK_ERR_TIMEOUT,66000);
    until_phase(WORK_WAIT_LOCATE,1000);acknowledge();until_phase(WORK_COMPLETE,20);

    pending_qr(0);
    uint8_t payload[64];result_header(payload,VISION_CMD_QR,tx_seq,VISION_FAILURE,19);
    deliver(VISION_CMD_RESULT,payload,15);until_retry(WORK_ERR_VISION,20);
    until_phase(WORK_WAIT_LOCATE,1000);acknowledge();until_phase(WORK_COMPLETE,20);

    pending_qr(0);command("TOTALSTOP!");acknowledge();step(true);
    assert(task_state==TASK_IDLE && !task_running && !app_task_work_get_status()->request_pending);
    assert(send_count[VISION_CMD_CANCEL]==0);
}

static void open_submission_retry_and_stop(uint32_t base)
{
    pending_qr(base);clamp_open_result=RET_BUSY;
    acknowledge();step(true);step(true);
    assert(app_task_work_get_status()->phase==WORK_QR_OPEN_CLAMP);
    assert(clamp_open_attempts==1 && clamp_open_calls==0 && task_state==TASK_SCAN);
    for(unsigned i=0;i<10;++i)step(true);
    assert(clamp_open_attempts==1);
    clamp_open_result=RET_OK;
    unsigned start=elapsed;
    while(task_state!=TASK_MOVE2 && elapsed-start<20U)step(true);
    assert(task_state==TASK_MOVE2 && clamp_open_attempts==2 && clamp_open_calls==1);
    assert(app_chassis_motion_get_state()==MOTION_STATE_RUNNING);

    pending_qr(base);clamp_open_result=RET_BUSY;
    acknowledge();step(true);step(true);
    command("TOTALSTOP!");clamp_open_result=RET_OK;step(true);
    for(unsigned i=0;i<30;++i)step(true);
    assert(task_state==TASK_IDLE && clamp_open_calls==0 && clamp_open_attempts==1);

    pending_qr(base);clamp_open_result=RET_ERROR;
    acknowledge();step(true);step(true);
    assert(task_state==TASK_FAULT && app_task_work_get_status()->error==WORK_ERR_ARM);
    assert(clamp_open_calls==0 && send_count[VISION_CMD_CANCEL]==0);

    pending_qr(base);clamp_open_result=RET_BUSY;
    acknowledge();step(true);
    unsigned began=elapsed;
    until_fault(1100);
    assert(elapsed-began<=1000U && clamp_open_calls==0);
    assert(app_task_work_get_status()->error==WORK_ERR_ARM);
}

int main(void)
{
    qr_success(0);qr_success(UINT32_MAX-1000U);
    strict_ack_and_stale_results();retry_and_total_stop();
    open_submission_retry_and_stop(0);open_submission_retry_and_stop(UINT32_MAX-1000U);
    puts("PASS: valid QR ACK queues clamp open once then immediately starts MOVE2; BUSY/error/timeout/STOP/rollover, zero correction/travel, strict payload and SEQ retained");
    return 0;
}
