/* Real MCU task/work/UART/motion with stationary, physical-slot scan replies. */
#define main legacy_task_fixture_main
#include "test_task_vision.c"
#undef main

static uint8_t scan_cmd(TaskWorkKind_t kind)
{
    return kind==WORK_BOMB ? VISION_CMD_BOMB_SCAN :
           kind==WORK_TARGET ? VISION_CMD_TARGET_SCAN : VISION_CMD_RESCUE_SCAN;
}

static uint8_t locate_cmd(TaskWorkKind_t kind)
{
    return kind==WORK_BOMB ? VISION_CMD_BOMB_LOCATE :
           kind==WORK_TARGET ? VISION_CMD_TARGET_LOCATE : VISION_CMD_RESCUE_LOCATE;
}

static void target_at_slot(TaskWorkKind_t kind,uint8_t slot,uint32_t base,bool errors)
{
    reset(base);scene_slot=slot;enter_work(kind,errors);
    float start_x=px,start_y=py,start_yaw=yaw;
    unsigned scan_count=0;
    bool located=false;
    while(app_task_work_get_status()->phase!=WORK_COMPLETE && elapsed<60000U && task_state!=TASK_FAULT) {
        step(true);
        const TaskWorkStatus_t *status=app_task_work_get_status();
        if(send_count[scan_cmd(kind)]!=scan_count) {
            ++scan_count;
            assert(scan_count==tx_slot && tx_slot<=slot);
            assert(status->phase==WORK_WAIT_SCAN);
            assert(fabsf(yaw-start_yaw)<.0001f);
        }
        if(send_count[locate_cmd(kind)]) {
            located=true;
            assert(status->selected_slot==slot && scan_count==slot);
        }
    }
    if(app_task_work_get_status()->phase!=WORK_COMPLETE)
        printf("stop scan kind=%d slot=%u errors=%d phase=%d error=%d time=%u\n",kind,slot,errors,
            app_task_work_get_status()->phase,app_task_work_get_status()->error,elapsed);
    assert(app_task_work_get_status()->phase==WORK_COMPLETE && located);
    assert(scan_count==slot && send_count[scan_cmd(kind)]==slot);
    assert(hypotf(px-start_x,py-start_y)<3 && fabsf(yaw-start_yaw)<.018f);
    if(kind==WORK_TARGET)assert(laser_starts==1 && pick_calls==0 && place_calls==0 && !laser);
    else if(kind==WORK_RESCUE)assert(pick_calls==1 && place_calls==0);
    else {
        assert(pick_calls==1 && place_calls==1);
        /* The bomb still travels to the fixed bucket, even when slot 1 matched. */
        assert(hypotf(placed_x-start_x,placed_y-start_y)>40);
    }
    step(true);
    assert(task_state==(kind==WORK_BOMB ? TASK_MOVE12 : kind==WORK_TARGET ? TASK_MOVE13 : TASK_MOVE16));
}

static void scan_failures(void)
{
    /* Silence at a stop does not move the chassis into another observation. */
    reset(0);automatic_reply=false;enter_work(WORK_BOMB,false);
    until_phase(WORK_WAIT_SCAN,4000);
    float x=px,y=py;
    for(unsigned i=0;i<3000;++i)step(true);
    assert(send_count[VISION_CMD_BOMB_SCAN]==1 && hypotf(px-x,py-y)<.01f);
    uint8_t p[64];result_header(p,tx_cmd,tx_seq,VISION_FAILURE,VISION_REASON_SCAN_NOT_TARGET);
    p[6]=1;deliver(VISION_CMD_RESULT,p,15);
    until_phase(WORK_WAIT_MOTION,100);
    uint8_t old_seq=tx_seq;
    until_phase(WORK_WAIT_SCAN,5000);
    result_header(p,VISION_CMD_BOMB_SCAN,old_seq,VISION_SUCCESS,0);p[6]=1;
    deliver(VISION_CMD_RESULT,p,15);step(true);
    assert(app_task_work_get_status()->request_pending && pick_calls==0);
    /* A same-SEQ hit for a different physical stop is rejected. */
    result_header(p,tx_cmd,tx_seq,VISION_SUCCESS,0);p[6]=3;
    deliver(VISION_CMD_RESULT,p,15);until_fault(20);
    assert(app_task_work_get_status()->error==WORK_ERR_DATA);

    reset(0);scene_slot=0;enter_work(WORK_RESCUE,false);
    until_fault(20000);
    assert(send_count[VISION_CMD_RESCUE_SCAN]==3 && pick_calls==0);
    assert(app_task_work_get_status()->vision_reason==VISION_REASON_SCAN_NOT_TARGET);
    /* A real model error is never treated as "try the next position". */
    reset(0);automatic_reply=false;enter_work(WORK_RESCUE,false);
    until_phase(WORK_WAIT_SCAN,4000);
    result_header(p,tx_cmd,tx_seq,VISION_FAILURE,9);p[6]=1;
    deliver(VISION_CMD_RESULT,p,15);until_fault(20);
    assert(send_count[VISION_CMD_RESCUE_SCAN]==1 && pick_calls==0);
    assert(app_task_work_get_status()->vision_reason==9);

    reset(UINT32_MAX-1000U);automatic_reply=false;enter_work(WORK_TARGET,false);
    until_phase(WORK_WAIT_SCAN,4000);command("TOTALSTOP!");step(true);
    assert(task_state==TASK_IDLE && !task_running && !laser);
    reset(0);automatic_reply=false;enter_work(WORK_TARGET,false);
    until_phase(WORK_WAIT_SCAN,4000);yaw+=.05f;until_fault(20);
    assert(app_task_work_get_status()->error==WORK_ERR_OBSERVATION_MOVED);
    reset(0);automatic_reply=false;enter_work(WORK_BOMB,false);
    until_phase(WORK_WAIT_SCAN,4000);until_fault(64000);
    assert(app_task_work_get_status()->error==WORK_ERR_TIMEOUT && send_count[VISION_CMD_BOMB_SCAN]==1);
}

int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    for(unsigned kind=WORK_BOMB;kind<WORK_COUNT;++kind)
        for(uint8_t slot=1;slot<=3;++slot)
            for(unsigned rollover=0;rollover<2;++rollover) {
                target_at_slot((TaskWorkKind_t)kind,slot,rollover?UINT32_MAX-1000U:0,false);
                target_at_slot((TaskWorkKind_t)kind,slot,rollover?UINT32_MAX-1000U:0,true);
            }
    scan_failures();
    puts("PASS: stopped scan hits at slots 1/2/3, early exit, no moving scans, bucket/rescue/laser, correction undo and origin/yaw return, stale/mismatched replies, missing targets, failures, STOP and rollover");
    return 0;
}
