/* Actual work/vision/task/motion handoff, including the complete next MOVE group. */
#define main historical_vision_fixture_main
#include "test_task_vision.c"
#undef main

static Task_State_t first_move(TaskWorkKind_t kind)
{
    return kind==WORK_BOMB ? TASK_MOVE12 : kind==WORK_TARGET ? TASK_MOVE13 : TASK_MOVE16;
}

static void handoff_and_route(TaskWorkKind_t kind,uint8_t slot,bool errors,uint32_t base)
{
    reset(base);scene_slot=slot;enter_work(kind,errors);
    until_phase(WORK_RETURN,40000);
    float exit_x=px,exit_y=py;
    assert(!app_task_work_get_status()->request_pending && !laser);
    until_phase(WORK_COMPLETE,5000);
    assert(hypotf(px-exit_x,py-exit_y)<.1f); /* No translation back to scan start. */
    float exit_yaw=yaw;
    const float deduction=kind==WORK_BOMB ? 150.0f : (slot-1U)*150.0f;
    const TaskWorkStatus_t *status=app_task_work_get_status();
    assert(status->next_move_ready && fabsf(status->next_move_deduction_mm-deduction)<.001f);
    assert(!status->request_pending && !status->alignment_valid);
    assert(clamp_open_calls==0); /* Async empty-claw release belongs only to QR. */
    float untouched=-1;
    assert(!app_task_work_take_move_deduction(WORK_QR,&untouched) && untouched==-1);
    assert(!app_task_work_take_move_deduction(kind,NULL));
    assert(!app_task_work_take_move_deduction(kind==WORK_BOMB?WORK_TARGET:WORK_BOMB,&untouched));

    unsigned commands[256];memcpy(commands,send_count,sizeof(commands));
    uint8_t stale[64];memcpy(stale,last_payload,sizeof(stale));
    uint8_t stale_len=last_payload_len;
    float original_first=task_route[kind==WORK_BOMB?11:kind==WORK_TARGET?12:15].distance;
    step(true);
    assert(task_state==first_move(kind) && app_chassis_motion_get_state()==MOTION_STATE_RUNNING);
    assert(!app_task_work_get_status()->next_move_ready);
    assert(!app_task_work_take_move_deduction(kind,&untouched));

    /* Late success/failure and a duplicate START cannot reenter the old vision task. */
    deliver(VISION_CMD_RESULT,stale,stale_len);
    stale[3]=VISION_FAILURE;stale[14]=9;
    deliver(VISION_CMD_RESULT,stale,stale_len);
    command("TOTALSTART!");
    AlgPathSample_t before={0},after={0};
    bool path=app_chassis_motion_path_get_sample(&before);
    unsigned seams=0,start=elapsed;
    while(app_chassis_motion_get_result()==MOTION_RESULT_RUNNING && elapsed-start<20000U) {
        step(true);
        assert(task_state!=TASK_FAULT && task_running);
        assert(!app_task_work_get_status()->request_pending);
        assert(memcmp(commands,send_count,sizeof(commands))==0);
        if(path) {
            assert(app_chassis_motion_path_get_sample(&after));
            if(after.segment!=before.segment) {
                ++seams;assert(before.speed>1 && after.speed>1);
            }
            before=after;
        }
    }
    assert(app_chassis_motion_get_result()==MOTION_RESULT_DONE);
    assert(seams==(kind==WORK_TARGET?2U:0U));
    float right,forward;
    if(kind==WORK_BOMB) {
        right=TASK_MOVE12_RIGHT_MM;
        forward=original_first-deduction;
    } else if(kind==WORK_TARGET) {
        right=200.0f+task_route[14].distance;
        forward=original_first-deduction+200.0f-task_route[14].right_offset_mm;
    } else {
        right=task_route[15].right_offset_mm;
        forward=original_first-deduction;
    }
    float dx=px-exit_x,dy=py-exit_y;
    assert(fabsf(cosf(exit_yaw)*dx+sinf(exit_yaw)*dy-right)<3);
    assert(fabsf(-sinf(exit_yaw)*dx+cosf(exit_yaw)*dy-forward)<3);
    if(!errors) {
        /* Original route endpoint must be independent of which scan stop matched. */
        float from_start_x=px-anchor[kind][0],from_start_y=py-anchor[kind][1];
        float original_forward=original_first+(kind==WORK_TARGET ?
            200.0f-task_route[14].right_offset_mm : 0.0f);
        float original_yaw=anchor[kind][2];
        assert(fabsf(cosf(original_yaw)*from_start_x+sinf(original_yaw)*from_start_y-right)<3);
        assert(fabsf(-sinf(original_yaw)*from_start_x+cosf(original_yaw)*from_start_y-original_forward)<3);
    }
    assert(task_route[kind==WORK_BOMB?11:kind==WORK_TARGET?12:15].distance==original_first);

    uint32_t done=elapsed;
    Task_State_t next=kind==WORK_BOMB?TASK_CS:kind==WORK_TARGET?TASK_RESCUE:TASK_END;
    while(task_state!=next && elapsed-done<1100U) {
        step(true);
        assert(memcmp(commands,send_count,sizeof(commands))==0);
    }
    assert(task_state==next && elapsed-done>=1000U);
    if(kind!=WORK_RESCUE) {
        /* The following work starts at slot 1 with a fresh request, while old replies stay invalid. */
        scene_slot=1;
        until_phase(WORK_WAIT_SCAN,2000);
        const TaskWorkStatus_t *fresh=app_task_work_get_status();
        assert(fresh->kind==(kind==WORK_BOMB?WORK_TARGET:WORK_RESCUE));
        assert(fresh->request_pending && tx_slot==1 && !fresh->next_move_ready);
        deliver(VISION_CMD_RESULT,stale,stale_len);
        assert(app_task_work_get_status()->request_pending);
    } else {
        step(true);assert(!task_running);
    }
    assert(send_count[VISION_CMD_CANCEL]==0);
}

static void waits_for_arm(TaskWorkKind_t kind,uint32_t base)
{
    reset(base);scene_slot=2;enter_work(kind,false);
    until_phase(WORK_TRAVEL_ARM,30000);
    arm_wait=true;
    float x=px,y=py,out=-1;
    unsigned start=elapsed;
    while(elapsed-start<1000U) {
        step(true);
        assert(app_task_work_get_status()->phase==WORK_TRAVEL_ARM);
        assert(!app_task_work_take_move_deduction(kind,&out));
        assert(app_chassis_motion_get_state()==MOTION_STATE_IDLE);
        assert(hypotf(px-x,py-y)<.01f && !laser);
    }
    arm_wait=false;
    until_phase(WORK_COMPLETE,1000);
    step(true);assert(task_state==first_move(kind));
}

static void stop_and_single_use(uint32_t base)
{
    reset(base);scene_slot=3;enter_work(WORK_TARGET,false);
    float out=-1;
    assert(!app_task_work_take_move_deduction(WORK_TARGET,&out));
    until_phase(WORK_COMPLETE,30000);
    assert(app_task_work_take_move_deduction(WORK_TARGET,&out) && out==300);
    assert(!app_task_work_take_move_deduction(WORK_TARGET,&out));

    reset(base);scene_slot=3;enter_work(WORK_TARGET,false);
    until_phase(WORK_COMPLETE,30000);
    assert(app_task_work_get_status()->next_move_ready);
    command("TOTALSTOP!");
    assert(!app_task_work_take_move_deduction(WORK_TARGET,&out));
    step(true);assert(task_state==TASK_IDLE && !app_task_work_get_status()->next_move_ready);
    command("TOTALSTART!");step(true);
    assert(task_state==TASK_MOVE1 && task_first_segment==0 && task_running);
    assert(!app_task_work_get_status()->next_move_ready);

    reset(base);scene_slot=3;enter_work(WORK_RESCUE,false);
    until_phase(WORK_COMPLETE,30000);step(true);
    assert(task_state==TASK_MOVE16);
    command("TOTALSTOP!");step(true);
    assert(task_state==TASK_IDLE && app_chassis_motion_get_state()==MOTION_STATE_IDLE);
    assert(!app_task_work_take_move_deduction(WORK_RESCUE,&out));
}

static void reject_excess_deduction(void)
{
    const uint8_t first[]={12,13,16},count[]={1,3,1};
    for(unsigned i=0;i<3;++i) {
        for(unsigned extra=0;extra<2;++extra) {
            reset(0);task_running=true;
            task_start_path(first[i],count[i],HAL_GetTick(),task_route[first[i]-1U].distance+extra);
            assert(task_state==TASK_FAULT && !task_path_active);
            assert(app_chassis_motion_get_state()==MOTION_STATE_IDLE);
        }
    }
}

int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    for(unsigned kind=WORK_BOMB;kind<WORK_COUNT;++kind) {
        for(uint8_t slot=1;slot<=3;++slot)
            for(unsigned rollover=0;rollover<2;++rollover)
                for(unsigned errors=0;errors<2;++errors)
                    handoff_and_route((TaskWorkKind_t)kind,slot,errors!=0,
                        rollover?UINT32_MAX-1000U:0);
        waits_for_arm((TaskWorkKind_t)kind,0);
        waits_for_arm((TaskWorkKind_t)kind,UINT32_MAX-1000U);
    }
    stop_and_single_use(0);stop_and_single_use(UINT32_MAX-1000U);
    reject_excess_deduction();
    puts("PASS: 36 real work-to-next-MOVE handoffs, completed scan travel deducted once, preserved corrections/route parameters/offsets, continuous seams, late vision rejection, fresh next scan, arm gate, STOP/restart and rollover");
    return 0;
}
