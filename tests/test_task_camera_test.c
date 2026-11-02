/* Reuse hardware/scene fixture, but do not provide ANY vision replies. */
#define main normal_fixture_main
#define task_work_load_config fixture_load_config
#include "test_task_vision.c"
#undef task_work_load_config
#undef main

bool task_work_load_config(TaskWorkKind_t kind, TaskWorkConfig_t *out)
{
    if (!fixture_load_config(kind,out)) return false;
    for(unsigned i=0;i<3;++i) {
        out->slot[i].alignment.yaw_mode=VISION_YAW_NONE;
        out->slot[i].alignment.map_reference_valid=false;
    }
    return true;
}

static void camera_course(uint32_t base)
{
    reset(base);automatic_reply=false;
    command("TOTALSTART!");
    for(;elapsed<180000U && task_state!=TASK_END && task_state!=TASK_FAULT;) {
        step(true);
        const TaskWorkStatus_t *status=app_task_work_get_status();
        assert(status->corrections==0);
        if(status->selected_slot) assert(status->selected_slot==1U);
    }
    assert(task_state==TASK_END && pick_calls==2 && place_calls==1 && laser_starts==1 && !laser);
    const uint8_t commands[]={0x10,0x20,0x21,0x30,0x31,0x40,0x41};
    for(unsigned i=0;i<sizeof(commands);++i)assert(send_count[commands[i]]==1);
}
int main(void)
{
    assert(TASK_WORK_CAMERA_TEST==1);
    camera_course(0);camera_course(UINT32_MAX-5000U);
    reset(0);automatic_reply=false;enter_work(WORK_QR,false);
    until_phase(WORK_WAIT_LOCATE,4000);
    uint8_t payload[64];result_header(payload,tx_cmd,tx_seq,VISION_FAILURE,9);
    deliver(VISION_CMD_RESULT,payload,15); /* Broken camera failure must be ignored. */
    until_phase(WORK_COMPLETE,4000);
    assert(app_task_work_get_status()->corrections==0);
    reset(0);automatic_reply=false;enter_work(WORK_TARGET,false);
    until_phase(WORK_LASER,30000);assert(laser);
    command("TOTALSTOP!");step(true);
    assert(task_state==TASK_IDLE && !laser && !task_running);
    puts("PASS: camera test full course without vision replies, slot1, zero correction, stale failure ignored and real STOP");
    return 0;
}
