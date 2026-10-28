#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../Core/Src/task.c"

static uint32_t test_tick;
uint32_t HAL_GetTick(void) { return test_tick; }
void app_vision_set_rx_cb(proto_vision_frame_cb_t cb) { proto_vision_set_cb(cb); }
bool app_imu_get_euler(float e[3]) { e[0]=e[1]=e[2]=0; return true; }
int app_chassis_move(float vx,float vy,float omega) { (void)vx;(void)vy;(void)omega;return 0; }
static void reset(void)
{
    test_tick=100;
    app_chassis_motion_init();proto_vision_init();task_init();
}
static void receive_payload(const void *payload,uint8_t len)
{
    uint8_t bytes[PROTO_VISION_OVERHEAD+PROTO_VISION_MAX_PAYLOAD];
    uint8_t n=proto_vision_pack(PROTO_VISION_CMD_STATUS,payload,len,bytes,(uint8_t)sizeof(bytes));
    assert(n);proto_vision_feed(bytes,n);
}
static void text(const char *s) { receive_payload(s,(uint8_t)strlen(s)); }
static void test_start_events(void)
{
    static const char *const invalid[]={"","TOTALSTART","TOTALSTART!!","totalstart!","TOTALSTART!\r\n"};
    reset();
    for(unsigned n=0;n<sizeof(invalid)/sizeof(invalid[0]);++n) {
        text(invalid[n]);task_poll();assert(task_get_state()==TASK_IDLE);
    }
    receive_payload("TOTALSTART!",(uint8_t)sizeof("TOTALSTART!"));
    receive_payload("TOTAL\0TART!",11);
    proto_vision_feed((const uint8_t *)"TOTALSTART!",11);
    task_poll();assert(task_get_state()==TASK_IDLE);
    test_tick+=10000;task_poll();assert(task_get_state()==TASK_IDLE);
    text("TOTALSTART!");text("UNKNOWN!");
    assert(task_total_start_requested);
    task_poll();assert(task_get_state()==TASK_MOVE1);
    assert(app_chassis_motion_get_result()==MOTION_RESULT_RUNNING);
    AlgPathSample_t before,after;
    assert(app_chassis_motion_path_get_sample(&before));
    test_tick+=500;text("TOTALSTART!");task_poll();
    assert(task_get_state()==TASK_MOVE1 && !task_total_start_requested);
    assert(app_chassis_motion_path_get_sample(&after));
    assert(before.duration==after.duration && after.elapsed==0);
    reset();text("TOTALSTART!");text("TOTALSTOP!");task_poll();
    assert(task_get_state()==TASK_IDLE && !task_total_start_requested);
}
static void test_complete_frames(void)
{
    uint8_t bytes[PROTO_VISION_OVERHEAD+PROTO_VISION_MAX_PAYLOAD];
    reset();
    uint8_t n=proto_vision_pack(PROTO_VISION_CMD_STATUS,(const uint8_t *)"TOTALSTART!",11,bytes,(uint8_t)sizeof(bytes));
    assert(n);proto_vision_feed(bytes,n-1U);task_poll();assert(task_get_state()==TASK_IDLE);
    bytes[n-1U]^=1;proto_vision_feed(&bytes[n-1U],1);task_poll();
    assert(task_get_state()==TASK_IDLE && proto_vision_get_stat()->crc_errors==1);
    bytes[n-1U]^=1;proto_vision_feed(bytes,3);proto_vision_feed(&bytes[3],n-3U);
    assert(task_get_state()==TASK_IDLE && task_total_start_requested);
    task_poll();assert(task_get_state()==TASK_MOVE1);
    task_vision_on_frame(NULL);
    proto_vision_frame_t bad={0};bad.len=PROTO_VISION_MAX_PAYLOAD+1U;
    task_vision_on_frame(&bad);
}
static void test_offset_validation(void)
{
    static const char *const bad[]={
        "OFFSET,X=","OFFSET,X=1","OFFSET,X=,Y=2","OFFSET,X=1,Y=",
        "OFFSET,X=1,Y=bad","OFFSET,X=bad,Y=2","OFFSET,X=1,Z=2",
        "OFFSET,X=1,Y=2junk","OFFSET,X=1,Y=2\r\n","OFFSET,X=1, Y=2",
        "OFFSET,X=1e999,Y=2","OFFSET,X=1,Y=-1e999",
        "OFFSET,X=1e-999,Y=2","OFFSET,X=1,Y=1e-999",
        "OFFSET,X=nan,Y=2","OFFSET,X=1,Y=nan","OFFSET,X=inf,Y=2","OFFSET,X=1,Y=-inf"
    };
    reset();text("OFFSET,X=12.5,Y=-3.25");
    assert(task_offset_pending && task_offset_x==12.5f && task_offset_y==-3.25f);
    task_offset_pending=false;
    for(unsigned n=0;n<sizeof(bad)/sizeof(bad[0]);++n) {
        text(bad[n]);assert(!task_offset_pending);
        assert(task_offset_x==12.5f && task_offset_y==-3.25f);
    }
    receive_payload("OFFSET,X=1,Y=2\0junk",(uint8_t)(sizeof("OFFSET,X=1,Y=2\0junk")-1U));
    receive_payload("OFFSET,X=1,Y=2",(uint8_t)sizeof("OFFSET,X=1,Y=2"));
    assert(!task_offset_pending);
    text("OFFSET,X=-1.25e2,Y=2.5e-1");text("UNKNOWN!");text("OFFSET,X=5,Y=bad");
    assert(task_offset_pending && task_offset_x==-125 && task_offset_y==.25f);
    task_poll();assert(task_get_state()==TASK_IDLE);
    /* Coordinates are stored for the future work logic, not acted upon yet. */
    assert(task_offset_pending);
    uint8_t full[PROTO_VISION_MAX_PAYLOAD];memset(full,'0',sizeof(full));
    memcpy(full,"OFFSET,X=0,Y=",sizeof("OFFSET,X=0,Y=")-1U);full[sizeof(full)-1U]='1';
    receive_payload(full,(uint8_t)sizeof(full));
    assert(task_offset_x==0 && task_offset_y==1);
}
int main(void)
{
    test_start_events();test_complete_frames();test_offset_validation();
    puts("PASS: exact vision commands, framing, CRC, start latching, duplicate starts and offset validation");
    return 0;
}