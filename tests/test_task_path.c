#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../Core/Src/task.c"

#define PI 3.14159265358979323846f
static uint32_t clock_base, elapsed_ms;
static bool imu_valid;
static float imu_yaw, out_vx, out_vy, out_omega, px, py;

uint32_t HAL_GetTick(void) { return clock_base + elapsed_ms; }
void app_vision_set_rx_cb(proto_vision_frame_cb_t cb) { proto_vision_set_cb(cb); }
bool app_imu_get_euler(float e[3]) { e[0]=e[1]=0;e[2]=imu_yaw;return imu_valid; }
int app_chassis_move(float x,float y,float w)
{
    assert(isfinite(x) && isfinite(y) && isfinite(w));
    out_vx=x;out_vy=y;out_omega=w;return 0;
}
static void reset(uint32_t base)
{
    clock_base=base;elapsed_ms=0;imu_valid=true;imu_yaw=PI/2;
    px=3350;py=1750;out_vx=out_vy=out_omega=0;
    app_chassis_motion_init();proto_vision_init();task_init();
}
static void payload(const void *text,uint8_t len)
{
    uint8_t frame[PROTO_VISION_OVERHEAD+PROTO_VISION_MAX_PAYLOAD];
    uint8_t n=proto_vision_pack(PROTO_VISION_CMD_STATUS,text,len,frame,(uint8_t)sizeof(frame));
    assert(n);proto_vision_feed(frame,n);
}
static void command(const char *text) { payload(text,(uint8_t)strlen(text)); }
static void tick(bool control)
{
    task_poll();
    if(control && elapsed_ms%10U==0U) {
        app_chassis_motion_update(.01f);app_chassis_closeloop_update();
        float mid=imu_yaw+.005f*out_omega;
        px+=(cosf(mid)*out_vx-sinf(mid)*out_vy)*.01f;
        py+=(sinf(mid)*out_vx+cosf(mid)*out_vy)*.01f;
        imu_yaw+=out_omega*.01f;
    }
    ++elapsed_ms;
}
static void test_course(uint32_t base)
{
    static const Task_State_t stops[]={TASK_SCAN,TASK_EXPLOSIVE,TASK_CS,TASK_RESCUE,TASK_END};
    static const float expected[5][2]={{2900,1750},{770,895},{770,2625},{1750,3200},{3350,3200}};
    reset(base);command("TOTALSTART!");
    unsigned stop_count=0,segment_seams=0;
    bool seen[TASK_FAULT+1]={false};
    Task_State_t last=TASK_IDLE;
    float last_command=0,max_command_accel=0;
    uint32_t done_ms=0;
    bool awaiting_stop=false;
    for(;elapsed_ms<90000U;) {
        /* Repeated start / unknown packets must not restart a running path. */
        if(elapsed_ms==6000U) { command("TOTALSTART!");command("UNKNOWN!"); }
        if(app_chassis_motion_get_result()==MOTION_RESULT_DONE && task_path_active && !awaiting_stop) {
            awaiting_stop=true;done_ms=elapsed_ms;
        }
        AlgPathSample_t before,after;
        bool had_path=app_chassis_motion_path_get_sample(&before);
        float previous_command=out_omega;
        bool control_tick=elapsed_ms%10U==0U;
        tick(true);
        if(control_tick) {
            max_command_accel=fmaxf(max_command_accel,fabsf(out_omega-previous_command)/.01f);
            last_command=out_omega;
        }
        (void)last_command;
        assert(task_get_state()!=TASK_FAULT);
        seen[task_get_state()]=true;
        if(had_path && app_chassis_motion_path_get_sample(&after) &&
           before.segment!=after.segment && after.segment>before.segment) {
            ++segment_seams;assert(before.speed>1 && after.speed>1);
        }
        if(task_get_state()!=last) {
            if(stop_count<5U && task_get_state()==stops[stop_count]) {
                assert(awaiting_stop && elapsed_ms-done_ms>=1000U);
                assert(app_chassis_motion_get_result()==MOTION_RESULT_DONE);
                assert(hypotf(out_vx,out_vy)<1e-5f);
                float error=hypotf(px-expected[stop_count][0],py-expected[stop_count][1]);
                printf("stop %u: state=%d model=(%.2f,%.2f), error=%.2f mm\n",stop_count,task_get_state(),px,py,error);
                assert(error<15);
                ++stop_count;awaiting_stop=false;
            }
            last=task_get_state();
        }
        if(task_get_state()==TASK_END) break;
    }
    assert(stop_count==5U && segment_seams==11U);
    for(unsigned n=0;n<16U;++n) assert(seen[task_move_state[n]]);
    assert(elapsed_ms<80500U);
    printf("PASS: %s course, elapsed=%lu ms, 11 moving seams, max ideal-loop command angular acceleration=%.3f rad/s^2\n",base?"clock rollover":"normal",(unsigned long)elapsed_ms,max_command_accel);
}
static void test_failures(void)
{
    reset(0);command("TOTALSTART!");tick(true);
    imu_valid=false;tick(true);
    while(elapsed_ms<20U) tick(true);
    assert(task_get_state()==TASK_FAULT);
    assert(app_chassis_motion_get_state()==MOTION_STATE_IDLE);
    reset(0);imu_valid=false;command("TOTALSTART!");tick(true);
    assert(task_get_state()==TASK_FAULT);
    reset(0);command("TOTALSTART!");tick(true);
    app_chassis_motion_stop();tick(true);
    assert(task_get_state()==TASK_FAULT);
    reset(UINT32_MAX-1000U);command("TOTALSTART!");tick(false);
    while(task_get_state()!=TASK_FAULT && elapsed_ms<6000U) tick(false);
    assert(task_get_state()==TASK_FAULT); /* stalled control must not claim success */
}
int main(void)
{
    test_course(0);test_course(UINT32_MAX-5000U);test_failures();
    puts("PASS: grouped task lifecycle, stops, continuity, rollover and failures");
    return 0;
}
