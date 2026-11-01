/* Real task/work, app UART, BSP/ring buffer, protocol, transform and motion.
 * Only UART hardware, IMU/motors, arm hooks and physical scene are simulated.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../Core/Src/task.c"
#include "bsp_uart.h"

#define PI 3.14159265358979323846f
static UART_HandleTypeDef vision_uart;
static uint8_t *rx_byte;
static uint32_t elapsed, clock_base;
static float px,py,yaw,vx,vy,omega;
static bool imu_ok, config_ready, automatic_reply, laser;
static bool nonconverging, arm_wait;
static unsigned laser_starts, arm_stops, pick_calls, place_calls;
static uint8_t scene_slot = 2U;
static float picked_x, picked_y, placed_x, placed_y;
static HAL_StatusTypeDef force_tx;
static uint8_t tx_cmd,tx_seq,peer_seq;
static uint32_t tx_time;
static bool reply_pending;
static unsigned send_count[256];
static uint8_t last_payload[64],last_payload_len;
static const float anchor[4][3]={{2900,1750,PI/2},{770,895,0},{770,2625,0},{1750,3200,-PI/2}};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
uint32_t HAL_GetTick(void) { return clock_base+elapsed; }
void HAL_Delay(uint32_t ms) { (void)ms; assert(!"blocking delay in work loop"); }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n,uint32_t t)
{ (void)u;(void)p;(void)n;(void)t;assert(!"blocking UART in work loop");return HAL_ERROR; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,const uint8_t *p,uint16_t n)
{
    if (force_tx!=HAL_OK) return force_tx;
    assert(u==&vision_uart && n>=6 && p[0]==0xaa && p[1]==0x55);
    assert(p[2]==0 && n==6 && proto_vision_crc8(p+2,n-3)==p[n-1]);
    u->test_tx_ptr=p;u->test_tx_len=n;u->gState=HAL_UART_STATE_BUSY_TX;
    tx_cmd=p[4];tx_seq=p[3];tx_time=elapsed;++send_count[tx_cmd];
    reply_pending=tx_cmd!=VISION_CMD_CANCEL;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ assert(u==&vision_uart && n==1);rx_byte=p;return HAL_OK; }
bool app_imu_get_euler(float e[3]) { e[0]=e[1]=0;e[2]=yaw;return imu_ok; }
int app_chassis_move(float x,float y,float w)
{ assert(isfinite(x)&&isfinite(y)&&isfinite(w));vx=x;vy=y;omega=w;return 0; }
void task_work_laser_set(bool on) { if(on&&!laser) ++laser_starts;laser=on; }
int task_work_arm_stop_it(void) { ++arm_stops;return RET_OK; }
TaskArmResult_t task_work_arm_poll(TaskWorkKind_t k,TaskArmStep_t step,uint8_t slot,bool entering)
{
    (void)k;(void)slot;
    if (arm_wait && step != ARM_CANCEL) return ARM_WAITING;
    if(entering && step==ARM_PICK) { ++pick_calls; picked_x=px; picked_y=py; }
    if(entering && step==ARM_PLACE) {
        assert(k==WORK_BOMB); /* Rescue must never release the hostage. */
        ++place_calls; placed_x=px; placed_y=py;
    }
    return ARM_DONE; /* Test fixture only, not real firmware. */
}
bool task_work_load_config(TaskWorkKind_t kind,TaskWorkConfig_t *c)
{
    if(!config_ready) return false;
    *c=(TaskWorkConfig_t){.settle_ms=250,.scan_dwell_ms=300,.arm_timeout_ms=3000,.laser_ms=1000,
        .scan_speed=100,.correction_speed=60,.accel=120,.max_correction_mm=100,
        .max_correction_yaw=.174532925f,.position_tolerance_mm=3,.yaw_tolerance_rad=.008726646f,
        .height_tolerance_mm=10,.max_corrections=4};
    for(unsigned i=0;i<3;++i) {
        float x=kind==WORK_QR?0:80.0f*i;
        c->slot[i].offset_mm[0]=x;
        c->slot[i].observe_pose=(End_Effector_Pos_t){50,100,100,-.15};
        VisionAlignmentConfig_t *a=&c->slot[i].alignment;
        a->desired_reference_body[1]=300;a->desired_reference_body[2]=100;
        for(unsigned j=0;j<3;++j) {a->min_reference_body[j]=-1000;a->max_reference_body[j]=1000;}
        a->map_reference_valid=true;
        float cs=cosf(anchor[kind][2]),sn=sinf(anchor[kind][2]);
        a->reference_map[0]=anchor[kind][0]+cs*x-sn*300;
        a->reference_map[1]=anchor[kind][1]+sn*x+cs*300;
        a->reference_map[2]=100;
        a->desired_map_yaw=anchor[kind][2];
        a->desired_direction_body[1]=1;a->qr_axis[1]=1;
        a->yaw_mode=kind==WORK_QR?VISION_YAW_QR_AXIS:
                    kind==WORK_TARGET?VISION_YAW_TARGET_NORMAL:VISION_YAW_NONE;
    }
    return true;
}
static void receive_bytes(const uint8_t *bytes,unsigned n)
{
    for(unsigned i=0;i<n;++i) {*rx_byte=bytes[i];HAL_UART_RxCpltCallback(&vision_uart);}
}
static unsigned packet(uint8_t cmd,const uint8_t *p,uint8_t n,uint8_t *out)
{
    out[0]=0xaa;out[1]=0x55;out[2]=n;out[3]=peer_seq++;out[4]=cmd;
    if(n)memcpy(out+5,p,n);
    out[n+5]=proto_vision_crc8(out+2,n+3);return n+6;
}
static void deliver(uint8_t cmd,const uint8_t *p,uint8_t n)
{
    uint8_t bytes[70];unsigned len=packet(cmd,p,n,bytes);
    receive_bytes(bytes,4);app_vision_poll();receive_bytes(bytes+4,len-4);app_vision_poll();
}
static void command(const char *text) {deliver(PROTO_VISION_CMD_STATUS,(const uint8_t *)text,(uint8_t)strlen(text));}
static void put_float(uint8_t *p,float v)
{
    uint32_t b;memcpy(&b,&v,4);for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(b>>(i*8));
}
static void result_header(uint8_t *p,uint8_t cmd,uint8_t seq,uint8_t status,uint8_t reason)
{
    memset(p,0,64);p[0]=1;p[1]=cmd;p[2]=seq;p[3]=status;p[14]=reason;
}
static uint8_t make_scene_result(uint8_t *p)
{
    const TaskWorkStatus_t *s=app_task_work_get_status();
    result_header(p,tx_cmd,tx_seq,0,0);
    if(proto_vision_is_scan(tx_cmd)) {p[6]=scene_slot;return 15;}
    TaskWorkConfig_t cfg;assert(task_work_load_config(s->kind,&cfg));
    unsigned slot=s->kind==WORK_QR?0:scene_slot-1U;
    VisionAlignmentConfig_t *c=&cfg.slot[slot].alignment;
    VisionTransform_t t;assert(alg_vision_transform_from_arm_pose(&cfg.slot[slot].observe_pose,&t));
    float mx=c->reference_map[0]-px,my=c->reference_map[1]-py;
    float b[3]={cosf(yaw)*mx+sinf(yaw)*my,-sinf(yaw)*mx+cosf(yaw)*my,100};
    if(nonconverging) { b[0]=20;b[1]=300; }
    float e[3]={0};
    for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)e[i]+=t.rotation[j][i]*(b[j]-t.translation[j]);
    p[4]=3;p[6]=s->kind==WORK_QR?0:scene_slot;p[7]=5;
    unsigned off=15;for(unsigned i=0;i<3;++i){put_float(p+off,e[i]);off+=4;}
    float d=anchor[s->kind][2]-yaw;
    if(s->kind==WORK_QR) {
        p[4]|=VISION_QUATERNION;
        float rz[3][3]={{cosf(d),-sinf(d),0},{sinf(d),cosf(d),0},{0,0,1}},r[3][3]={{0}};
        for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)for(unsigned k=0;k<3;++k)
            r[i][j]+=t.rotation[k][i]*rz[k][j];
        float scale=sqrtf(1+r[0][0]+r[1][1]+r[2][2])*2;
        float q[4]={.25f*scale,(r[2][1]-r[1][2])/scale,(r[0][2]-r[2][0])/scale,(r[1][0]-r[0][1])/scale};
        for(unsigned i=0;i<4;++i){put_float(p+off,q[i]);off+=4;}
    } else if(s->kind==WORK_TARGET) {
        p[4]|=VISION_NORMAL;
        float n[3]={-sinf(d),cosf(d),0};
        for(unsigned i=0;i<3;++i) {
            float value=0;for(unsigned j=0;j<3;++j)value+=t.rotation[j][i]*n[j];
            put_float(p+off,value);off+=4;
        }
    }
    put_float(p+off,.5f);return (uint8_t)(off+4);
}
static void reset(uint32_t base)
{
    elapsed=0;clock_base=base;px=3350;py=1750;yaw=PI/2;vx=vy=omega=0;
    imu_ok=config_ready=automatic_reply=true;laser=false;laser_starts=arm_stops=pick_calls=place_calls=0;
    nonconverging=arm_wait=false;
    scene_slot=2U;
    force_tx=HAL_OK;reply_pending=false;memset(send_count,0,sizeof(send_count));peer_seq=0;
    vision_uart=(UART_HandleTypeDef){.gState=HAL_UART_STATE_READY};
    assert(app_vision_init(&vision_uart)==RET_OK);app_chassis_motion_init();task_init();
}
static void enter_work(TaskWorkKind_t kind,bool errors)
{
    static const Task_State_t states[]={TASK_SCAN,TASK_EXPLOSIVE,TASK_CS,TASK_RESCUE};
    task_state=states[kind];task_running=true;
    px=anchor[kind][0]+(errors?20:0);py=anchor[kind][1]-(errors?15:0);
    yaw=anchor[kind][2]+((errors&&(kind==WORK_QR||kind==WORK_TARGET))?.06f:0);
    app_chassis_motion_init();
}
static void step(bool control)
{
    if(vision_uart.gState==HAL_UART_STATE_BUSY_TX && elapsed>tx_time)
        vision_uart.gState=HAL_UART_STATE_READY;
    const TaskWorkStatus_t *s=app_task_work_get_status();
    if(automatic_reply&&reply_pending&&elapsed-tx_time>=300U&&
       (!proto_vision_is_scan(tx_cmd)||s->phase==WORK_WAIT_SCAN)) {
        last_payload_len=make_scene_result(last_payload);
        deliver(VISION_CMD_RESULT,last_payload,last_payload_len);
        deliver(VISION_CMD_RESULT,last_payload,last_payload_len); /* duplicate terminal */
        reply_pending=false;
    }
    app_vision_poll();task_poll();
    if(control&&elapsed%10U==0U) {
        app_chassis_motion_update(.01f);app_chassis_closeloop_update();
        float mid=yaw+.005f*omega;
        px+=(cosf(mid)*vx-sinf(mid)*vy)*.01f;
        py+=(sinf(mid)*vx+cosf(mid)*vy)*.01f;yaw+=omega*.01f;
    }
    ++elapsed;
}
static void until_phase(TaskWorkPhase_t p,unsigned limit)
{
    unsigned start=elapsed;
    while(app_task_work_get_status()->phase!=p && elapsed-start<limit && task_state!=TASK_FAULT)step(true);
    if(app_task_work_get_status()->phase!=p)
        printf("phase wanted %d got %d err %d time %u\n",p,app_task_work_get_status()->phase,app_task_work_get_status()->error,elapsed);
    assert(app_task_work_get_status()->phase==p);
}
static void until_fault(unsigned limit)
{
    unsigned start=elapsed;while(task_state!=TASK_FAULT&&elapsed-start<limit)step(true);
    assert(task_state==TASK_FAULT && !laser && app_chassis_motion_get_state()==MOTION_STATE_IDLE);
}
static void test_seq_and_events(void)
{
    reset(0);automatic_reply=false;
    uint8_t seq=199;force_tx=HAL_ERROR;
    assert(app_vision_send_with_seq(0x10,NULL,0,&seq)==RET_ERROR && seq==199);
    force_tx=HAL_OK;
    assert(app_vision_send_with_seq(0x10,NULL,0,&seq)==RET_OK && seq==vision_uart.test_tx_ptr[3]);
    uint8_t first=seq;
    assert(app_vision_send_with_seq(0x20,NULL,0,&seq)==RET_BUSY && seq==first);
    assert(vision_uart.test_tx_ptr[4]==0x10);
    vision_uart.gState=HAL_UART_STATE_READY;
    assert(app_vision_send_with_seq(0x20,NULL,0,&seq)==RET_OK && seq!=(uint8_t)(first+1));
    for(unsigned i=0;i<260;++i) {
        vision_uart.gState=HAL_UART_STATE_READY;
        assert(app_vision_send_with_seq(0x10,NULL,0,&seq)==RET_OK && seq==vision_uart.test_tx_ptr[3]);
    }
    reset(0);automatic_reply=false;
    const uint8_t start[]="TOTALSTART!";
    deliver(0x81,start,11);task_poll();assert(task_state==TASK_IDLE);
    deliver(0x02,start,12);task_poll();assert(task_state==TASK_IDLE);
    command("TOTALSTART!");command("UNKNOWN!");task_poll();assert(task_state==TASK_MOVE1);
    command("TOTALSTART!");task_poll();assert(task_state==TASK_MOVE1);
    command("TOTALSTOP!");task_poll();assert(task_state==TASK_IDLE && !task_running);
    reset(0);automatic_reply=false;enter_work(WORK_QR,true);
    until_phase(WORK_WAIT_LOCATE,4000);assert(app_task_work_get_status()->request_pending);
    uint8_t p[64];uint8_t n=make_scene_result(p);uint8_t seq_saved=p[2];
    p[2]=(uint8_t)(seq_saved+1);deliver(0x81,p,n);step(true);
    assert(app_task_work_get_status()->request_pending && app_chassis_motion_get_state()==MOTION_STATE_IDLE);
    p[2]=seq_saved;p[1]=0x31;deliver(0x81,p,n);step(true);
    assert(app_task_work_get_status()->request_pending);
    p[1]=0x10;p[3]=VISION_OBSERVATION;p[4]=(p[4]&~VISION_CONTROL_READY)|VISION_CANDIDATE;
    deliver(0x81,p,n);step(true);assert(app_task_work_get_status()->request_pending);
    n=make_scene_result(p);deliver(0x81,p,n);
    put_float(p+15,99999);deliver(0x81,p,n); /* Duplicate cannot overwrite copied terminal. */
    assert(!app_task_work_get_status()->request_pending);
    step(true);step(true);assert(app_task_work_get_status()->corrections==1);
    assert(app_chassis_motion_get_state()==MOTION_STATE_RUNNING);
}
static void test_full_course(uint32_t base)
{
    reset(base);command("TOTALSTART!");bool injected=false;
    for(;elapsed<180000U && task_state!=TASK_END && task_state!=TASK_FAULT;) {
        if(task_state==TASK_SCAN&&!injected) {
            px+=20;py-=15;yaw+=.06f;app_chassis_motion_init();injected=true;
        }
        step(true);
    }
    if(task_state!=TASK_END)printf("course state=%d phase=%d error=%d t=%u\n",task_state,
        app_task_work_get_status()->phase,app_task_work_get_status()->error,elapsed);
    assert(task_state==TASK_END && !laser && laser_starts==1);
    assert(send_count[0x10]>=3 && send_count[0x20]==1 && send_count[0x30]==1 && send_count[0x40]==1);
    assert(send_count[0x21]>=1 && send_count[0x31]>=1 && send_count[0x41]>=1);
    assert(pick_calls==2 && place_calls==1);
    float error=hypotf(px-3350,py-3200);assert(error<10);
    printf("PASS: full route+work, rollover=%d, time=%u ms, final model error=%.2f mm\n",base!=0,elapsed,error);
}
static void test_stop_and_errors(void)
{
    reset(0);config_ready=false;enter_work(WORK_QR,false);
    for(unsigned i=0;i<2000;++i)step(true);
    assert(app_task_work_get_status()->phase==WORK_WAIT_CONFIG && send_count[0x10]==0);
    reset(0);enter_work(WORK_TARGET,true);until_phase(WORK_LASER,30000);assert(laser);
    command("TOTALSTOP!");assert(!laser);
    deliver(0x81,last_payload,last_payload_len);step(true);
    assert(task_state==TASK_IDLE && !task_running && !app_task_work_get_status()->request_pending);
    for(unsigned i=0;i<30;++i)step(true);
    assert(arm_stops==1 && send_count[0x7f]==1 && laser_starts==1);
    assert(hypotf(vx,vy)<.001f && fabsf(omega)<.001f);
    reset(0);enter_work(WORK_QR,true);until_phase(WORK_WAIT_MOTION,5000);
    command("TOTALSTOP!");step(true);
    assert(task_state==TASK_IDLE && app_chassis_motion_get_state()==MOTION_STATE_IDLE);
    reset(UINT32_MAX-1000U);automatic_reply=false;enter_work(WORK_QR,false);
    until_phase(WORK_WAIT_LOCATE,4000);until_fault(24000);
    assert(app_task_work_get_status()->error==WORK_ERR_TIMEOUT);
    reset(0);automatic_reply=false;enter_work(WORK_RESCUE,false);
    until_phase(WORK_SCAN_DWELL,4000);
    uint8_t p[64];result_header(p,tx_cmd,tx_seq,VISION_FAILURE,9);deliver(0x81,p,15);
    until_fault(20);assert(app_task_work_get_status()->vision_reason==9);
    reset(0);automatic_reply=false;force_tx=HAL_ERROR;enter_work(WORK_QR,false);
    until_fault(4000);assert(app_task_work_get_status()->error==WORK_ERR_TX);
    assert(!app_task_work_get_status()->request_pending);
    reset(0);automatic_reply=false;force_tx=HAL_BUSY;enter_work(WORK_QR,false);
    until_fault(5000);assert(app_task_work_get_status()->error==WORK_ERR_TX && send_count[0x10]==0);
    reset(0);automatic_reply=false;enter_work(WORK_QR,false);until_phase(WORK_WAIT_LOCATE,4000);
    uint8_t n=make_scene_result(p);p[7]=4;deliver(0x81,p,n);until_fault(20);
    assert(app_task_work_get_status()->error==WORK_ERR_DATA);
    reset(0);automatic_reply=false;enter_work(WORK_QR,false);until_phase(WORK_WAIT_LOCATE,4000);
    yaw+=.05f;step(true);assert(task_state==TASK_FAULT);
    assert(app_task_work_get_status()->error==WORK_ERR_OBSERVATION_MOVED);
    reset(0);enter_work(WORK_QR,true);until_phase(WORK_WAIT_MOTION,5000);
    app_chassis_motion_stop();until_fault(20);
    assert(app_task_work_get_status()->error==WORK_ERR_MOTION);
    reset(0);enter_work(WORK_QR,true);until_phase(WORK_WAIT_LOCATE,4000);
    imu_ok=false;until_fault(20);assert(app_task_work_get_status()->error==WORK_ERR_IMU);
    reset(0);enter_work(WORK_QR,false);px+=400;
    until_fault(4000);assert(app_task_work_get_status()->error==WORK_ERR_ALIGNMENT);

    reset(0);enter_work(WORK_BOMB,false);nonconverging=true;
    until_fault(40000);assert(app_task_work_get_status()->error==WORK_ERR_ALIGNMENT);
    assert(app_task_work_get_status()->corrections==4);
    reset(0);enter_work(WORK_QR,false);arm_wait=true;
    until_fault(5000);assert(app_task_work_get_status()->error==WORK_ERR_ARM);

    reset(0);automatic_reply=false;enter_work(WORK_QR,false);
    until_phase(WORK_WAIT_LOCATE,4000);
    n=make_scene_result(p);elapsed+=23000;deliver(0x81,p,n);
    until_fault(20);assert(app_task_work_get_status()->error==WORK_ERR_TIMEOUT);

    /* Start waits for old cancellation TX; no late 0x7F can cancel the new request. */
    reset(0);automatic_reply=false;enter_work(WORK_QR,false);
    until_phase(WORK_WAIT_LOCATE,4000);force_tx=HAL_BUSY;
    command("TOTALSTOP!");step(true);command("TOTALSTART!");
    for(unsigned i=0;i<100;++i)step(true);
    assert(task_state==TASK_IDLE && task_total_start_requested && app_task_work_cancel_pending());
    force_tx=HAL_OK;
    for(unsigned i=0;i<50;++i)step(true);
    assert(task_state==TASK_MOVE1 && task_running && send_count[0x7f]==1);
    puts("PASS: UART seq/busy/error, stale/duplicate/observation rejection, remeasurement, total stop, timeouts and missing model");
}
static void test_bucket_transport(void)
{
    for (uint8_t selected=1U; selected<=3U; ++selected) {
        reset(0);
        scene_slot=selected;
        enter_work(WORK_BOMB,false);
        until_phase(WORK_COMPLETE,60000);
        assert(pick_calls==1U && place_calls==1U);
        /* Fixture slots use +X 80mm spacing: bucket always at slot2. */
        assert(fabsf(placed_x-(anchor[WORK_BOMB][0]+80))<3);
        assert(fabsf(placed_y-anchor[WORK_BOMB][1])<3);
        assert(fabsf((placed_x-picked_x)-(2-(int)selected)*80)<3);
        assert(fabsf(placed_y-picked_y)<3);
        assert(hypotf(px-anchor[WORK_BOMB][0],py-anchor[WORK_BOMB][1])<3);
    }
    reset(0);enter_work(WORK_RESCUE,false);
    until_phase(WORK_COMPLETE,60000);
    assert(pick_calls==1U && place_calls==0U);
    puts("PASS: all bomb selections transport to slot2 and return; rescue skips place");
}

int main(void)
{
    test_seq_and_events();test_full_course(0);test_full_course(UINT32_MAX-5000U);test_stop_and_errors();
    test_bucket_transport();
    return 0;
}
