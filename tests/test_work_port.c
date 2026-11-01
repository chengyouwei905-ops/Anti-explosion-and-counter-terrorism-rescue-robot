/* Real port, arm app, FK and packet encoder; only UART/GPIO/tick are mocked. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "app_task_work.h"
#include "app_roboarm.h"
#include "bsp_uart.h"
#include "main.h"

static uint32_t tick;
static unsigned sent;
static uint8_t frames[32][64];
static int uart_result=RET_OK;
static bool laser;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_WritePin(void *port,uint16_t pin,GPIO_PinState state)
{ assert(port==LASER_GPIO_Port && pin==LASER_Pin);laser=state==GPIO_PIN_SET; }
int bsp_uart_attach(bsp_uart_id_t id,UART_HandleTypeDef *u)
{ (void)id;(void)u;return RET_OK; }
int bsp_uart_send(bsp_uart_id_t id,const uint8_t *p,uint16_t n,uint32_t timeout)
{ (void)id;(void)p;(void)n;(void)timeout;assert(!"blocking UART in port");return RET_ERROR; }
int bsp_uart_send_it(bsp_uart_id_t id,const uint8_t *p,uint16_t n)
{
    assert(id==BSP_UART_ROBOARM && n<=64);
    if(uart_result==RET_OK) { assert(sent<32);memcpy(frames[sent++],p,n); }
    return uart_result;
}
static unsigned position(unsigned f,unsigned i)
{return frames[f][8+i*3]+256U*frames[f][9+i*3];}
static void assert_pose(unsigned f,Roboarm_Pose_t pose)
{
    const uint16_t *p=app_roboarm_pose_positions(pose);
    assert(frames[f][4]==4);
    for(unsigned i=0;i<4;++i)assert(frames[f][7+i*3]==i+1 && position(f,i)==p[i]);
}
static void assert_clamp(unsigned f,unsigned p)
{assert(frames[f][4]==1 && frames[f][7]==JOINT_CLAMP && position(f,0)==p);}
static void reset(uint32_t base)
{
    tick=base;sent=0;uart_result=RET_OK;
    assert(task_work_arm_poll(WORK_QR,ARM_CANCEL,0,true)==ARM_DONE);
}
static void run(TaskWorkKind_t kind,TaskArmStep_t step,uint8_t slot)
{
    uint32_t start=tick;
    TaskArmResult_t result=task_work_arm_poll(kind,step,slot,true);
    while(result==ARM_WAITING && (uint32_t)(tick-start)<20000U) {
        tick+=10;
        result=task_work_arm_poll(kind,step,slot,false);
    }
    assert(result==ARM_DONE);
    unsigned before=sent;
    assert(task_work_arm_poll(kind,step,slot,false)==ARM_DONE && sent==before);
}
static void sequence(uint32_t base)
{
    reset(base);
    run(WORK_BOMB,ARM_PICK,3);
    assert(sent==4);
    assert_clamp(0,437);assert_pose(1,ROBOARM_POSE_BOMB_PICK);
    assert_clamp(2,747);assert_pose(3,ROBOARM_POSE_QR_OBSERVE);
    run(WORK_BOMB,ARM_PLACE,3);
    assert_pose(4,ROBOARM_POSE_BOMB_PLACE);assert_clamp(5,437);
    run(WORK_BOMB,ARM_TRAVEL,3);assert_pose(6,ROBOARM_POSE_QR_OBSERVE);

    reset(base);run(WORK_RESCUE,ARM_PICK,1);
    assert_clamp(0,437);assert_pose(1,ROBOARM_POSE_RESCUE_PICK);
    assert_clamp(2,747);assert_pose(3,ROBOARM_POSE_QR_OBSERVE);
    run(WORK_RESCUE,ARM_TRAVEL,1);
    assert(sent==5);assert_pose(4,ROBOARM_POSE_QR_OBSERVE);
    assert(task_work_arm_poll(WORK_RESCUE,ARM_PLACE,1,true)==ARM_FAILED);
    assert(sent==5);
}
int main(void)
{
    TaskWorkConfig_t c;
    assert(!task_work_load_config(WORK_COUNT,&c));
    assert(!task_work_load_config(WORK_QR,NULL));
    for(unsigned k=0;k<WORK_COUNT;++k) {
        assert(task_work_load_config((TaskWorkKind_t)k,&c));
        const float expected_xyz[WORK_COUNT][3]={{0,1250,55},{-225,0,0},{-620,0,20},{-225,0,55}};
        unsigned slots=k==WORK_QR?1U:3U;
        for(unsigned i=0;i<slots;++i) {
            for(unsigned axis=0;axis<3;++axis)
                assert(c.slot[i].alignment.desired_reference_body[axis]==expected_xyz[k][axis]);
            assert(alg_vision_alignment_config_valid(&c.slot[i].alignment));
            /* Real configured transforms: synthesize tool measurements at the
             * desired body point, then 20mm to its right, and check correction. */
            const uint8_t cmds[]={VISION_CMD_QR,VISION_CMD_BOMB_LOCATE,
                VISION_CMD_TARGET_LOCATE,VISION_CMD_RESCUE_LOCATE};
            for(unsigned shifted=0;shifted<2;++shifted) {
                VisionResult_t r={.task_cmd=cmds[k],.status=VISION_SUCCESS,
                    .flags=VISION_POSITION|VISION_CONTROL_READY,.sample_count=5,.scatter_mm=1};
                VisionTransform_t *t=&c.slot[i].alignment.body_from_tcp;
                for(unsigned axis=0;axis<3;++axis) {
                    r.xyz_mm[axis]=0;
                    for(unsigned j=0;j<3;++j)
                        r.xyz_mm[axis]+=t->rotation[j][axis]*(expected_xyz[k][j]+
                            (j==0?20.0f*shifted:0)-t->translation[j]);
                }
                VisionAlignment_t a;
                assert(alg_vision_alignment_compute(&r,&c.slot[i].alignment,0,&a));
                assert(fabsf(a.delta_body[0]-20.0f*shifted)<.001f);
                assert(fabsf(a.delta_body[1])<.001f && fabsf(a.delta_yaw)<.001f);
                assert(fabsf(a.reference_body[2]-expected_xyz[k][2])<.001f);
            }
        }
        assert(c.slot[0].alignment.yaw_mode==VISION_YAW_NONE);
        assert(!c.slot[0].alignment.map_reference_valid);
        if(k!=WORK_QR) {
            assert(c.slot[0].offset_mm[1]==0 && c.slot[1].offset_mm[1]==150 && c.slot[2].offset_mm[1]==300);
        }
        reset(0);run((TaskWorkKind_t)k,ARM_OBSERVE,k==WORK_QR?0:1);
        const Roboarm_Pose_t observations[]={ROBOARM_POSE_QR_OBSERVE,ROBOARM_POSE_BOMB_OBSERVE,
            ROBOARM_POSE_TARGET_OBSERVE,ROBOARM_POSE_RESCUE_OBSERVE};
        assert(sent==1);assert_pose(0,observations[k]);
    }
    sequence(0);sequence(UINT32_MAX-1500U);
    reset(0);uart_result=RET_BUSY;
    assert(task_work_arm_poll(WORK_QR,ARM_OBSERVE,0,true)==ARM_WAITING);
    tick=100;uart_result=RET_OK;
    assert(task_work_arm_poll(WORK_QR,ARM_OBSERVE,0,false)==ARM_WAITING && sent==1);
    tick=3699;
    assert(task_work_arm_poll(WORK_QR,ARM_OBSERVE,0,false)==ARM_WAITING && sent==1);
    tick=3700;
    assert(task_work_arm_poll(WORK_QR,ARM_OBSERVE,0,false)==ARM_DONE && sent==1);
    reset(0);uart_result=RET_BUSY;
    assert(task_work_arm_poll(WORK_QR,ARM_OBSERVE,0,true)==ARM_WAITING);
    tick=1000;assert(task_work_arm_poll(WORK_QR,ARM_OBSERVE,0,false)==ARM_FAILED && sent==0);
    reset(0);uart_result=RET_ERROR;
    assert(task_work_arm_poll(WORK_QR,ARM_OBSERVE,0,true)==ARM_FAILED && sent==0);
    reset(0);assert(task_work_arm_poll(WORK_BOMB,ARM_PICK,2,true)==ARM_WAITING);
    assert(sent==1);assert(task_work_arm_poll(WORK_BOMB,ARM_CANCEL,2,true)==ARM_DONE);
    tick+=10000;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PICK,2,false)==ARM_FAILED && sent==1);
    task_work_laser_set(true);assert(laser);task_work_laser_set(false);assert(!laser);
    puts("PASS: real work port config, observation/pick/place/carry, no hostage release, busy/error/cancel and rollover");
    return 0;
}
