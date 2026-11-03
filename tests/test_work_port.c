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
static uint16_t frame_sizes[32];
static int uart_result=RET_OK;
static bool laser;
static bool allow_blocking;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_GPIO_WritePin(void *port,uint16_t pin,GPIO_PinState state)
{ assert(port==LASER_GPIO_Port && pin==LASER_Pin);laser=state==GPIO_PIN_SET; }
int bsp_uart_attach(bsp_uart_id_t id,UART_HandleTypeDef *u)
{ (void)id;(void)u;return RET_OK; }
int bsp_uart_send(bsp_uart_id_t id,const uint8_t *p,uint16_t n,uint32_t timeout)
{ (void)timeout;assert(allow_blocking);return bsp_uart_send_it(id,p,n); }
int bsp_uart_send_it(bsp_uart_id_t id,const uint8_t *p,uint16_t n)
{
    assert(id==BSP_UART_ROBOARM && n<=64);
    if(uart_result==RET_OK) { assert(sent<32);frame_sizes[sent]=n;memcpy(frames[sent++],p,n); }
    return uart_result;
}
static unsigned position(unsigned f,unsigned i)
{return frames[f][8+i*3]+256U*frames[f][9+i*3];}
static void assert_pose(unsigned f,Roboarm_Pose_t pose)
{
    /* Expected packets for the field poses, with current servo calibration. */
    static const uint16_t field_positions[ROBOARM_POSE_COUNT][4]={
        [ROBOARM_POSE_BOMB_OBSERVE]={29,741,858,681},
        [ROBOARM_POSE_BOMB_PICK]={29,959,762,586},
        [ROBOARM_POSE_BOMB_PLACE]={971,775,937,748},
        [ROBOARM_POSE_TARGET_OBSERVE]={29,888,702,227},
        [ROBOARM_POSE_RESCUE_OBSERVE]={32,455,996,601},
        [ROBOARM_POSE_RESCUE_PICK]={29,700,908,552}
    };
    const uint16_t *p=pose==ROBOARM_POSE_QR_OBSERVE ?
        app_roboarm_pose_positions(pose):field_positions[pose];
    assert(frames[f][4]==4);
    for(unsigned i=0;i<4;++i)assert(frames[f][7+i*3]==i+1 && position(f,i)==p[i]);
}
static void assert_clamp(unsigned f,unsigned p)
{assert(frames[f][4]==1 && frames[f][7]==JOINT_CLAMP && position(f,0)==p);}
static void assert_joint_ids(unsigned f,unsigned first,unsigned count)
{
    assert(frames[f][4]==count);
    for(unsigned i=0;i<count;++i)assert(frames[f][7+3*i]==first+i);
}
static void reset(uint32_t base)
{
    tick=base;sent=0;uart_result=RET_OK;allow_blocking=false;
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
    assert(sent==5);
    assert_clamp(0,437);assert_pose(1,ROBOARM_POSE_BOMB_PICK);
    assert_clamp(2,747);assert_joint_ids(3,2,3);assert_joint_ids(4,1,1);
    run(WORK_BOMB,ARM_PLACE,3);
    assert_joint_ids(5,2,3);assert_joint_ids(6,1,1);assert_clamp(7,437);
    run(WORK_BOMB,ARM_TRAVEL,3);assert_joint_ids(8,2,3);assert_joint_ids(9,1,1);

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
    /* Both FK and IK share the requested +/-78 degree first-joint limit. */
    const double radians=3.14159265358979323846/180.0;
    for(int sign=-1;sign<=1;sign+=2) {
        Roboarm_State_t j={0,sign*77.9*radians,(sign<0?65:105)*radians,0};
        End_Effector_Pos_t p;
        Roboarm_State_t inverse;
        assert(alg_roboarm_forward(&j,&p)==ROBOARM_KINEMATICS_OK);
        assert(alg_roboarm_invert(&p,&inverse)==ROBOARM_KINEMATICS_OK);
        assert(fabs(inverse.joint1_angle-j.joint1_angle)<1e-9);
        j.joint1_angle=sign*78.1*radians;
        assert(alg_roboarm_forward(&j,&p)==ROBOARM_KINEMATICS_ERR);
    }

    reset(0);
    assert(app_roboarm_servo_move_to_it(NAN,0,0,0,3000)==RET_PARAM && sent==0);
    assert(app_roboarm_servo_move_to_it(1e6,0,0,0,3000)!=RET_OK && sent==0);
    assert(app_roboarm_servo_move_to_it(131,0,77,-.3686,3000)!=RET_OK && sent==0);

    /* The new async path preserves the existing XYZ conversion/packet. */
    allow_blocking=true;
    assert(app_roboarm_servo_move_to(154,0,6,-1.25)==RET_OK && sent==1);
    allow_blocking=false;
    assert(app_roboarm_servo_move_to_it(154,0,6,-1.25,3000)==RET_OK && sent==2);
    assert(frame_sizes[0]==frame_sizes[1]);
    assert(memcmp(frames[0],frames[1],frame_sizes[0])==0);

    TaskWorkConfig_t c;
    assert(!task_work_load_config(WORK_COUNT,&c));
    assert(!task_work_load_config(WORK_QR,NULL));
    for(unsigned k=0;k<WORK_COUNT;++k) {
        assert(task_work_load_config((TaskWorkKind_t)k,&c));
        const float expected_xyz[WORK_COUNT][3]={
            {0,1250,55},{-225,0,0},{-620,0,20},
            {-247.784439f,19.8726902f,33.2348137f}
        };
        unsigned slots=k==WORK_QR?1U:3U;
        for(unsigned i=0;i<slots;++i) {
            if(k!=WORK_QR) {
                const End_Effector_Pos_t expected_observe[WORK_COUNT]={
                    [WORK_BOMB]={154,0,6,-1.25},
                    [WORK_TARGET]={180,0,25,0},
                    [WORK_RESCUE]={133.537333615,1.525593299,82.971289222,-.338}
                };
                assert(c.slot[i].observe_pose.x==expected_observe[k].x);
                assert(c.slot[i].observe_pose.y==expected_observe[k].y);
                assert(c.slot[i].observe_pose.z==expected_observe[k].z);
                assert(c.slot[i].observe_pose.pitch==expected_observe[k].pitch);
            }
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
        reset(0);
        run((TaskWorkKind_t)k,ARM_OBSERVE,k==WORK_QR?0:1);
        const Roboarm_Pose_t observations[]={ROBOARM_POSE_QR_OBSERVE,ROBOARM_POSE_BOMB_OBSERVE,
            ROBOARM_POSE_TARGET_OBSERVE,ROBOARM_POSE_RESCUE_OBSERVE};
        assert(sent==1);assert_pose(0,observations[k]);
    }
    /* Captured TCP measurement at the real rescue parking reference. The
     * displayed arm-origin-relative value has the opposite sign to p_body.
     */
    assert(task_work_load_config(WORK_RESCUE,&c));
    VisionResult_t captured={.task_cmd=VISION_CMD_RESCUE_LOCATE,.status=VISION_SUCCESS,
        .flags=VISION_POSITION|VISION_CONTROL_READY,.sample_count=5,.scatter_mm=1,
        .xyz_mm={17.0407677f,124.466324f,-8.9709912f}};
    VisionAlignment_t capture_alignment;
    assert(alg_vision_alignment_compute(&captured,&c.slot[0].alignment,0,&capture_alignment));
    assert(fabsf(capture_alignment.reference_body[0]+247.784439f)<.001f);
    assert(fabsf(capture_alignment.reference_body[1]-19.8726902f)<.001f);
    assert(fabsf(capture_alignment.reference_body[2]-33.2348137f)<.001f);
    assert(hypotf(capture_alignment.delta_body[0],capture_alignment.delta_body[1])<.001f);
    /* The existing broad Z workspace still rejects a +60mm outlier. */
    for(unsigned i=0;i<3;++i)
        captured.xyz_mm[i]+=60*c.slot[0].alignment.body_from_tcp.rotation[2][i];
    assert(!alg_vision_alignment_compute(&captured,&c.slot[0].alignment,0,&capture_alignment));
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
    /* New XYZ requests also keep the port nonblocking and retry BUSY. */
    reset(0);uart_result=RET_BUSY;
    assert(task_work_arm_poll(WORK_BOMB,ARM_OBSERVE,1,true)==ARM_WAITING && sent==0);
    tick=100;uart_result=RET_OK;
    assert(task_work_arm_poll(WORK_BOMB,ARM_OBSERVE,1,false)==ARM_WAITING && sent==1);
    assert_pose(0,ROBOARM_POSE_BOMB_OBSERVE);
    tick=3700;
    assert(task_work_arm_poll(WORK_BOMB,ARM_OBSERVE,1,false)==ARM_DONE && sent==1);
    reset(0);uart_result=RET_ERROR;
    assert(task_work_arm_poll(WORK_TARGET,ARM_OBSERVE,1,true)==ARM_FAILED && sent==0);
    reset(0);assert(task_work_arm_poll(WORK_BOMB,ARM_PICK,2,true)==ARM_WAITING);
    assert(sent==1);assert(task_work_arm_poll(WORK_BOMB,ARM_CANCEL,2,true)==ARM_DONE);
    tick+=10000;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PICK,2,false)==ARM_FAILED && sent==1);
    task_work_laser_set(true);assert(laser);task_work_laser_set(false);assert(!laser);
    puts("PASS: field XYZ poses/packets/transforms including rescue observation, QR defaults, +/-78 limits, nonblocking work sequence/retry/cancel");
    return 0;
}
