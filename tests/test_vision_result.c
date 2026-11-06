#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "proto_vision_result.h"
#include "alg_vision_align.h"

static unsigned frames;
static proto_vision_frame_t last;
static void callback(const proto_vision_frame_t *f) { last = *f; ++frames; }
static void put_float(uint8_t *p, float v)
{
    uint32_t n; memcpy(&n, &v, 4);
    for (unsigned i=0; i<4; ++i) p[i]=(uint8_t)(n>>(8*i));
}
static size_t unhex(const char *s, uint8_t *out)
{
    size_t n=0; unsigned v; int consumed;
    while (sscanf(s," %2x%n", &v, &consumed)==1) { out[n++]=(uint8_t)v; s+=consumed; }
    return n;
}
static VisionResult_t decode_example(const char *hex, uint8_t cmd, uint8_t flags)
{
    uint8_t bytes[70]; size_t n=unhex(hex,bytes);
    proto_vision_init(); proto_vision_set_cb(callback); frames=0;
    for (size_t i=0;i<n;++i) proto_vision_feed(&bytes[i],1);
    assert(frames==1 && last.cmd==VISION_CMD_RESULT && n==last.len+6U);
    VisionResult_t r;
    assert(proto_vision_decode_result(last.payload,last.len,&r));
    assert(r.task_cmd==cmd && r.flags==flags);
    return r;
}
static void test_examples_and_validation(void)
{
    VisionResult_t r=decode_example("AA 55 1F 07 81 01 21 2A 00 03 00 03 05 34 12 04 03 02 01 00 00 00 48 41 00 00 40 C0 00 00 E4 43 00 00 00 3F 44",0x21,3);
    assert(r.xyz_mm[0]==12.5f && r.xyz_mm[1]==-3 && r.xyz_mm[2]==456 && r.scatter_mm==.5f);
    assert(r.run_id==0x1234 && r.frame_id==0x01020304 && r.input_seq==42);
    assert(proto_vision_result_control_ready(&r,5,8));
    decode_example("AA 55 2F 06 81 01 10 29 00 07 00 00 05 33 12 04 03 02 01 00 00 00 48 41 00 00 40 C0 00 00 E4 43 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 3F 9B",0x10,7);
    assert(proto_vision_decode_result(last.payload,last.len,&r));
    assert(r.quaternion_wxyz[0]==1 && r.scatter_mm==.5f);
    decode_example("AA 55 0F 08 81 01 20 2B 00 00 00 03 00 35 12 05 03 02 01 00 A8",0x20,0);
    assert(proto_vision_decode_result(last.payload,last.len,&r) && r.slot_id==3);
    assert(!proto_vision_result_control_ready(&r,5,8));
    r=decode_example("AA 55 0F 09 81 01 41 2C 01 00 00 00 00 36 12 00 00 00 00 09 7C",0x41,0);
    assert(r.status==VISION_FAILURE && r.reason==9);
    r=decode_example("AA 55 2B 0A 81 01 31 2D 00 0B 00 02 05 37 12 04 03 02 01 00 00 00 48 41 00 00 40 C0 00 00 E4 43 00 00 00 00 00 00 00 00 00 00 80 3F 00 00 00 3F A4",0x31,11);
    assert(r.normal[2]==1 && r.scatter_mm==.5f);
    uint8_t original[64]; memcpy(original,last.payload,64);
    for (unsigned n=0;n<43;++n) assert(!proto_vision_decode_result(original,n,&r));
    uint8_t p[64];
    const unsigned bad_offsets[]={0,3,4,5,6};
    const uint8_t bad_values[]={2,4,0x6b,1,4};
    for (unsigned i=0;i<5;++i) {
        memcpy(p,original,64); p[bad_offsets[i]]=bad_values[i];
        assert(!proto_vision_decode_result(p,43,&r));
    }
    for (unsigned i=15;i<43;i+=4) {
        memcpy(p,original,64); put_float(p+i,NAN);
        assert(!proto_vision_decode_result(p,43,&r));
        put_float(p+i,INFINITY); assert(!proto_vision_decode_result(p,43,&r));
    }
    memcpy(p,original,64); put_float(p+39,-1); assert(!proto_vision_decode_result(p,43,&r));
    memcpy(p,original,64); p[4]|=VISION_CANDIDATE; assert(!proto_vision_decode_result(p,43,&r));
    p[3]=VISION_OBSERVATION;p[4]&=(uint8_t)~VISION_CONTROL_READY;
    assert(proto_vision_decode_result(p,43,&r) && !proto_vision_result_control_ready(&r,5,8));
    memcpy(p,original,64); p[7]=4;
    assert(proto_vision_decode_result(p,43,&r) && !proto_vision_result_control_ready(&r,5,8));
    p[7]=5; put_float(p+39,8.01f);
    assert(proto_vision_decode_result(p,43,&r) && !proto_vision_result_control_ready(&r,5,8));
    /* 可选字段按 flags 顺序：XYZ + NORMAL + GRASP + scatter = 55 字节。 */
    memcpy(p,original,64); p[4]|=VISION_GRASP;
    put_float(p+39,11);put_float(p+43,22);put_float(p+47,33);put_float(p+51,.25f);
    assert(proto_vision_decode_result(p,55,&r) && r.grasp_mm[2]==33 && r.scatter_mm==.25f);
    assert(!proto_vision_decode_result(p,54,&r));
    assert(!proto_vision_decode_result(p,56,&r));
    /* 扫描失败和成功都是 15 字节，必须检查 status。 */
    memset(p,0,64);p[0]=1;p[1]=VISION_CMD_BOMB_SCAN;p[3]=VISION_FAILURE;p[14]=9;
    assert(proto_vision_decode_result(p,15,&r) && r.status==VISION_FAILURE);
    p[3]=VISION_SUCCESS; assert(!proto_vision_decode_result(p,15,&r));
    p[6]=1;p[14]=0; assert(proto_vision_decode_result(p,15,&r));

    uint8_t frame[70];uint8_t len=proto_vision_pack(VISION_CMD_RESULT,p,15,frame,sizeof(frame));
    proto_vision_init();proto_vision_set_cb(callback);frames=0;
    proto_vision_feed(frame,4);assert(frames==0);
    proto_vision_feed(frame+4,len-4);proto_vision_feed(frame,len);assert(frames==2);
    frame[len-1]^=1;proto_vision_feed(frame,len);assert(frames==2);
    uint8_t oversized[]={0xaa,0x55,65};proto_vision_feed(oversized,3);
    assert(proto_vision_get_stat()->crc_errors==1 && proto_vision_get_stat()->len_errors==1);
    frame[len-1]^=1;proto_vision_feed(frame,len);assert(frames==3);
}

static void near(float x,float y) { assert(fabsf(x-y)<.002f); }
static void test_measurement_sample_limits(void)
{
    VisionResult_t r={.task_cmd=VISION_CMD_QR,.status=VISION_SUCCESS,
        .flags=VISION_POSITION|VISION_CONTROL_READY,.scatter_mm=12};
    const uint8_t accepted[]={3,4,5,7,32};
    const uint8_t rejected[]={0,1,2,33,255};
    for(unsigned i=0;i<sizeof(accepted)/sizeof(accepted[0]);++i) {
        r.sample_count=accepted[i];assert(proto_vision_result_control_ready(&r,3,16));
    }
    for(unsigned i=0;i<sizeof(rejected)/sizeof(rejected[0]);++i) {
        r.sample_count=rejected[i];assert(!proto_vision_result_control_ready(&r,3,16));
    }
    r.sample_count=5;
    assert(!proto_vision_result_control_ready(&r,0,16));
    assert(!proto_vision_result_control_ready(&r,33,16));
    r.scatter_mm=16;assert(proto_vision_result_control_ready(&r,3,16));
    r.scatter_mm=16.01f;assert(!proto_vision_result_control_ready(&r,3,16));
    r.scatter_mm=12;r.xyz_mm[0]=NAN;assert(!proto_vision_result_control_ready(&r,3,16));
    r.xyz_mm[0]=0;r.flags|=VISION_CANDIDATE;assert(!proto_vision_result_control_ready(&r,3,16));
    r.flags=VISION_POSITION;assert(!proto_vision_result_control_ready(&r,3,16));
    r.flags|=VISION_CONTROL_READY;r.status=VISION_OBSERVATION;
    assert(!proto_vision_result_control_ready(&r,3,16));
}
static VisionAlignmentConfig_t config(void)
{
    VisionAlignmentConfig_t c={
        .body_from_tcp={.rotation={{1,0,0},{0,1,0},{0,0,1}}},
        .desired_reference_body={20,300,100},
        .min_reference_body={-2000,-2000,-2000},.max_reference_body={2000,2000,2000},
        .map_reference_valid=true,.reference_map={1000,2000,100}
    };
    return c;
}
static void test_transforms(void)
{
    VisionTransform_t t;
    End_Effector_Pos_t p={100,200,150,-.2};
    assert(alg_vision_transform_from_arm_pose(&p,&t));
    near(t.translation[0],-100);near(t.translation[1],200);near(t.translation[2],150);
    float c=2/sqrtf(5),s=1/sqrtf(5);
    near(t.rotation[0][0],c);near(t.rotation[1][0],s);
    near(t.rotation[0][1],-s*cosf(.2f));near(t.rotation[2][1],-sinf(.2f));
    p.x=p.y=0;assert(!alg_vision_transform_from_arm_pose(&p,&t));
    p.x=NAN;assert(!alg_vision_transform_from_arm_pose(&p,&t));
    Roboarm_State_t joints={.yaw_angle=.6,.joint1_angle=.2,.joint2_angle=1.3,.joint3_angle=.1};
    End_Effector_Pos_t fk;
    assert(alg_roboarm_forward(&joints,&fk)==ROBOARM_KINEMATICS_OK);
    assert(alg_vision_transform_from_arm_joints(&joints,&t));
    near(t.translation[0],(float)-fk.x);
    near(t.rotation[1][0],sinf((float)(.6 * ROBOARM_YAW_OUTPUT_PER_SERVO)));

    VisionResult_t r={.task_cmd=VISION_CMD_BOMB_LOCATE,.status=0,.flags=3,.sample_count=5,.scatter_mm=1,
                      .xyz_mm={40,270,100}};
    VisionAlignmentConfig_t cfg=config();VisionAlignment_t a;
    assert(alg_vision_alignment_compute(&r,&cfg,1.57079632679f,&a));
    near(a.delta_body[0],20);near(a.delta_body[1],-30);near(a.delta_yaw,0);
    near(a.chassis_map[0],1270);near(a.chassis_map[1],1960);
    near(a.chassis_from_reference_body[0],-40);
    r.scatter_mm=20;
    assert(alg_vision_alignment_compute(&r,&cfg,0,&a));
    r.scatter_mm=20.01f;
    assert(!alg_vision_alignment_compute(&r,&cfg,0,&a));
    r.scatter_mm=1;
    cfg.body_from_tcp.rotation[0][0]=-1;
    assert(!alg_vision_alignment_compute(&r,&cfg,0,&a)); /* 镜像不是旋转 */
    cfg=config();cfg.yaw_mode=VISION_YAW_TARGET_NORMAL;cfg.desired_direction_body[1]=1;
    r.task_cmd=VISION_CMD_TARGET_LOCATE;r.flags|=VISION_NORMAL;
    float d=.12f;r.normal[0]=-sinf(d);r.normal[1]=cosf(d);
    assert(alg_vision_alignment_compute(&r,&cfg,0,&a));near(a.delta_yaw,d);
    near(a.delta_body[0],40-(20*cosf(d)-300*sinf(d)));
    near(a.delta_body[1],270-(20*sinf(d)+300*cosf(d)));
    r.normal[0]=r.normal[1]=0;r.normal[2]=1;
    assert(!alg_vision_alignment_compute(&r,&cfg,0,&a)); /* 竖直法向不能解 yaw */
    cfg.yaw_mode=VISION_YAW_QR_AXIS;cfg.qr_axis[1]=1;
    r.task_cmd=VISION_CMD_QR;r.flags=7;r.quaternion_wxyz[0]=cosf(d/2);r.quaternion_wxyz[3]=sinf(d/2);
    assert(alg_vision_alignment_compute(&r,&cfg,0,&a));near(a.delta_yaw,d);
    near(a.map_yaw,-d); /* 当前朝向 + 修正量 = 期望地图朝向 */
    r.quaternion_wxyz[0]=0;r.quaternion_wxyz[3]=0;
    assert(!alg_vision_alignment_compute(&r,&cfg,0,&a));
    r.task_cmd=VISION_CMD_BOMB_LOCATE;r.flags=3;cfg=config();
    cfg.min_reference_body[0]=50;assert(!alg_vision_alignment_compute(&r,&cfg,0,&a));
    puts("PASS: K230 examples, strict payload validation, stream framing, arm axes/FK, planar correction and observable yaw");
}
static void test_target_height_diagnostic(void)
{
    VisionAlignmentConfig_t cfg=config();
    cfg.min_reference_body[2]=50;cfg.max_reference_body[2]=150;
    VisionResult_t r={.task_cmd=VISION_CMD_TARGET_LOCATE,.status=VISION_SUCCESS,
        .flags=VISION_POSITION|VISION_CONTROL_READY,.sample_count=5,.scatter_mm=1,
        .xyz_mm={40,270,160}};
    VisionAlignment_t a;
    const float heights[]={40,160,-500,1000};
    for(unsigned i=0;i<sizeof(heights)/sizeof(heights[0]);++i) {
        r.xyz_mm[2]=heights[i];
        assert(alg_vision_alignment_compute(&r,&cfg,0,&a));
        near(a.reference_body[2],heights[i]);
        near(a.delta_body[0],20);near(a.delta_body[1],-30);
    }
    r.xyz_mm[2]=NAN;
    assert(!alg_vision_alignment_compute(&r,&cfg,0,&a));
    r.xyz_mm[2]=160;r.xyz_mm[0]=2001;
    assert(!alg_vision_alignment_compute(&r,&cfg,0,&a));
    r.xyz_mm[0]=40;r.xyz_mm[1]=2001;
    assert(!alg_vision_alignment_compute(&r,&cfg,0,&a));
    r.xyz_mm[1]=270;
    const uint8_t others[]={VISION_CMD_QR,VISION_CMD_BOMB_LOCATE,VISION_CMD_RESCUE_LOCATE};
    for(unsigned i=0;i<sizeof(others)/sizeof(others[0]);++i) {
        r.task_cmd=others[i];
        assert(!alg_vision_alignment_compute(&r,&cfg,0,&a));
    }
    puts("PASS: target Z is diagnostic only, finite/XY checks retained, other work Z checks unchanged");
}
int main(void) { test_examples_and_validation(); test_measurement_sample_limits(); test_transforms(); test_target_height_diagnostic(); return 0; }
