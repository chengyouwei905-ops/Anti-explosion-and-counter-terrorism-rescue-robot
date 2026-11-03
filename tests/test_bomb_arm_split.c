/* Actual arm port/IK/packet encoder. Reuse UART/tick fixtures, independently of
 * the historical reference-coordinate assertions in test_work_port. */
#define main legacy_port_fixture_main
#include "test_work_port.c"
#undef main

static uint16_t current_positions[5];

static void apply_frame(unsigned f, unsigned first_id, unsigned count)
{
    assert(f<sent && frames[f][3]==3 && frames[f][4]==count);
    assert(frame_sizes[f]==7+3*count);
    for(unsigned i=0;i<count;++i) {
        unsigned id=frames[f][7+3*i];
        assert(id==first_id+i);
        current_positions[id-1]=(uint16_t)position(f,i);
    }
}

static void expect_tcp(double x,double y,double z,double pitch)
{
    const double scale=.24*3.14159265358979323846/180.0;
    Roboarm_State_t joints={
        (YAW_SERVO_OFFSET-(double)current_positions[0])*scale,
        ((double)current_positions[1]-JOINT1_OFFSET)*scale,
        (JOINT2_OFFSET-(double)current_positions[2])*scale,
        ((double)current_positions[3]-JOINT3_OFFSET)*scale
    };
    End_Effector_Pos_t p;
    assert(alg_roboarm_forward(&joints,&p)==ROBOARM_KINEMATICS_OK);
    assert(fabs(p.x-x)<3 && fabs(p.y-y)<3 && fabs(p.z-z)<3 && fabs(p.pitch-pitch)<.02);
}

static void bomb_sequence(uint32_t base)
{
    reset(base);memset(current_positions,0,sizeof(current_positions));
    run(WORK_BOMB,ARM_PICK,1);
    assert(sent==5);
    apply_frame(0,5,1);assert(current_positions[4]==CLAMP_OPEN_POSITION);
    apply_frame(1,1,4);expect_tcp(190,0,-35,-1.25);
    uint16_t pick_yaw=current_positions[0];
    apply_frame(2,5,1);assert(current_positions[4]==CLAMP_CLOSE_POSITION);
    /* Lift to the QR radial/vertical/pitch geometry without changing the pick yaw. */
    apply_frame(3,2,3);
    assert(current_positions[0]==pick_yaw && current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(260,0,60,0);
    uint16_t lift_joints[3];memcpy(lift_joints,&current_positions[1],sizeof(lift_joints));
    apply_frame(4,1,1);
    assert(memcmp(lift_joints,&current_positions[1],sizeof(lift_joints))==0);
    expect_tcp(0,260,60,0);

    run(WORK_BOMB,ARM_PLACE,1);
    assert(sent==8);
    uint16_t travel_yaw=current_positions[0];
    apply_frame(5,2,3);
    assert(current_positions[0]==travel_yaw && current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(0,180,50,-1.2);
    memcpy(lift_joints,&current_positions[1],sizeof(lift_joints));
    apply_frame(6,1,1);
    assert(memcmp(lift_joints,&current_positions[1],sizeof(lift_joints))==0);
    expect_tcp(-180,0,50,-1.2);
    apply_frame(7,5,1);assert(current_positions[4]==CLAMP_OPEN_POSITION);

    run(WORK_BOMB,ARM_TRAVEL,1);
    assert(sent==10);
    uint16_t place_yaw=current_positions[0];
    apply_frame(8,2,3);
    assert(current_positions[0]==place_yaw && current_positions[4]==CLAMP_OPEN_POSITION);
    expect_tcp(-260,0,60,0);
    memcpy(lift_joints,&current_positions[1],sizeof(lift_joints));
    apply_frame(9,1,1);
    assert(memcmp(lift_joints,&current_positions[1],sizeof(lift_joints))==0);
    expect_tcp(0,260,60,0);
}

static void split_wait_retry_cancel(void)
{
    reset(0);
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,true)==ARM_WAITING && sent==1);
    assert(frames[0][4]==3 && frames[0][7]==2);
    tick=3599;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_WAITING && sent==1);
    tick=3600;uart_result=RET_BUSY;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_WAITING && sent==1);
    tick=3610;uart_result=RET_OK;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_WAITING && sent==1);
    tick=3620;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_WAITING && sent==2);
    assert(frames[1][4]==1 && frames[1][7]==1);
    tick=7219;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_WAITING && sent==2);
    tick=7220;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_DONE && sent==2);

    reset(0);
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,true)==ARM_WAITING && sent==1);
    assert(task_work_arm_poll(WORK_BOMB,ARM_CANCEL,1,true)==ARM_DONE);
    tick=10000;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_FAILED && sent==1);
    reset(0);
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,true)==ARM_WAITING && sent==1);
    tick=3600;uart_result=RET_ERROR;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_FAILED && sent==1);
    reset(0);
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,true)==ARM_WAITING && sent==1);
    tick=3600;uart_result=RET_BUSY;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_WAITING);
    tick=4600;
    assert(task_work_arm_poll(WORK_BOMB,ARM_TRAVEL,1,false)==ARM_FAILED && sent==1);
}

int main(void)
{
    bomb_sequence(0);bomb_sequence(UINT32_MAX-1500U);
    split_wait_retry_cancel();
    reset(0);
    run(WORK_RESCUE,ARM_PICK,1);run(WORK_RESCUE,ARM_TRAVEL,1);
    assert(sent==5 && frames[1][4]==4 && frames[3][4]==4 && frames[4][4]==4);
    assert_clamp(0,CLAMP_OPEN_POSITION);assert_clamp(2,CLAMP_CLOSE_POSITION);
    for(unsigned kind=0;kind<WORK_COUNT;++kind) {
        reset(0);run((TaskWorkKind_t)kind,ARM_OBSERVE,kind==WORK_QR?0:1);
        assert(sent==1 && frames[0][4]==4); /* Observations still synchronize all four joints. */
    }
    reset(0);
    const Roboarm_Servo_t yaw={YAW_SERVO,0};
    assert(app_roboarm_servo_move_direct_it(&yaw,1,3000)==RET_OK);
    assert(frames[0][4]==1 && frames[0][7]==YAW_SERVO && position(0,0)==YAW_SERVO_OFFSET);
    unsigned before=sent;
    assert(app_roboarm_servo_move_direct_it(NULL,1,3000)==RET_PARAM);
    const Roboarm_Servo_t duplicate[]={{JOINT1,0},{JOINT1,0}};
    assert(app_roboarm_servo_move_direct_it(duplicate,2,3000)==RET_PARAM);
    const Roboarm_Servo_t bad={YAW_SERVO,NAN};
    assert(app_roboarm_servo_move_direct_it(&bad,1,3000)==RET_PARAM && sent==before);
    uart_result=RET_BUSY;
    assert(app_roboarm_servo_move_direct_it(&yaw,1,3000)==RET_BUSY && sent==before);
    puts("PASS: bomb pick/place/retract split ID2..4 then ID1, FK endpoints, clamp order, timing/retry/cancel/rollover; rescue/observe unchanged");
    return 0;
}
