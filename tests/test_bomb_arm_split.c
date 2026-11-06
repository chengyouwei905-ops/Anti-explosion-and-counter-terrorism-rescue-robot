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
    assert(frames[f][5]+256U*frames[f][6]==1000U);
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

static void start_from_travel(uint32_t base)
{
    reset(base);memset(current_positions,0,sizeof(current_positions));
    assert(app_roboarm_servo_move_to_it(0,200,140,0,1000)==RET_OK);
    apply_frame(0,1,4);expect_tcp(0,200,140,0);
    current_positions[4]=CLAMP_CLOSE_POSITION;
    sent=0;
}

static void observe_sequence(TaskWorkKind_t kind,uint32_t base,uint8_t slot)
{
    start_from_travel(base);
    uint16_t original_joints[3];memcpy(original_joints,&current_positions[1],sizeof(original_joints));
    TaskWorkConfig_t config;assert(task_work_load_config(kind,&config));
    End_Effector_Pos_t target=config.slot[slot-1U].observe_pose;
    assert(task_work_arm_poll(kind,ARM_OBSERVE,slot,true)==ARM_WAITING && sent==1);
    if(kind==WORK_RESCUE) {
        /* A single packet starts all four observation joints together. */
        apply_frame(0,1,4);
        assert(current_positions[4]==CLAMP_CLOSE_POSITION);
        expect_tcp(target.x,target.y,target.z,target.pitch);
        tick=base+1299U;
        assert(task_work_arm_poll(kind,ARM_OBSERVE,slot,false)==ARM_WAITING && sent==1);
        tick=base+1300U;
        assert(task_work_arm_poll(kind,ARM_OBSERVE,slot,false)==ARM_DONE && sent==1);
        tick=base+2600U;
        assert(task_work_arm_poll(kind,ARM_OBSERVE,slot,false)==ARM_DONE && sent==1);
        return;
    }
    apply_frame(0,1,1);
    assert(memcmp(original_joints,&current_positions[1],sizeof(original_joints))==0);
    expect_tcp(200,0,140,0);
    tick=base+1299U;
    assert(task_work_arm_poll(kind,ARM_OBSERVE,slot,false)==ARM_WAITING && sent==1);
    tick=base+1300U;
    assert(task_work_arm_poll(kind,ARM_OBSERVE,slot,false)==ARM_WAITING && sent==2);
    uint16_t yaw=current_positions[0];
    apply_frame(1,2,3);
    assert(current_positions[0]==yaw && current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(target.x,target.y,target.z,target.pitch);
    tick=base+2599U;
    assert(task_work_arm_poll(kind,ARM_OBSERVE,slot,false)==ARM_WAITING && sent==2);
    tick=base+2600U;
    assert(task_work_arm_poll(kind,ARM_OBSERVE,slot,false)==ARM_DONE && sent==2);
}

static void bomb_sequence(uint32_t base)
{
    reset(base);memset(current_positions,0,sizeof(current_positions));
    run(WORK_BOMB,ARM_PICK,1);
    assert(sent==5);
    apply_frame(0,5,1);assert(current_positions[4]==CLAMP_OPEN_POSITION);
    apply_frame(1,1,4);expect_tcp(200,0,-23,-1.25);
    uint16_t pick_yaw=current_positions[0];
    apply_frame(2,5,1);assert(current_positions[4]==CLAMP_CLOSE_POSITION);
    /* Lift to the travel geometry without changing the pick yaw. */
    apply_frame(3,2,3);
    assert(current_positions[0]==pick_yaw && current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(200,0,140,0);
    uint16_t lift_joints[3];memcpy(lift_joints,&current_positions[1],sizeof(lift_joints));
    apply_frame(4,1,1);
    assert(memcmp(lift_joints,&current_positions[1],sizeof(lift_joints))==0);
    expect_tcp(0,200,140,0);

    run(WORK_BOMB,ARM_PLACE,1);
    assert(sent==8);
    memcpy(lift_joints,&current_positions[1],sizeof(lift_joints));
    apply_frame(5,1,1);
    assert(memcmp(lift_joints,&current_positions[1],sizeof(lift_joints))==0);
    assert(current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(-200,0,140,0);
    uint16_t place_yaw=current_positions[0];
    apply_frame(6,2,3);
    assert(current_positions[0]==place_yaw && current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(-174,0,25,-1.2);
    apply_frame(7,5,1);assert(current_positions[4]==CLAMP_OPEN_POSITION);

    run(WORK_BOMB,ARM_TRAVEL,1);
    assert(sent==10);
    place_yaw=current_positions[0];
    apply_frame(8,2,3);
    assert(current_positions[0]==place_yaw && current_positions[4]==CLAMP_OPEN_POSITION);
    expect_tcp(-200,0,140,0);
    memcpy(lift_joints,&current_positions[1],sizeof(lift_joints));
    apply_frame(9,1,1);
    assert(memcmp(lift_joints,&current_positions[1],sizeof(lift_joints))==0);
    expect_tcp(0,200,140,0);
}

static void target_sequence(uint32_t base,uint8_t slot)
{
    start_from_travel(base);
    TaskWorkConfig_t cfg;assert(task_work_load_config(WORK_TARGET,&cfg));
    End_Effector_Pos_t observation=cfg.slot[slot-1U].observe_pose;
    run(WORK_TARGET,ARM_OBSERVE,slot);
    assert(sent==2);
    apply_frame(0,1,1);expect_tcp(200,0,140,0);
    apply_frame(1,2,3);expect_tcp(observation.x,observation.y,observation.z,observation.pitch);
    uint16_t target_yaw=current_positions[0];
    run(WORK_TARGET,ARM_TRAVEL,slot);
    assert(sent==4);
    apply_frame(2,2,3);
    assert(current_positions[0]==target_yaw && current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(200,0,140,0);
    uint16_t lift_joints[3];memcpy(lift_joints,&current_positions[1],sizeof(lift_joints));
    apply_frame(3,1,1);
    assert(memcmp(lift_joints,&current_positions[1],sizeof(lift_joints))==0);
    assert(current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(0,200,140,0);
}

static void rescue_sequence(uint32_t base,uint8_t slot)
{
    reset(base);memset(current_positions,0,sizeof(current_positions));
    run(WORK_RESCUE,ARM_PICK,slot);
    assert(sent==5 && (uint32_t)(tick-base)==6500U);
    apply_frame(0,5,1);assert(current_positions[4]==CLAMP_OPEN_POSITION);
    apply_frame(1,1,4);expect_tcp(205,0,78,-.5);
    uint16_t pick_yaw=current_positions[0];
    apply_frame(2,5,1);assert(current_positions[4]==CLAMP_CLOSE_POSITION);
    apply_frame(3,2,3);
    assert(current_positions[0]==pick_yaw && current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(200,0,140,0);
    uint16_t retract_joints[3];memcpy(retract_joints,&current_positions[1],sizeof(retract_joints));
    apply_frame(4,1,1);
    assert(memcmp(retract_joints,&current_positions[1],sizeof(retract_joints))==0);
    assert(current_positions[4]==CLAMP_CLOSE_POSITION);
    expect_tcp(0,200,140,0);

    uint32_t travel_start=tick;
    run(WORK_RESCUE,ARM_TRAVEL,slot);
    assert(sent==7 && (uint32_t)(tick-travel_start)==2600U);
    apply_frame(5,2,3);expect_tcp(0,200,140,0);
    apply_frame(6,1,1);expect_tcp(0,200,140,0);
    assert(current_positions[4]==CLAMP_CLOSE_POSITION);
    assert(task_work_arm_poll(WORK_RESCUE,ARM_PLACE,slot,true)==ARM_FAILED && sent==7);
}

static void place_wait_retry(uint32_t base)
{
    start_from_travel(base);
    uint16_t travel_joints[3];memcpy(travel_joints,&current_positions[1],sizeof(travel_joints));
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,true)==ARM_WAITING && sent==1);
    apply_frame(0,1,1);expect_tcp(-200,0,140,0);
    assert(memcmp(travel_joints,&current_positions[1],sizeof(travel_joints))==0);
    assert(current_positions[4]==CLAMP_CLOSE_POSITION);
    tick=base+1299U;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,false)==ARM_WAITING && sent==1);
    tick=base+1300U;uart_result=RET_BUSY;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,false)==ARM_WAITING && sent==1);
    tick=base+1310U;uart_result=RET_OK;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,false)==ARM_WAITING && sent==1);
    tick=base+1320U;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,false)==ARM_WAITING && sent==2);
    uint16_t place_yaw=current_positions[0];
    apply_frame(1,2,3);expect_tcp(-174,0,25,-1.2);
    assert(current_positions[0]==place_yaw && current_positions[4]==CLAMP_CLOSE_POSITION);
    tick=base+2619U;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,false)==ARM_WAITING && sent==2);
    tick=base+2620U;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,false)==ARM_WAITING && sent==3);
    apply_frame(2,5,1);assert(current_positions[4]==CLAMP_OPEN_POSITION);
    tick=base+3919U;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,false)==ARM_WAITING && sent==3);
    tick=base+3920U;
    assert(task_work_arm_poll(WORK_BOMB,ARM_PLACE,1,false)==ARM_DONE && sent==3);
}

static void split_wait_retry_cancel(TaskWorkKind_t kind,uint32_t base)
{
    reset(base);
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,true)==ARM_WAITING && sent==1);
    assert(frames[0][4]==3 && frames[0][7]==2);
    tick=base+1299U;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_WAITING && sent==1);
    tick=base+1300U;uart_result=RET_BUSY;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_WAITING && sent==1);
    tick=base+1310U;uart_result=RET_OK;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_WAITING && sent==1);
    tick=base+1320U;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_WAITING && sent==2);
    assert(frames[1][4]==1 && frames[1][7]==1);
    tick=base+2619U;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_WAITING && sent==2);
    tick=base+2620U;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_DONE && sent==2);

    reset(base);
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,true)==ARM_WAITING && sent==1);
    assert(task_work_arm_poll(kind,ARM_CANCEL,1,true)==ARM_DONE);
    tick=base+10000U;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_FAILED && sent==1);
    reset(base);
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,true)==ARM_WAITING && sent==1);
    tick=base+1300U;uart_result=RET_ERROR;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_FAILED && sent==1);
    reset(base);
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,true)==ARM_WAITING && sent==1);
    tick=base+1300U;uart_result=RET_BUSY;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_WAITING);
    tick=base+2300U;
    assert(task_work_arm_poll(kind,ARM_TRAVEL,1,false)==ARM_FAILED && sent==1);
}

int main(void)
{
    bomb_sequence(0);bomb_sequence(UINT32_MAX-1500U);
    place_wait_retry(0);place_wait_retry(UINT32_MAX-1500U);
    for(uint8_t slot=1;slot<=3;++slot) {
        for(unsigned kind=WORK_BOMB;kind<WORK_COUNT;++kind) {
            observe_sequence((TaskWorkKind_t)kind,0,slot);
            observe_sequence((TaskWorkKind_t)kind,UINT32_MAX-1500U,slot);
        }
        target_sequence(0,slot);target_sequence(UINT32_MAX-1500U,slot);
        rescue_sequence(0,slot);rescue_sequence(UINT32_MAX-1500U,slot);
    }
    split_wait_retry_cancel(WORK_BOMB,0);
    split_wait_retry_cancel(WORK_TARGET,0);
    split_wait_retry_cancel(WORK_TARGET,UINT32_MAX-1500U);
    split_wait_retry_cancel(WORK_RESCUE,0);
    split_wait_retry_cancel(WORK_RESCUE,UINT32_MAX-1500U);
    /* Post-QR open only queues ID5, with no mechanical wait or arm-joint motion. */
    reset(0);
    assert(task_work_clamp_open_it()==RET_OK && sent==1 && tick==0);
    apply_frame(0,5,1);assert(current_positions[4]==CLAMP_OPEN_POSITION);
    uart_result=RET_BUSY;
    assert(task_work_clamp_open_it()==RET_BUSY && sent==1);
    uart_result=RET_ERROR;
    assert(task_work_clamp_open_it()==RET_ERROR && sent==1);
    /* QR observation/return retain their own geometry, independently of travel. */
    reset(0);
    run(WORK_QR,ARM_OBSERVE,0);apply_frame(0,1,4);expect_tcp(0,260,60,0);
    run(WORK_QR,ARM_TRAVEL,0);apply_frame(1,1,4);expect_tcp(0,260,60,0);
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
    puts("PASS: travel (0,200,140,0) IK/packets/FK; bomb place yaw then arm then open; observe/retract sequencing, clamp retention, timing/retry/cancel/rollover; QR unchanged");
    return 0;
}
