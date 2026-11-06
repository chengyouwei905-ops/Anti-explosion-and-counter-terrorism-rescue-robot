/* Exercise the real production configuration and work state machine. Reuse
 * UART/scene/arm fixtures; rename the port's arm hooks to keep them simulated. */
#define main task_fixture_main
#define task_work_load_config fixture_load_config
#include "test_task_vision.c"
#undef task_work_load_config
#undef main

#define task_work_load_config production_load_config
#define task_work_arm_poll production_arm_poll
#define task_work_arm_stop_it production_arm_stop_it
#define task_work_laser_set production_laser_set
#define task_work_clamp_open_it production_clamp_open_it
#include "../App/Src/app_task_work_port.c"
#undef task_work_load_config
#undef task_work_arm_poll
#undef task_work_arm_stop_it
#undef task_work_laser_set
#undef task_work_clamp_open_it

void HAL_GPIO_WritePin(void *port,uint16_t pin,GPIO_PinState state)
{ (void)port;(void)pin;(void)state; }

static bool use_production_config;
static bool override_yaw;
static int yaw_slot=-1;
static float yaw_limit,yaw_tolerance;
static bool override_scan_window;
static uint32_t scan_window;

bool task_work_load_config(TaskWorkKind_t kind,TaskWorkConfig_t *out)
{
    bool loaded=use_production_config ? production_load_config(kind,out) : fixture_load_config(kind,out);
    if(!loaded) return false;
    if(override_scan_window)out->scan_dwell_ms=scan_window;
    if(override_yaw) {
        out->max_correction_yaw=yaw_limit;
        out->yaw_tolerance_rad=yaw_tolerance;
        for(unsigned i=0;i<3;++i) {
            out->slot[i].alignment.yaw_mode=(int)i==yaw_slot ?
                (kind==WORK_QR ? VISION_YAW_QR_AXIS : VISION_YAW_TARGET_NORMAL) : VISION_YAW_NONE;
        }
    }
    return true;
}

static void until_config_phase(TaskWorkPhase_t phase,unsigned limit)
{
    unsigned start=elapsed;
    while(app_task_work_get_status()->phase!=phase && elapsed-start<limit && task_state!=TASK_FAULT) {
        const TaskWorkStatus_t *status=app_task_work_get_status();
        if(automatic_reply && reply_pending && elapsed-tx_time>=300U &&
           (!proto_vision_is_scan(tx_cmd) || status->phase==WORK_WAIT_SCAN)) {
            last_payload_len=make_scene_result(last_payload);
            deliver(VISION_CMD_RESULT,last_payload,last_payload_len);
            deliver(VISION_CMD_RESULT,last_payload,last_payload_len);
            reply_pending=false;
        }
        step(true);
    }
    if(app_task_work_get_status()->phase!=phase)
        printf("config phase wanted %d got %d err %d time %u\n",phase,
            app_task_work_get_status()->phase,app_task_work_get_status()->error,elapsed);
    assert(app_task_work_get_status()->phase==phase);
}

static void production_startup(uint32_t base)
{
    use_production_config=true;override_yaw=false;
    for(unsigned kind=0;kind<WORK_COUNT;++kind) {
        reset(base);automatic_reply=false;enter_work((TaskWorkKind_t)kind,false);
        until_config_phase(kind==WORK_QR ? WORK_WAIT_LOCATE : WORK_WAIT_SCAN,5000);
        assert(app_task_work_get_status()->error==WORK_ERR_NONE);
        const uint8_t commands[]={VISION_CMD_QR,VISION_CMD_BOMB_SCAN,
            VISION_CMD_TARGET_SCAN,VISION_CMD_RESCUE_SCAN};
        assert(send_count[commands[kind]]==1);
    }
}

static void scan_window_validation(void)
{
    use_production_config=true;override_yaw=false;override_scan_window=true;
    /* QR never sends a scan window: this unrelated field cannot prevent QR startup. */
    const uint32_t unused[]={0,249,12000,60000,60001,UINT32_MAX};
    for(unsigned i=0;i<sizeof(unused)/sizeof(unused[0]);++i) {
        scan_window=unused[i];reset(0);automatic_reply=false;enter_work(WORK_QR,false);
        until_config_phase(WORK_WAIT_LOCATE,5000);
        assert(task_running && send_count[VISION_CMD_QR]==1);
    }
    const uint32_t valid[]={250,10000,12000,60000};
    const uint32_t invalid[]={0,249,60001,65536};
    for(unsigned kind=WORK_BOMB;kind<WORK_COUNT;++kind) {
        for(unsigned i=0;i<sizeof(valid)/sizeof(valid[0]);++i) {
            scan_window=valid[i];reset(UINT32_MAX-1000U);automatic_reply=false;
            enter_work((TaskWorkKind_t)kind,false);
            until_config_phase(WORK_WAIT_SCAN,5000);
            assert(tx_window==scan_window && tx_slot==1 && task_running);
        }
        for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) {
            scan_window=invalid[i];reset(0);automatic_reply=false;
            enter_work((TaskWorkKind_t)kind,false);until_fault(1000);
            assert(app_task_work_get_status()->error==WORK_ERR_CONFIG);
        }
    }
    override_scan_window=false;
    /* Reproduce the reported path using the unmodified 12-second production config. */
    reset(0);automatic_reply=false;command("TOTALSTART!");
    until_config_phase(WORK_WAIT_LOCATE,10000);
    assert(task_state==TASK_SCAN && task_running && tx_cmd==VISION_CMD_QR);
}

static void unused_yaw_parameters(float limit,float tolerance,uint32_t base)
{
    use_production_config=false;override_yaw=true;yaw_slot=-1;
    yaw_limit=limit;yaw_tolerance=tolerance;
    reset(base);scene_slot=1;enter_work(WORK_TARGET,false);
    px+=60;py-=45;
    until_config_phase(WORK_WAIT_MOTION,5000);
    assert(app_task_work_get_status()->corrections==1);
    assert(app_task_work_get_status()->alignment.delta_yaw==0);
    until_config_phase(WORK_COMPLETE,8000);
    assert(app_task_work_get_status()->error==WORK_ERR_NONE);
    /* Slot 1 has no scan offset: keep the completed 75mm visual correction. */
    assert(hypotf(px-anchor[WORK_TARGET][0],py-anchor[WORK_TARGET][1])<3);
    assert(fabsf(yaw-anchor[WORK_TARGET][2])<.0001f && send_count[VISION_CMD_TARGET_LOCATE]==2);
}

static void enabled_yaw_validation(float limit,float tolerance)
{
    use_production_config=false;override_yaw=true;
    yaw_limit=limit;yaw_tolerance=tolerance;
    for(unsigned kind=WORK_QR;kind<=WORK_TARGET;kind+=2) {
        /* The third target slot also requires validation before scanning. */
        yaw_slot=kind==WORK_QR ? 0 : 2;
        reset(0);automatic_reply=false;enter_work((TaskWorkKind_t)kind,false);
        until_fault(1000);
        assert(app_task_work_get_status()->error==WORK_ERR_CONFIG);
        assert(send_count[VISION_CMD_QR]==0 && send_count[VISION_CMD_TARGET_SCAN]==0);
    }
}

static void enabled_yaw_correction(void)
{
    use_production_config=false;override_yaw=true;yaw_slot=0;
    yaw_limit=.174532925f;yaw_tolerance=.008726646f;
    reset(0);scene_slot=1;enter_work(WORK_TARGET,true);
    until_config_phase(WORK_WAIT_MOTION,5000);
    assert(fabsf(app_task_work_get_status()->alignment.delta_yaw)>.05f);
    until_config_phase(WORK_LASER,15000);
    assert(app_task_work_get_status()->error==WORK_ERR_NONE);
    assert(fabsf(yaw-anchor[WORK_TARGET][2])<=yaw_tolerance);
}

static void variable_measurement_samples(void)
{
    use_production_config=false;override_yaw=false;
    const uint8_t counts[]={3,5,32};
    const Task_State_t next[]={TASK_MOVE2,TASK_MOVE12,TASK_MOVE13,TASK_MOVE16};
    for(unsigned kind=WORK_BOMB;kind<WORK_COUNT;++kind) {
        for(unsigned i=0;i<sizeof(counts)/sizeof(counts[0]);++i) {
            reset(i==1 ? UINT32_MAX-1000U : 0);
            scene_samples=counts[i];enter_work((TaskWorkKind_t)kind,false);
            until_config_phase(WORK_COMPLETE,40000);
            assert(app_task_work_get_status()->error==WORK_ERR_NONE);
            assert(last_payload[7]==counts[i]);
            step(true);
            assert(task_state==next[kind] && app_chassis_motion_get_state()==MOTION_STATE_RUNNING);
        }
    }
    const uint8_t rejected[]={1,2,33,255};
    for(unsigned i=0;i<sizeof(rejected)/sizeof(rejected[0]);++i) {
        reset(0);scene_slot=1;enter_work(WORK_TARGET,false);
        until_config_phase(WORK_WAIT_LOCATE,10000);automatic_reply=false;
        uint8_t payload[64];uint8_t length=make_scene_result(payload);
        payload[7]=rejected[i];deliver(VISION_CMD_RESULT,payload,length);
        until_config_phase(WORK_RETRY_VISION,20);
        assert(task_running && app_task_work_get_status()->last_retry_error==WORK_ERR_DATA);
        automatic_reply=true;
        until_config_phase(WORK_COMPLETE,5000);
    }
}

static void send_target_height_result(float height_error,float x_error)
{
    TaskWorkConfig_t cfg;assert(task_work_load_config(WORK_TARGET,&cfg));
    const VisionAlignmentConfig_t *a=&cfg.slot[0].alignment;
    uint8_t payload[64];result_header(payload,tx_cmd,tx_seq,VISION_SUCCESS,0);
    payload[4]=VISION_POSITION|VISION_CONTROL_READY;payload[6]=1;payload[7]=5;
    for(unsigned i=0;i<3;++i) {
        float value=0;
        for(unsigned j=0;j<3;++j)
            value+=a->body_from_tcp.rotation[j][i]*(a->desired_reference_body[j]+
                (j==0?x_error:j==2?height_error:0)-a->body_from_tcp.translation[j]);
        put_float(payload+15+4*i,value);
    }
    put_float(payload+27,.5f);
    deliver(VISION_CMD_RESULT,payload,31);
}

static void production_target_height(float height_error,bool correct_xy,uint32_t base)
{
    use_production_config=true;override_yaw=false;
    reset(base);scene_slot=1;automatic_reply=false;enter_work(WORK_TARGET,false);
    until_config_phase(WORK_WAIT_SCAN,5000);
    uint8_t scan[64];result_header(scan,tx_cmd,tx_seq,VISION_SUCCESS,0);scan[6]=1;
    deliver(VISION_CMD_RESULT,scan,15);
    until_config_phase(WORK_WAIT_LOCATE,2000);
    send_target_height_result(height_error,correct_xy?40.0f:0.0f);
    if(correct_xy) {
        until_config_phase(WORK_WAIT_MOTION,30);
        until_config_phase(WORK_WAIT_LOCATE,8000);
        send_target_height_result(height_error,0);
    }
    until_config_phase(WORK_LASER,30);
    const TaskWorkStatus_t *status=app_task_work_get_status();
    assert(laser && status->error==WORK_ERR_NONE && status->vision_retries==0);
    assert(status->height_out_of_tolerance && fabsf(status->height_error_mm-height_error)<.01f);
    assert(status->corrections==(correct_xy?1U:0U));
    until_config_phase(WORK_COMPLETE,5000);
    step(true);assert(task_state==TASK_MOVE13);
    use_production_config=false;
}

int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    assert(TASK_WORK_CAMERA_TEST==0);
    production_startup(0);production_startup(UINT32_MAX-1000U);
    scan_window_validation();
    const float invalid_yaw[][2]={
        {.174532925f*20,.08726646f}, {0,0}, {-1,-1}, {NAN,NAN}, {INFINITY,INFINITY},
        {.174532925f,0}, {.174532925f,-1}, {.174532925f,NAN}, {.174532925f,.174532925f}
    };
    for(unsigned i=0;i<sizeof(invalid_yaw)/sizeof(invalid_yaw[0]);++i) {
        unused_yaw_parameters(invalid_yaw[i][0],invalid_yaw[i][1],0);
        unused_yaw_parameters(invalid_yaw[i][0],invalid_yaw[i][1],UINT32_MAX-1000U);
        enabled_yaw_validation(invalid_yaw[i][0],invalid_yaw[i][1]);
    }
    enabled_yaw_correction();
    variable_measurement_samples();
    const float height_errors[]={-200,-60,60,200};
    for(unsigned i=0;i<sizeof(height_errors)/sizeof(height_errors[0]);++i)
        for(unsigned correct=0;correct<2;++correct) {
            production_target_height(height_errors[i],correct!=0,0);
            production_target_height(height_errors[i],correct!=0,UINT32_MAX-1000U);
        }
    puts("PASS: 16 production target-height cases reach XY correction/laser/next MOVE beyond the old Z range; height diagnostics retained");
    puts("PASS: production configuration reaches QR/scan commands, unused yaw ignored during translation/retest, enabled yaw validation/correction retained, tick rollover");
    return 0;
}
