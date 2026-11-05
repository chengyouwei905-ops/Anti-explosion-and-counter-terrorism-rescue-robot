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
#include "../App/Src/app_task_work_port.c"
#undef task_work_load_config
#undef task_work_arm_poll
#undef task_work_arm_stop_it
#undef task_work_laser_set

void HAL_GPIO_WritePin(void *port,uint16_t pin,GPIO_PinState state)
{ (void)port;(void)pin;(void)state; }

static bool use_production_config;
static bool override_yaw;
static int yaw_slot=-1;
static float yaw_limit,yaw_tolerance;

bool task_work_load_config(TaskWorkKind_t kind,TaskWorkConfig_t *out)
{
    bool loaded=use_production_config ? production_load_config(kind,out) : fixture_load_config(kind,out);
    if(!loaded) return false;
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

static void unused_yaw_parameters(float limit,float tolerance,uint32_t base)
{
    use_production_config=false;override_yaw=true;yaw_slot=-1;
    yaw_limit=limit;yaw_tolerance=tolerance;
    reset(base);enter_work(WORK_QR,false);
    px+=60;py-=45;
    until_config_phase(WORK_WAIT_MOTION,4000);
    assert(app_task_work_get_status()->corrections==1);
    assert(app_task_work_get_status()->alignment.delta_yaw==0);
    until_config_phase(WORK_COMPLETE,8000);
    assert(app_task_work_get_status()->error==WORK_ERR_NONE);
    assert(hypotf(px-anchor[WORK_QR][0],py-anchor[WORK_QR][1])<=20);
    assert(fabsf(yaw-anchor[WORK_QR][2])<.0001f && send_count[VISION_CMD_QR]==2);
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
    reset(0);enter_work(WORK_QR,true);
    until_config_phase(WORK_WAIT_MOTION,5000);
    assert(fabsf(app_task_work_get_status()->alignment.delta_yaw)>.05f);
    until_config_phase(WORK_COMPLETE,15000);
    assert(app_task_work_get_status()->error==WORK_ERR_NONE);
    assert(fabsf(yaw-anchor[WORK_QR][2])<=yaw_tolerance);
}

int main(void)
{
    setvbuf(stdout,NULL,_IONBF,0);
    assert(TASK_WORK_CAMERA_TEST==0);
    production_startup(0);production_startup(UINT32_MAX-1000U);
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
    puts("PASS: production configuration reaches QR/scan commands, unused yaw ignored during translation/retest, enabled yaw validation/correction retained, tick rollover");
    return 0;
}
