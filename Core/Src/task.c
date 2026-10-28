#include "task.h"
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static Task_State_t task_state = TASK_IDLE;
static bool task_total_start_requested;
static float task_offset_x, task_offset_y;
static bool task_offset_pending;

/* Map origin: lower left. Start: (3350,1750), facing left.
 * Smooth turns preserve nominal circular endpoints; radius varies during ramps.
 */
#define ROUTE_LINE(d,v,a,b) {ALG_PATH_LINE,(d),0.0f,0.0f,(v),(a),(b)}
#define ROUTE_TURN(angle,v,a,b) {ALG_PATH_ARC,0.0f,200.0f,(angle),(v),(a),(b)}
#define ROUTE_HALF_PI 1.57079632679f
static const AlgPathSegment_t task_route[] = {
    ROUTE_LINE(450.0f,250.0f,300.0f,300.0f),             /* MOVE1: scan */
    ROUTE_TURN( ROUTE_HALF_PI,140.0f,200.0f,250.0f),    /* MOVE2: left */
    ROUTE_LINE(305.0f,180.0f,200.0f,250.0f),             /* MOVE3 */
    ROUTE_TURN(-ROUTE_HALF_PI,100.0f,150.0f,150.0f),    /* MOVE4: right */
    ROUTE_LINE(550.0f,100.0f,150.0f,150.0f),             /* MOVE5: bumps */
    ROUTE_TURN( ROUTE_HALF_PI,100.0f,150.0f,150.0f),    /* MOVE6: left */
    ROUTE_LINE(455.0f,220.0f,250.0f,300.0f),             /* MOVE7 */
    ROUTE_TURN(-ROUTE_HALF_PI,140.0f,200.0f,250.0f),    /* MOVE8: right */
    ROUTE_LINE(580.0f,250.0f,300.0f,350.0f),             /* MOVE9 */
    ROUTE_TURN(-ROUTE_HALF_PI,140.0f,200.0f,250.0f),    /* MOVE10: right */
    ROUTE_LINE(505.0f,200.0f,250.0f,300.0f),             /* MOVE11: explosive */
    ROUTE_LINE(1730.0f,300.0f,350.0f,400.0f),            /* MOVE12: targets */
    ROUTE_LINE(375.0f,220.0f,250.0f,300.0f),             /* MOVE13 */
    ROUTE_TURN(-ROUTE_HALF_PI,140.0f,200.0f,250.0f),    /* MOVE14: right */
    ROUTE_LINE(780.0f,220.0f,250.0f,300.0f),             /* MOVE15: rescue */
    ROUTE_LINE(1600.0f,250.0f,250.0f,300.0f)             /* MOVE16: return */
};
#undef ROUTE_LINE
#undef ROUTE_TURN
#undef ROUTE_HALF_PI

static const Task_State_t task_move_state[] = {
    TASK_MOVE1,TASK_MOVE2,TASK_MOVE3,TASK_MOVE4,
    TASK_MOVE5,TASK_MOVE6,TASK_MOVE7,TASK_MOVE8,
    TASK_MOVE9,TASK_MOVE10,TASK_MOVE11,TASK_MOVE12,
    TASK_MOVE13,TASK_MOVE14,TASK_MOVE15,TASK_MOVE16
};
_Static_assert(sizeof(task_route)/sizeof(task_route[0]) ==
               sizeof(task_move_state)/sizeof(task_move_state[0]), "route state count mismatch");
static const AlgPathLimits_t task_path_limits = {
    .transition_mm=30.0f,
    .max_omega=0.9f, /* reserve headroom below the yaw loop limit of 1.2 rad/s */
    .max_angular_accel=2.0f,
    .max_lateral_accel=400.0f
};
typedef struct { uint8_t first,count; Task_State_t next; } Task_Path_Group_t;
static const Task_Path_Group_t task_groups[] = {
    {0U,1U,TASK_SCAN}, {1U,10U,TASK_EXPLOSIVE}, {11U,1U,TASK_CS},
    {12U,3U,TASK_RESCUE}, {15U,1U,TASK_END}
};
#define TASK_STOP_DWELL_MS 1000U
#define TASK_WORK_PLACEHOLDER_MS 1000U
#define TASK_TIMEOUT_MARGIN_MS 2000U
static uint8_t task_group;
static bool task_path_active,task_settling;
static uint32_t task_enter_ms,task_path_start_ms,task_settle_ms,task_path_timeout_ms;

void task_vision_on_frame(const proto_vision_frame_t *frame)
{
    static const char total_start_cmd[] = "TOTALSTART!";
    static const char total_stop_cmd[] = "TOTALSTOP!";
    static const char offset_prefix[] = "OFFSET,X=";

    if ((frame == NULL) || (frame->len > PROTO_VISION_MAX_PAYLOAD)) {
        return;
    }

    /* payload 不是 C 字符串：先匹配完整长度，再比较字节，避免越界或前缀误匹配。
     * 暂按 payload 分派指令；后续约定 CMD 含义后可在此增加 frame->cmd 筛选。
     */
    if ((frame->len == sizeof(total_start_cmd) - 1U) &&
        (memcmp(frame->payload, total_start_cmd, sizeof(total_start_cmd) - 1U) == 0)) {
        task_total_start_requested = true;
        return;
    }
    if ((frame->len == sizeof(total_stop_cmd) - 1U) &&
        (memcmp(frame->payload, total_stop_cmd, sizeof(total_stop_cmd) - 1U) == 0)) {
        task_total_start_requested = false;
        task_state = TASK_IDLE;
        return;
    }
    if ((frame->len >= sizeof(offset_prefix) - 1U) &&
        (memcmp(frame->payload, offset_prefix, sizeof(offset_prefix) - 1U) == 0)) {
        char text[PROTO_VISION_MAX_PAYLOAD + 1U];
        char *p;
        char *end;
        float x;
        float y;
        /* payload 不带 NUL：复制到本地缓冲后补终止符，供 strtof() 安全读取。 */
        memcpy(text, frame->payload, frame->len);
        text[frame->len] = '\0';
        p = text + sizeof(offset_prefix) - 1U;
        errno = 0;
        x = strtof(p, &end);
        if ((end == p) || (errno == ERANGE) || !isfinite(x) ||
            (strncmp(end, ",Y=", 3U) != 0)) {
            return;
        }
        p = end + 3U;
        errno = 0;
        y = strtof(p, &end);
        if ((end == p) || (errno == ERANGE) || !isfinite(y) ||
            (end != text + frame->len)) {
            return;
        }
        /* 整条指令校验通过后再一起更新，避免只更新 X 或误接收尾部垃圾。 */
        task_offset_x = x;
        task_offset_y = y;
        task_offset_pending = true;
        return;
    }
    /* 后续指令在这里继续添加 else if；未知帧不修改已收到的事件。 */
}

static void task_enter(Task_State_t state,uint32_t now)
{
    task_state=state;
    task_enter_ms=now;
}
static void task_fault(uint32_t now)
{
    app_chassis_motion_stop();
    task_path_active=task_settling=false;
    task_enter(TASK_FAULT,now);
}
static void task_start_group(uint8_t group,uint32_t now)
{
    if(group>=sizeof(task_groups)/sizeof(task_groups[0])) {
        task_fault(now);
        return;
    }
    const Task_Path_Group_t *path=&task_groups[group];
    if (path->count == 0U || (unsigned)path->first + path->count >
        sizeof(task_route)/sizeof(task_route[0])) {
        task_fault(now);
        return;
    }
    Chassis_Motion_Start_t result=app_chassis_motion_path_start(
        &task_route[path->first],path->count,&task_path_limits);
    AlgPathSample_t sample;
    if(result!=MOTION_START_OK || !app_chassis_motion_path_get_sample(&sample)) {
        task_fault(now);
        return;
    }
    task_group=group;
    task_path_active=true;
    task_settling=false;
    task_path_start_ms=now;
    task_path_timeout_ms=(uint32_t)ceilf(sample.duration*1000.0f)+TASK_TIMEOUT_MARGIN_MS;
    task_enter(task_move_state[path->first],now);
}
void task_init(void)
{
    task_state=TASK_IDLE;
    task_total_start_requested=false;
    task_offset_x=task_offset_y=0.0f;
    task_offset_pending=false;
    task_path_active=task_settling=false;
    task_enter_ms=HAL_GetTick();
    app_vision_set_rx_cb(task_vision_on_frame);
}
void task_poll(void)
{
    uint32_t now=HAL_GetTick();
    if(task_state==TASK_IDLE) {
        task_path_active=task_settling=false;
        if(task_total_start_requested) {
            task_total_start_requested=false;
            task_start_group(0U,now);
        }
    } else if(task_path_active) {
        Chassis_Motion_Result_t result=app_chassis_motion_get_result();
        const Task_Path_Group_t *group=&task_groups[task_group];
        if(result==MOTION_RESULT_RUNNING) {
            AlgPathSample_t sample;
            if(!app_chassis_motion_path_get_sample(&sample) || sample.segment>=group->count ||
               (uint32_t)(now-task_path_start_ms)>task_path_timeout_ms) {
                task_fault(now);
            } else {
                task_state=task_move_state[group->first+sample.segment];
            }
        } else if(result==MOTION_RESULT_DONE) {
            task_state=task_move_state[group->first+group->count-1U];
            if(!task_settling) {
                task_settling=true;
                task_settle_ms=now;
            }
            if((uint32_t)(now-task_settle_ms)>=TASK_STOP_DWELL_MS) {
                task_path_active=task_settling=false;
                task_enter(group->next,now);
            }
        } else {
            /* Cancellation / IMU failure does not mean a successful arrival. */
            task_fault(now);
        }
    } else if((uint32_t)(now-task_enter_ms)>=TASK_WORK_PLACEHOLDER_MS) {
        switch(task_state) {
        case TASK_SCAN:      task_start_group(1U,now); break;
        case TASK_EXPLOSIVE: task_start_group(2U,now); break;
        case TASK_CS:        task_start_group(3U,now); break;
        case TASK_RESCUE:    task_start_group(4U,now); break;
        default: break; /* END / FAULT wait for an explicit new lifecycle. */
        }
    }
    task_total_start_requested=false; /* ignore duplicate starts while running */
}
Task_State_t task_get_state(void)
{
    return task_state;
}
