#include "task.h"
#include "app_task_work.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static Task_State_t task_state = TASK_IDLE;
static bool task_total_start_requested;
static bool task_total_stop_requested;
static bool task_running;

/* 赛道坐标原点在左下角，起点为 (3350,1750)，车头朝左。
 * 距离单位 mm，速度单位 mm/s，加减速度单位 mm/s^2，角度单位 rad。
 * 正角度左转，负角度右转。平滑转弯仍保留原圆弧的终点和总转角。
 *
 * 连续运动的分组：
 * MOVE1           -> 扫码位置停车
 * MOVE2 ~ MOVE11  -> 排爆位置停车（这十段在底层连续执行）
 * MOVE12          -> 反恐位置停车
 * MOVE13 ~ MOVE15 -> 救援位置停车（这三段在底层连续执行）
 * MOVE16          -> 终点停车
 * task_poll() 仍逐个列出 MOVE 状态；同组内切换状态时不重新下发运动。
 */
/* 每段参数单独展开，调车时直接修改对应 MOVE 的字段即可。
 * max_speed 是速度上限，实际转弯速度还受曲率和角加速度限制。
 */
static const AlgPathSegment_t task_route[] = {
    {   /* MOVE1：直行 450 mm */
        .type = ALG_PATH_LINE,
        .distance = 450.0f,
        .max_speed = 250.0f,
        .accel = 300.0f,
        .decel = 300.0f
    },
    {   /* MOVE2：左转 90 度 */
        .type = ALG_PATH_ARC,
        .radius = 200.0f,
        .angle = 1.57079632679f,
        .max_speed = 140.0f,
        .accel = 200.0f,
        .decel = 250.0f
    },
    {   /* MOVE3：直行 305 mm */
        .type = ALG_PATH_LINE,
        .distance = 305.0f,
        .max_speed = 180.0f,
        .accel = 200.0f,
        .decel = 250.0f
    },
    {   /* MOVE4：右转 90 度 */
        .type = ALG_PATH_ARC,
        .radius = 200.0f,
        .angle = -1.57079632679f,
        .max_speed = 100.0f,
        .accel = 150.0f,
        .decel = 150.0f
    },
    {   /* MOVE5：直行 550 mm */
        .type = ALG_PATH_LINE,
        .distance = 550.0f,
        .max_speed = 100.0f,
        .accel = 150.0f,
        .decel = 150.0f
    },
    {   /* MOVE6：左转 90 度 */
        .type = ALG_PATH_ARC,
        .radius = 200.0f,
        .angle = 1.57079632679f,
        .max_speed = 100.0f,
        .accel = 150.0f,
        .decel = 150.0f
    },
    {   /* MOVE7：直行 455 mm */
        .type = ALG_PATH_LINE,
        .distance = 455.0f,
        .max_speed = 220.0f,
        .accel = 250.0f,
        .decel = 300.0f
    },
    {   /* MOVE8：右转 90 度 */
        .type = ALG_PATH_ARC,
        .radius = 200.0f,
        .angle = -1.57079632679f,
        .max_speed = 140.0f,
        .accel = 200.0f,
        .decel = 250.0f
    },
    {   /* MOVE9：直行 580 mm */
        .type = ALG_PATH_LINE,
        .distance = 580.0f,
        .max_speed = 250.0f,
        .accel = 300.0f,
        .decel = 350.0f
    },
    {   /* MOVE10：右转 90 度 */
        .type = ALG_PATH_ARC,
        .radius = 200.0f,
        .angle = -1.57079632679f,
        .max_speed = 140.0f,
        .accel = 200.0f,
        .decel = 250.0f
    },
    {   /* MOVE11：直行 505 mm */
        .type = ALG_PATH_LINE,
        .distance = 505.0f,
        .max_speed = 200.0f,
        .accel = 250.0f,
        .decel = 300.0f
    },
    {   /* MOVE12：直行 1730 mm */
        .type = ALG_PATH_LINE,
        .distance = 1730.0f,
        .max_speed = 300.0f,
        .accel = 350.0f,
        .decel = 400.0f
    },
    {   /* MOVE13：直行 375 mm */
        .type = ALG_PATH_LINE,
        .distance = 375.0f,
        .max_speed = 220.0f,
        .accel = 250.0f,
        .decel = 300.0f
    },
    {   /* MOVE14：右转 90 度 */
        .type = ALG_PATH_ARC,
        .radius = 200.0f,
        .angle = -1.57079632679f,
        .max_speed = 140.0f,
        .accel = 200.0f,
        .decel = 250.0f
    },
    {   /* MOVE15：直行 780 mm */
        .type = ALG_PATH_LINE,
        .distance = 780.0f,
        .max_speed = 220.0f,
        .accel = 250.0f,
        .decel = 300.0f
    },
    {   /* MOVE16：直行 1600 mm */
        .type = ALG_PATH_LINE,
        .distance = 1600.0f,
        .max_speed = 250.0f,
        .accel = 250.0f,
        .decel = 300.0f
    }
};

static const Task_State_t task_move_state[] = {
    TASK_MOVE1,TASK_MOVE2,TASK_MOVE3,TASK_MOVE4,
    TASK_MOVE5,TASK_MOVE6,TASK_MOVE7,TASK_MOVE8,
    TASK_MOVE9,TASK_MOVE10,TASK_MOVE11,TASK_MOVE12,
    TASK_MOVE13,TASK_MOVE14,TASK_MOVE15,TASK_MOVE16
};
_Static_assert(sizeof(task_route)/sizeof(task_route[0]) ==
               sizeof(task_move_state)/sizeof(task_move_state[0]), "route state count mismatch");
static const AlgPathLimits_t task_path_limits = {
    .transition_mm = 30.0f,        /* 转弯入口和出口的曲率过渡长度参数 */
    .max_omega = 0.9f,            /* rad/s，给航向闭环的 1.2 rad/s 限幅留余量 */
    .max_angular_accel = 2.0f,    /* rad/s^2 */
    .max_lateral_accel = 400.0f   /* mm/s^2 */
};

/* 每组路线结束后停稳 1 秒，再进入对应作业流程。 */
#define TASK_STOP_DWELL_MS 1000U
#define TASK_TIMEOUT_MARGIN_MS 2000U
/* 这些变量只负责一组连续运动，不涉及扫码、机械臂或视觉纠偏。 */
static uint8_t task_first_segment;       /* 当前组在 task_route 中的起始下标 */
static uint8_t task_segment_count;       /* 当前组包含几段移动 */
static bool task_path_active;            /* 已启动路径，不能每次 poll 都重新启动 */
static bool task_settling;               /* 已走完路径，正在等待停稳 */
static uint32_t task_path_start_ms;
static uint32_t task_settle_ms;
static uint32_t task_path_timeout_ms;

void task_vision_on_frame(const proto_vision_frame_t *frame)
{
    static const char total_start_cmd[] = "TOTALSTART!";
    static const char total_stop_cmd[] = "TOTALSTOP!";

    if ((frame == NULL) || (frame->len > PROTO_VISION_MAX_PAYLOAD)) {
        return;
    }

    /* 0x81 是二进制任务结果，原请求 SEQ 在 payload[2]，不是 frame->seq。
     * 仅复制匹配的事件；实际坐标换算和运动由下面的 task_poll 驱动。
     */
    if (frame->cmd == VISION_CMD_RESULT) {
        if (task_running && !task_total_stop_requested) {
            app_task_work_on_frame(frame);
        }
        return;
    }
    if (frame->cmd != PROTO_VISION_CMD_STATUS) {
        return;
    }
    /* 启停指令严格匹配 CMD、长度、字节，不按未终止字符串访问 payload。 */
    if ((frame->len == sizeof(total_start_cmd) - 1U) &&
        (memcmp(frame->payload, total_start_cmd, sizeof(total_start_cmd) - 1U) == 0)) {
        if (task_state == TASK_IDLE && !task_total_stop_requested) {
            task_total_start_requested = true;
        }
        return;
    }
    if ((frame->len == sizeof(total_stop_cmd) - 1U) &&
        (memcmp(frame->payload, total_stop_cmd, sizeof(total_stop_cmd) - 1U) == 0)) {
        task_total_start_requested = false;
        task_total_stop_requested = true;
        task_running = false;
        /* 先失效所有等待/成功事件，保证同一批串口数据中的晚到结果也无效。 */
        app_task_work_cancel(HAL_GetTick());
        return;
    }
    /* 旧 OFFSET 文本不再作为控制来源，坐标只接受文档约定的二进制结果。 */
}

/* 运动/作业异常时结束本轮任务，不能把取消或 IMU 失效当作到达作业区。 */
static void task_fault(void)
{
    task_running = false;
    task_total_start_requested = false;
    if (app_task_work_get_status()->phase != WORK_ERROR &&
        !app_task_work_get_status()->cancel_failed) {
        app_task_work_cancel(HAL_GetTick());
    }
    app_chassis_closeloop_init();
    app_chassis_motion_stop();
    task_path_active = false;
    task_settling = false;
    task_state = TASK_FAULT;
}

/* 启动一组连续移动，只在这组的入口调用一次。
 * first_move 使用 MOVE 编号（从 1 开始），count 是这组包含的移动段数。
 * 例如 task_start_path(2U, 10U, now) 会连续执行 MOVE2 ~ MOVE11。
 */
static void task_start_path(uint8_t first_move, uint8_t count, uint32_t now)
{
    AlgPathSample_t sample;
    Chassis_Motion_Start_t result;
    unsigned route_count = sizeof(task_route) / sizeof(task_route[0]);

    if (first_move == 0U || count == 0U ||
        (unsigned)first_move - 1U + count > route_count) {
        task_fault();
        return;
    }

    uint8_t first_segment = first_move - 1U;
    result = app_chassis_motion_path_start(&task_route[first_segment],
                                          count, &task_path_limits);
    if (result != MOTION_START_OK) {
        task_fault();
        return;
    }
    if (!app_chassis_motion_path_get_sample(&sample)) {
        task_fault();
        return;
    }

    task_first_segment = first_segment;
    task_segment_count = count;
    task_path_active = true;
    task_settling = false;
    task_path_start_ms = now;
    task_path_timeout_ms = (uint32_t)ceilf(sample.duration * 1000.0f)
                          + TASK_TIMEOUT_MARGIN_MS;
    task_state = task_move_state[first_segment];
}

/* 判断某个 MOVE 是否可以进入下一个状态，不等待、不重新下发运动。
 * 组内的中间段：底层进入后一段即可切换，底盘保持连续运动。
 * 组内的最后段：整组轨迹结束并停稳 1 秒后，才允许进入作业状态。
 * 这里的完成是参考轨迹完成，尚未接入实际位置到位判断。
 */
static bool task_move_finished(uint8_t move_number, uint32_t now)
{
    AlgPathSample_t sample;
    Chassis_Motion_Result_t result;
    uint8_t last_move = task_first_segment + task_segment_count;

    if (!task_path_active || move_number <= task_first_segment ||
        move_number > last_move) {
        task_fault();
        return false;
    }

    result = app_chassis_motion_get_result();
    if (result != MOTION_RESULT_RUNNING && result != MOTION_RESULT_DONE) {
        task_fault();
        return false;
    }
    if (!app_chassis_motion_path_get_sample(&sample) ||
        sample.segment >= task_segment_count) {
        task_fault();
        return false;
    }

    if (result == MOTION_RESULT_RUNNING) {
        /* 超时只负责发现控制停滞，不用固定时间强行推进路线。 */
        if ((uint32_t)(now - task_path_start_ms) > task_path_timeout_ms) {
            task_fault();
            return false;
        }

        uint8_t current_move = task_first_segment + sample.segment + 1U;
        if (current_move > move_number) {
            return true;
        }
        return false;
    }

    /* 从首次观察到轨迹完成开始计时；无 HAL_Delay，不影响其他 poll。 */
    if (!task_settling) {
        task_settling = true;
        task_settle_ms = now;
    }

    /* 若 task_poll 暂时错过了段边界，逐个补上状态，不重启已走过的路径。 */
    if (move_number < last_move) {
        return true;
    }
    if ((uint32_t)(now - task_settle_ms) < TASK_STOP_DWELL_MS) {
        return false;
    }

    task_path_active = false;
    task_settling = false;
    return true;
}

void task_init(void)
{
    task_state = TASK_IDLE;
    task_total_start_requested = false;
    task_total_stop_requested = false;
    task_running = false;
    app_task_work_init();
    task_first_segment = 0U;
    task_segment_count = 0U;
    task_path_active = false;
    task_settling = false;
    task_path_start_ms = 0U;
    task_settle_ms = 0U;
    task_path_timeout_ms = 0U;
    app_vision_set_rx_cb(task_vision_on_frame);
}

void task_poll(void)
{
    uint32_t now = HAL_GetTick();

    /* 总停优先于任何移动/作业状态。激光在回调里已关，下面取消实际底盘运动。
     * 清 PID 后保持当前航向，避免旧误差的微分项在停车时继续输出转速。
     */
    if (task_total_stop_requested) {
        task_total_stop_requested = false;
        task_total_start_requested = false;
        app_chassis_closeloop_init();
        app_chassis_motion_stop();
        task_path_active = false;
        task_settling = false;
        task_state = TASK_IDLE;
    }
    app_task_work_service(now);

    switch (task_state) {
    case TASK_IDLE:
        task_path_active = false;
        task_settling = false;
        if (task_total_start_requested) {
            if (app_task_work_cancel_pending()) {
                break; /* 上一次的取消发送未完成，不能让它取消新一轮视觉任务 */
            }
            if (app_task_work_get_status()->cancel_failed) {
                task_fault();
                break;
            }
            task_total_start_requested = false;
            task_running = true;
            app_task_work_init();
            /* 第一组：从出发区直行到扫码位置。 */
            task_start_path(1U, 1U, now);
        }
        break;

    case TASK_MOVE1:
        /* 直行 450 mm；到扫码位置后停稳 1 秒。 */
        if (task_move_finished(1U, now)) {
            task_state = TASK_SCAN;
        }
        break;

    case TASK_SCAN: {
        /* 观察姿态 -> 发 0x10 -> 等位姿 -> 底盘微调并复测 -> 收臂。 */
        TaskWorkResult_t result = app_task_work_poll(WORK_QR, now);
        //result = WORK_DONE;     //测试时暂时屏蔽
        if (result == WORK_DONE) {
            task_start_path(2U, 10U, now);
        } else if (result == WORK_FAILED) {
            task_fault();
        }
        break;
    }

    case TASK_MOVE2:
        /* 左转 90 度，转弯名义半径 200 mm。 */
        if (task_move_finished(2U, now)) {
            task_state = TASK_MOVE3;
        }
        break;

    case TASK_MOVE3:
        /* 直行 305 mm，接近下方减速带入口。 */
        if (task_move_finished(3U, now)) {
            task_state = TASK_MOVE4;
        }
        break;

    case TASK_MOVE4:
        /* 右转 90 度，进入下方减速带方向。 */
        if (task_move_finished(4U, now)) {
            task_state = TASK_MOVE5;
        }
        break;

    case TASK_MOVE5:
        /* 以较低速度直行 550 mm，通过减速带。 */
        if (task_move_finished(5U, now)) {
            task_state = TASK_MOVE6;
        }
        break;

    case TASK_MOVE6:
        /* 左转 90 度，朝底部回廊行驶。 */
        if (task_move_finished(6U, now)) {
            task_state = TASK_MOVE7;
        }
        break;

    case TASK_MOVE7:
        /* 直行 455 mm。 */
        if (task_move_finished(7U, now)) {
            task_state = TASK_MOVE8;
        }
        break;

    case TASK_MOVE8:
        /* 右转 90 度，进入底部水平路段。 */
        if (task_move_finished(8U, now)) {
            task_state = TASK_MOVE9;
        }
        break;

    case TASK_MOVE9:
        /* 沿底部回廊直行 580 mm。 */
        if (task_move_finished(9U, now)) {
            task_state = TASK_MOVE10;
        }
        break;

    case TASK_MOVE10:
        /* 右转 90 度，朝排爆区行驶。 */
        if (task_move_finished(10U, now)) {
            task_state = TASK_MOVE11;
        }
        break;

    case TASK_MOVE11:
        /* 直行 505 mm；到排爆位置后停稳 1 秒。 */
        if (task_move_finished(11U, now)) {
            task_state = TASK_EXPLOSIVE;
        }
        break;

    case TASK_EXPLOSIVE: {
        /* 发 0x20 扫描三处 -> 按序号选位 -> 发 0x21 -> 微调 -> 夹取/放置。 */
        TaskWorkResult_t result = app_task_work_poll(WORK_BOMB, now);
        if (result == WORK_DONE) {
            task_start_path(12U, 1U, now);
        } else if (result == WORK_FAILED) {
            task_fault();
        }
        break;
    }

    case TASK_MOVE12:
        /* 直行 1730 mm；到反恐位置后停稳 1 秒。 */
        if (task_move_finished(12U, now)) {
            task_state = TASK_CS;
        }
        break;

    case TASK_CS: {
        /* 发 0x30/0x31 -> 微调并复测对准 -> PA0 高电平照射 -> 关闭激光。 */
        TaskWorkResult_t result = app_task_work_poll(WORK_TARGET, now);
        if (result == WORK_DONE) {
            task_start_path(13U, 3U, now);
        } else if (result == WORK_FAILED) {
            task_fault();
        }
        break;
    }

    case TASK_MOVE13:
        /* 直行 375 mm，接近顶部转弯位置。 */
        if (task_move_finished(13U, now)) {
            task_state = TASK_MOVE14;
        }
        break;

    case TASK_MOVE14:
        /* 右转 90 度，进入顶部水平路段。 */
        if (task_move_finished(14U, now)) {
            task_state = TASK_MOVE15;
        }
        break;

    case TASK_MOVE15:
        /* 直行 780 mm；到救援位置后停稳 1 秒。 */
        if (task_move_finished(15U, now)) {
            task_state = TASK_RESCUE;
        }
        break;

    case TASK_RESCUE: {
        /* 预留 0x40/0x41；模型未就绪 reason=9 时停止，不假装作业成功。 */
        TaskWorkResult_t result = app_task_work_poll(WORK_RESCUE, now);
        if (result == WORK_DONE) {
            task_start_path(16U, 1U, now);
        } else if (result == WORK_FAILED) {
            task_fault();
        }
        break;
    }

    case TASK_MOVE16:
        /* 直行 1600 mm；到终点后停稳 1 秒。 */
        if (task_move_finished(16U, now)) {
            task_state = TASK_END;
        }
        break;

    case TASK_END:
        /* 本轮任务结束，保持停车。 */
        task_running = false;
        break;

    case TASK_FAULT:
        /* 本轮任务已因异常停止，不自动继续下一段。 */
        break;

    default:
        task_fault();
        break;
    }

    /* 运行途中收到重复启动帧时，不重新开始整圈任务。 */
    if (task_state != TASK_IDLE) {
        task_total_start_requested = false;
    }
}

Task_State_t task_get_state(void)
{
    return task_state;
}
