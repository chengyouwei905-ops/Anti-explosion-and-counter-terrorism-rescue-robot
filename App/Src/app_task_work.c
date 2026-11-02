#include "app_task_work.h"
#include "app_vision.h"
#include "app_imu.h"
#include "app_chassis_motion.h"
#include "app_chassis_closeloop.h"
#include <math.h>
#include <string.h>

#define WORK_TX_TIMEOUT_MS 1000U
#define WORK_TX_RETRY_MS 20U
#define WORK_LOCATE_TIMEOUT_MS 23000U
#define WORK_SCAN_TIMEOUT_MS 63000U
#define WORK_ROTATE_SPEED 0.20f
#define WORK_ROTATE_ACCEL 0.40f
#define WORK_OBSERVE_YAW_TOLERANCE 0.017453293f

static TaskWorkStatus_t s;
static TaskWorkConfig_t config;
static VisionResult_t received;
static bool result_pending;
static uint32_t phase_ms, request_ms, motion_timeout_ms;
static uint32_t last_tx_ms;
static bool tx_tried, arm_entered;
static bool scan_started, scan_complete;
static uint8_t target_slot; /* 内部下标 0..2，线上序号 1..3 */
static float current_offset[2];
static float pending_offset[2];
static float anchor_yaw, request_yaw;
static TaskWorkPhase_t motion_next, settle_next;
static bool motion_changes_slot;
static bool cancel_vision, cancel_arm, cancel_hook;
static bool cancel_hook_entered;
static uint32_t cancel_ms, cancel_try_ms;
static bool cancel_tried;

static void phase_enter(TaskWorkPhase_t phase, uint32_t now)
{
    s.phase = phase;
    phase_ms = now;
    tx_tried = false;
    arm_entered = false;
}

static bool read_yaw(float *yaw)
{
    float euler[3];
    if (!app_imu_get_euler(euler) || !isfinite(euler[2])) {
        return false;
    }
    *yaw = euler[2];
    return true;
}

static void cancel_begin(uint32_t now)
{
    s.request_pending = false;
    result_pending = false;
    s.alignment_valid = false;
    task_work_laser_set(false);
    if (!s.cancel_pending) {
        cancel_ms = now;
        cancel_tried = false;
        cancel_vision = true;
        cancel_arm = true;
        cancel_hook = true;
        cancel_hook_entered = false;
        s.cancel_failed = false;
        s.cancel_pending = true;
    }
}

static void work_fail(TaskWorkError_t error, uint32_t now)
{
    s.error = error;
    cancel_begin(now);
    app_chassis_closeloop_init();
    app_chassis_motion_stop();
    phase_enter(WORK_ERROR, now);
}

void app_task_work_init(void)
{
    s = (TaskWorkStatus_t){0};
    result_pending = false;
    cancel_vision = cancel_arm = cancel_hook = false;
    task_work_laser_set(false);
}

void app_task_work_cancel(uint32_t now)
{
    cancel_begin(now);
    phase_enter(WORK_IDLE, now);
}

bool app_task_work_cancel_pending(void)
{
    return s.cancel_pending;
}

const TaskWorkStatus_t *app_task_work_get_status(void)
{
    return &s;
}

void app_task_work_on_frame(const proto_vision_frame_t *frame)
{
#if TASK_WORK_CAMERA_TEST
    /* 相机故障可能回失败或晚到结果，联调时不让其覆盖模拟结果。
     * 此回调只负责作业结果；task.c中的真实启停处理不受影响。
     */
    (void)frame;
    return;
#endif
    if (frame == NULL || frame->cmd != VISION_CMD_RESULT || !s.request_pending) {
        return;
    }
    VisionResult_t r;
    if (!proto_vision_decode_result(frame->payload, frame->len, &r)) {
        return; /* 坏载荷不解除等待，最终由本地超时处理 */
    }
    if (r.task_cmd != s.request_cmd || r.input_seq != s.request_seq ||
        r.status == VISION_OBSERVATION) {
        return;
    }
    /* 拷贝完整结果，不保存协议层的临时指针。清除 pending 拒绝重复终态。 */
    received = r;
    result_pending = true;
    s.request_pending = false;
}

void app_task_work_service(uint32_t now)
{
    if (s.cancel_pending) {
        if (cancel_hook) {
            TaskArmResult_t result = task_work_arm_poll(s.kind, ARM_CANCEL, 0U, !cancel_hook_entered);
            cancel_hook_entered = true;
            if (result == ARM_DONE) {
                cancel_hook = false;
            } else if (result == ARM_FAILED) {
                s.cancel_failed = true;
                cancel_hook = false;
            }
        }
        if (!cancel_tried || (uint32_t)(now - cancel_try_ms) >= WORK_TX_RETRY_MS) {
            cancel_tried = true;
            cancel_try_ms = now;
            if (cancel_vision) {
                uint8_t seq;
                int result = app_vision_send_with_seq(VISION_CMD_CANCEL, NULL, 0U, &seq);
                if (result == RET_OK) {
                    cancel_vision = false; /* 不等 0x7F 应答：对端可能空闲不回包 */
                } else if (result != RET_BUSY) {
                    s.cancel_failed = true;
                    cancel_vision = false;
                }
            }
            if (cancel_arm) {
                int result = task_work_arm_stop_it();
                if (result == RET_OK) {
                    cancel_arm = false;
                } else if (result != RET_BUSY) {
                    s.cancel_failed = true;
                    cancel_arm = false;
                }
            }
        }
        if (!cancel_vision && !cancel_arm && !cancel_hook) {
            s.cancel_pending = false;
        } else if ((uint32_t)(now - cancel_ms) >= WORK_TX_TIMEOUT_MS) {
            s.cancel_pending = false;
            s.cancel_failed = true;
        }
    }
    if (s.request_pending || result_pending) {
        uint32_t timeout = proto_vision_is_scan(s.request_cmd) ?
                           WORK_SCAN_TIMEOUT_MS : WORK_LOCATE_TIMEOUT_MS;
        /* 终态在超时后才到达也不能重新驱动动作。 */
        if ((uint32_t)(now - request_ms) >= timeout) {
            work_fail(WORK_ERR_TIMEOUT, now);
        }
    }
    if (s.phase == WORK_LASER && (uint32_t)(now - phase_ms) >= config.laser_ms) {
        task_work_laser_set(false);
    }
}

static bool positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static bool validate_config(void)
{
    if (config.settle_ms < 250U || config.settle_ms > 10000U ||
        config.scan_dwell_ms < 250U || config.scan_dwell_ms > 10000U ||
        config.arm_timeout_ms == 0U || config.arm_timeout_ms > 60000U ||
        config.laser_ms == 0U || config.laser_ms > 10000U ||
        !positive(config.scan_speed) || config.scan_speed > 200.0f ||
        !positive(config.correction_speed) || config.correction_speed > 100.0f ||
        !positive(config.accel) || config.accel > 500.0f ||
        !positive(config.max_correction_mm) || config.max_correction_mm > 300.0f ||
        !positive(config.max_correction_yaw) || config.max_correction_yaw > 0.523599f ||
        !positive(config.position_tolerance_mm) ||
        config.position_tolerance_mm >= config.max_correction_mm ||
        !positive(config.yaw_tolerance_rad) || config.yaw_tolerance_rad >= config.max_correction_yaw ||
        !positive(config.height_tolerance_mm) || config.max_corrections == 0U || config.max_corrections > 10U) {
        return false;
    }
    unsigned count = s.kind == WORK_QR ? 1U : 3U;
    for (unsigned i = 0; i < count; ++i) {
        if (!isfinite(config.slot[i].offset_mm[0]) || !isfinite(config.slot[i].offset_mm[1]) ||
            hypotf(config.slot[i].offset_mm[0], config.slot[i].offset_mm[1]) > 2000.0f ||
            !alg_vision_transform_from_arm_pose(&config.slot[i].observe_pose,
                                                &config.slot[i].alignment.body_from_tcp) ||
            !alg_vision_alignment_config_valid(&config.slot[i].alignment)) {
            return false;
        }
        if ((s.kind == WORK_BOMB || s.kind == WORK_RESCUE) &&
            config.slot[i].alignment.yaw_mode != VISION_YAW_NONE) {
            return false; /* 单个球心/中点不能提供 yaw */
        }
        if (s.kind == WORK_QR && config.slot[i].alignment.yaw_mode == VISION_YAW_TARGET_NORMAL) {
            return false;
        }
        if (s.kind == WORK_TARGET && config.slot[i].alignment.yaw_mode == VISION_YAW_QR_AXIS) {
            return false;
        }
    }
    return true;
}

static uint8_t locate_command(void)
{
    switch (s.kind) {
    case WORK_QR: return VISION_CMD_QR;
    case WORK_BOMB: return VISION_CMD_BOMB_LOCATE;
    case WORK_TARGET: return VISION_CMD_TARGET_LOCATE;
    default: return VISION_CMD_RESCUE_LOCATE;
    }
}

static uint8_t scan_command(void)
{
    switch (s.kind) {
    case WORK_BOMB: return VISION_CMD_BOMB_SCAN;
    case WORK_TARGET: return VISION_CMD_TARGET_SCAN;
    default: return VISION_CMD_RESCUE_SCAN;
    }
}

static bool send_request(uint8_t cmd, uint32_t now)
{
    if ((uint32_t)(now - phase_ms) >= WORK_TX_TIMEOUT_MS) {
        work_fail(WORK_ERR_TX, now);
        return false;
    }
    if (tx_tried && (uint32_t)(now - last_tx_ms) < WORK_TX_RETRY_MS) {
        return false;
    }
    tx_tried = true;
    last_tx_ms = now;
    if (s.request_pending || result_pending || app_chassis_motion_get_state() != MOTION_STATE_IDLE) {
        work_fail(WORK_ERR_MOTION, now);
        return false;
    }
    if (!read_yaw(&request_yaw)) {
        work_fail(WORK_ERR_IMU, now);
        return false;
    }
    uint8_t seq;
    int result = app_vision_send_with_seq(cmd, NULL, 0U, &seq);
    if (result == RET_BUSY) {
        return false;
    }
    if (result != RET_OK) {
        work_fail(WORK_ERR_TX, now);
        return false;
    }
    s.request_cmd = cmd;
    s.request_seq = seq;
    request_ms = now;
    s.request_pending = true;
    return true;
}

static void motion_started(float distance, float speed, float accel,
                            TaskWorkPhase_t next, uint32_t now)
{
    if (app_chassis_motion_get_result() != MOTION_RESULT_RUNNING) {
        work_fail(WORK_ERR_MOTION, now);
        return;
    }
    /* 梯形/三角形轨迹时间的保守上界，再加 2 秒故障裕量。 */
    float seconds = distance / speed + 2.0f * speed / accel;
    if (!isfinite(seconds) || seconds > 120.0f) {
        work_fail(WORK_ERR_CONFIG, now);
        return;
    }
    motion_timeout_ms = (uint32_t)ceilf(seconds * 1000.0f) + 2000U;
    motion_next = next;
    s.alignment_valid = false;
    phase_enter(WORK_WAIT_MOTION, now);
}

static void move_offset(float x, float y, TaskWorkPhase_t next, uint32_t now)
{
    float dx = x - current_offset[0];
    float dy = y - current_offset[1];
    float distance = hypotf(dx, dy);
    if (app_chassis_motion_get_state() != MOTION_STATE_IDLE) {
        work_fail(WORK_ERR_MOTION, now);
        return;
    }
    if (distance <= 0.01f) {
        phase_enter(next, now);
        return;
    }
    float yaw;
    if (!read_yaw(&yaw)) {
        work_fail(WORK_ERR_IMU, now);
        return;
    }
    /* linear_move 的方向 0=前，+PI/2=左，故 X右/Y前 使用 atan2(-dx,dy)。 */
    float direction = atan2f(-dx, dy) + anchor_yaw - yaw;
    pending_offset[0] = x;
    pending_offset[1] = y;
    motion_changes_slot = true;
    app_chassis_motion_linear_move(distance, direction, config.scan_speed, config.accel, config.accel);
    motion_started(distance, config.scan_speed, config.accel, next, now);
}

static bool arm_done(TaskArmStep_t step, uint32_t now)
{
    uint8_t slot = s.kind == WORK_QR ? 0U : target_slot + 1U;
    TaskArmResult_t result = task_work_arm_poll(s.kind, step, slot, !arm_entered);
    arm_entered = true;
    if (result == ARM_FAILED) {
        work_fail(WORK_ERR_ARM, now);
        return false;
    }
    if (result == ARM_DONE) {
        return true;
    }
    if ((uint32_t)(now - phase_ms) >= config.arm_timeout_ms) {
        work_fail(WORK_ERR_ARM, now);
    }
    return false;
}

static void start_alignment(uint32_t now)
{
    if (!proto_vision_result_control_ready(&received, 5U, 8.0f) ||
        (s.kind == WORK_QR ? received.slot_id != 0U : received.slot_id != s.selected_slot) ||
        !alg_vision_alignment_compute(&received, &config.slot[target_slot].alignment,
                                       request_yaw, &s.alignment)) {
        work_fail(WORK_ERR_DATA, now);
        return;
    }
    s.alignment_valid = true;
    float dx = s.alignment.delta_body[0];
    float dy = s.alignment.delta_body[1];
    float distance = hypotf(dx, dy);
    float angle = s.alignment.delta_yaw;
    float height_error = s.alignment.reference_body[2] -
                         config.slot[target_slot].alignment.desired_reference_body[2];
    if (distance > config.max_correction_mm || fabsf(angle) > config.max_correction_yaw ||
        fabsf(height_error) > config.height_tolerance_mm) {
        work_fail(WORK_ERR_ALIGNMENT, now);
        return; /* 不截断大误差后继续，Z 误差也不能靠平面底盘消除 */
    }
    if (distance <= config.position_tolerance_mm && fabsf(angle) <= config.yaw_tolerance_rad) {
        if (!read_yaw(&anchor_yaw)) {
            work_fail(WORK_ERR_IMU, now);
            return;
        }
        if (s.kind == WORK_QR) {
            phase_enter(WORK_TRAVEL_ARM, now);
        } else if (s.kind == WORK_TARGET) {
            task_work_laser_set(true);
            phase_enter(WORK_LASER, now);
        } else {
            phase_enter(WORK_PICK, now);
            s.alignment_valid = false; /* 物体即将被搬动，不能再当固定地图参照 */
        }
        return;
    }
    if (s.corrections >= config.max_corrections) {
        work_fail(WORK_ERR_ALIGNMENT, now);
        return;
    }
    ++s.corrections;
    motion_changes_slot = false;
    if (fabsf(angle) > config.yaw_tolerance_rad) {
        app_chassis_motion_circular_move(0.0f, angle, true, WORK_ROTATE_SPEED,
                                         WORK_ROTATE_ACCEL, WORK_ROTATE_ACCEL);
        motion_started(fabsf(angle), WORK_ROTATE_SPEED, WORK_ROTATE_ACCEL, WORK_PREPARE_ARM, now);
    } else {
        app_chassis_motion_linear_move(distance, atan2f(-dx, dy), config.correction_speed,
                                       config.accel, config.accel);
        motion_started(distance, config.correction_speed, config.accel, WORK_PREPARE_ARM, now);
    }
}

#if TASK_WORK_CAMERA_TEST
static void camera_test_result(void)
{
    if (!s.request_pending || result_pending) return;
    received = (VisionResult_t){
        .task_cmd = s.request_cmd,
        .input_seq = s.request_seq,
        .status = VISION_SUCCESS,
        .slot_id = s.kind == WORK_QR ? 0U : 1U
    };
    if (s.phase == WORK_WAIT_LOCATE) {
        const VisionAlignmentConfig_t *alignment = &config.slot[target_slot].alignment;
        /* 本轮配置只平移。期望底盘坐标反变换到夹爪E系：
         * p_E = R^T * (p_B_expected - t)，再走原换算/校验流程，得到零纠偏。
         * 不能把底盘XYZ直接填进K230的E系字段。
         */
        received.flags = VISION_POSITION | VISION_CONTROL_READY;
        received.sample_count = 5U;
        const VisionTransform_t *transform = &alignment->body_from_tcp;
        for (unsigned i = 0U; i < 3U; ++i) {
            for (unsigned j = 0U; j < 3U; ++j) {
                received.xyz_mm[i] += transform->rotation[j][i] *
                    (alignment->desired_reference_body[j] - transform->translation[j]);
            }
        }
    }
    s.request_pending = false;
    result_pending = true;
}
#endif

TaskWorkResult_t app_task_work_poll(TaskWorkKind_t kind, uint32_t now)
{
    if ((unsigned)kind >= WORK_COUNT || s.cancel_failed) {
        return WORK_FAILED;
    }
    if (s.cancel_pending) {
        return WORK_WAITING;
    }
    if (s.phase == WORK_IDLE || (s.phase == WORK_COMPLETE && s.kind != kind)) {
        s = (TaskWorkStatus_t){.kind = kind};
        result_pending = false;
        scan_started = scan_complete = false;
        target_slot = 0U;
        current_offset[0] = current_offset[1] = 0.0f;
        task_work_laser_set(false);
        phase_enter(WORK_WAIT_CONFIG, now);
    }
    if (s.kind != kind) {
        work_fail(WORK_ERR_CONFIG, now);
        return WORK_FAILED;
    }
    app_task_work_service(now);
    if (s.phase == WORK_ERROR) {
        return WORK_FAILED;
    }
    if (result_pending && received.status != VISION_SUCCESS) {
        s.vision_reason = received.reason;
        work_fail(WORK_ERR_VISION, now);
        return WORK_FAILED;
    }
    if (s.phase != WORK_WAIT_CONFIG && s.phase != WORK_COMPLETE) {
        float yaw;
        if (!read_yaw(&yaw)) {
            work_fail(WORK_ERR_IMU, now);
            return WORK_FAILED;
        }
        if (s.phase != WORK_WAIT_MOTION && app_chassis_motion_get_state() != MOTION_STATE_IDLE) {
            work_fail(WORK_ERR_MOTION, now);
            return WORK_FAILED;
        }
    }

    switch (s.phase) {
    case WORK_WAIT_CONFIG:
        /* 无机械臂观察位/目标参数时停在这里，继续服务所有外设 poll。 */
        if (!task_work_load_config(kind, &config)) {
            break;
        }
        if (!validate_config()) {
            work_fail(WORK_ERR_CONFIG, now);
            break;
        }
        if (!read_yaw(&anchor_yaw)) {
            work_fail(WORK_ERR_IMU, now);
            break;
        }
        phase_enter(WORK_MOVE_SLOT, now);
        break;

    case WORK_MOVE_SLOT:
        move_offset(config.slot[target_slot].offset_mm[0], config.slot[target_slot].offset_mm[1],
                    WORK_PREPARE_ARM, now);
        break;

    case WORK_WAIT_MOTION: {
        Chassis_Motion_Result_t result = app_chassis_motion_get_result();
        if (result == MOTION_RESULT_DONE) {
            if (motion_changes_slot) {
                current_offset[0] = pending_offset[0];
                current_offset[1] = pending_offset[1];
            }
            /* 下一步观察姿态也会执行停稳等待；返回路线时单独停稳。 */
            if (motion_next == WORK_COMPLETE) {
                settle_next = WORK_COMPLETE;
                phase_enter(WORK_SETTLE, now);
            } else {
                phase_enter(motion_next, now);
            }
        } else if (result != MOTION_RESULT_RUNNING || (uint32_t)(now - phase_ms) > motion_timeout_ms) {
            work_fail(WORK_ERR_MOTION, now);
        }
        break;
    }

    case WORK_PREPARE_ARM:
        if (arm_done(ARM_OBSERVE, now)) {
            if (s.kind == WORK_QR || scan_complete) {
                settle_next = WORK_SEND_LOCATE;
            } else if (!scan_started) {
                settle_next = WORK_SEND_SCAN;
            } else {
                settle_next = WORK_SCAN_DWELL;
            }
            phase_enter(WORK_SETTLE, now);
        }
        break;

    case WORK_SETTLE:
        if ((uint32_t)(now - phase_ms) >= config.settle_ms) {
            phase_enter(settle_next, now);
        }
        break;

    case WORK_SEND_SCAN:
        if (send_request(scan_command(), now)) {
            scan_started = true;
            phase_enter(WORK_SCAN_DWELL, now);
        }
        break;

    case WORK_SCAN_DWELL:
        if (result_pending) {
            phase_enter(WORK_SELECT_SLOT, now);
        } else if ((uint32_t)(now - phase_ms) >= config.scan_dwell_ms) {
            if (target_slot < 2U) {
                ++target_slot;
                phase_enter(WORK_MOVE_SLOT, now);
            } else {
                phase_enter(WORK_WAIT_SCAN, now);
            }
        }
        break;

    case WORK_WAIT_SCAN:
#if TASK_WORK_CAMERA_TEST
        camera_test_result(); /* 三个观察位走完后，模拟扫描成功并选择第1位。 */
#endif
        if (result_pending) {
            phase_enter(WORK_SELECT_SLOT, now);
        }
        break;

    case WORK_SELECT_SLOT:
        if (!result_pending || received.slot_id < 1U || received.slot_id > 3U) {
            work_fail(WORK_ERR_DATA, now);
            break;
        }
        s.selected_slot = received.slot_id;
        target_slot = received.slot_id - 1U;
        result_pending = false;
        scan_complete = true;
        phase_enter(WORK_MOVE_SLOT, now);
        break;

    case WORK_SEND_LOCATE:
        if (send_request(locate_command(), now)) {
            phase_enter(WORK_WAIT_LOCATE, now);
        }
        break;

    case WORK_WAIT_LOCATE: {
#if TASK_WORK_CAMERA_TEST
        camera_test_result(); /* QR及各作业定位均模拟已对准，无底盘二次修正。 */
#endif
        float yaw;
        if (!read_yaw(&yaw)) {
            work_fail(WORK_ERR_IMU, now);
        } else if (app_chassis_motion_get_state() != MOTION_STATE_IDLE ||
                   fabsf(yaw - request_yaw) > WORK_OBSERVE_YAW_TOLERANCE) {
            work_fail(WORK_ERR_OBSERVATION_MOVED, now);
        } else if (result_pending) {
            result_pending = false;
            phase_enter(WORK_ALIGN, now);
        }
        break;
    }

    case WORK_ALIGN:
        start_alignment(now);
        break;

    case WORK_PICK:
        if (arm_done(ARM_PICK, now)) {
            if (s.kind == WORK_BOMB) {
                /* ARM_PICK 完成表示已经闭爪并抬到 QR 搬运姿态。 */
                phase_enter(WORK_MOVE_TO_BUCKET, now);
            } else {
                /* 人质保持夹紧，收臂后带到终点，不经过放置/松爪。 */
                phase_enter(WORK_TRAVEL_ARM, now);
            }
        }
        break;

    case WORK_MOVE_TO_BUCKET:
        /* 第二个爆炸物位置对应桶的放置停车点。
         * 更新 current_offset，之后 WORK_RETURN 从此处撤回路线基点。
         * 保留视觉纠偏量，不把真实纠偏当作扫描名义位移撤销。
         */
        settle_next = WORK_PLACE;
        move_offset(config.slot[1].offset_mm[0], config.slot[1].offset_mm[1],
                    WORK_SETTLE, now);
        break;

    case WORK_PLACE:
        if (arm_done(ARM_PLACE, now)) {
            phase_enter(WORK_TRAVEL_ARM, now);
        }
        break;

    case WORK_LASER:
        if ((uint32_t)(now - phase_ms) >= config.laser_ms) {
            task_work_laser_set(false);
            phase_enter(WORK_TRAVEL_ARM, now);
        }
        break;

    case WORK_TRAVEL_ARM:
        if (arm_done(ARM_TRAVEL, now)) {
            phase_enter(WORK_RETURN, now);
        }
        break;

    case WORK_RETURN:
        /* 从选中观察位回到该作业区的路线停车点，下一段原路线才仍然成立。
         * 不撤销视觉微调：各 slot 的期望坐标必须对应同一地图基准和航向。
         */
        move_offset(0.0f, 0.0f, WORK_COMPLETE, now);
        break;

    case WORK_COMPLETE:
        s.alignment_valid = false; /* 即将回到路线运动，保留数值仅供回看 */
        return WORK_DONE;

    default:
        work_fail(WORK_ERR_CONFIG, now);
        break;
    }
    return s.phase == WORK_ERROR ? WORK_FAILED : WORK_WAITING;
}
