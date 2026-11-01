#include "app_task_work.h"
#include "main.h"
#include "bsp_uart.h"
#include "proto_roboarm.h"
#include "common_def.h"
#include "app_roboarm.h"
#include <math.h>

/* 只在确认正确停车时的参考点坐标后，填写 XYZ 并将对应 ready 改为 true。
 * X右/Y前/Z上，单位mm；同一类三个slot共用。NAN明确表示尚未标定。
 */
typedef struct {
    bool ready;
    float reference_body[3];
} WorkReference_t;
static const WorkReference_t work_reference[WORK_COUNT] = {
    /* 2026-11-01 用户确认；高度相对机械臂基座原点，不是离地高度。 */
    [WORK_QR] = {true, {0.0f, 1250.0f, 55.0f}},
    [WORK_BOMB] = {true, {-225.0f, 0.0f, 0.0f}},
    [WORK_TARGET] = {true, {-620.0f, 0.0f, 20.0f}},
    [WORK_RESCUE] = {true, {-225.0f, 0.0f, 55.0f}}
};

static const Roboarm_Pose_t observe_pose_id[WORK_COUNT] = {
    ROBOARM_POSE_QR_OBSERVE, ROBOARM_POSE_BOMB_OBSERVE,
    ROBOARM_POSE_TARGET_OBSERVE, ROBOARM_POSE_RESCUE_OBSERVE
};

/* 相对各自路线停车点的累计位移，X右/Y前，mm；三类均已由用户确认。 */
static const float observation_offsets[WORK_COUNT][3][2] = {
    [WORK_QR] = {{0, 0}, {0, 0}, {0, 0}},
    [WORK_BOMB] = {{0, 0}, {0, 150}, {0, 300}},
    [WORK_TARGET] = {{0, 0}, {0, 150}, {0, 300}},
    [WORK_RESCUE] = {{0, 0}, {0, 150}, {0, 300}}
};

static bool observation_pose(TaskWorkKind_t kind, End_Effector_Pos_t *pose)
{
    const uint16_t *positions = app_roboarm_pose_positions(observe_pose_id[kind]);
    if (positions == NULL) {
        return false;
    }
    const double rad_per_position = 0.24 * 3.14159265358979323846 / 180.0;
    Roboarm_State_t joints = {
        .yaw_angle = (YAW_SERVO_OFFSET - (double)positions[0]) * rad_per_position,
        .joint1_angle = ((double)positions[1] - JOINT1_OFFSET) * rad_per_position,
        .joint2_angle = (JOINT2_OFFSET - (double)positions[2]) * rad_per_position,
        .joint3_angle = ((double)positions[3] - JOINT3_OFFSET) * rad_per_position
    };
    /* 正解内部处理35:44，不重复减速。发送观察动作也使用同一原始姿态。 */
    return alg_roboarm_forward(&joints, pose) == ROBOARM_KINEMATICS_OK;
}

bool task_work_load_config(TaskWorkKind_t kind, TaskWorkConfig_t *out)
{
    if (out == NULL || (unsigned)kind >= WORK_COUNT) {
        return false;
    }
    *out = (TaskWorkConfig_t){
        .settle_ms = 1000U,
        .scan_dwell_ms = 3000U,
        .arm_timeout_ms = 30000U,
        .laser_ms = 1000U,          /* 激光照射时间，后续按实物判定调试 */
        .scan_speed = 80.0f,
        .correction_speed = 40.0f,
        .accel = 100.0f,
        .max_correction_mm = 100.0f,
        .max_correction_yaw = 0.174532925f, /* 单次最大 10 度，超出时报错 */
        .position_tolerance_mm = 5.0f,
        .yaw_tolerance_rad = 0.017453293f,  /* 1 度，第一版实际只做平移 */
        .height_tolerance_mm = 10.0f,
        .max_corrections = 4U
    };

    End_Effector_Pos_t pose;
    if (!observation_pose(kind, &pose)) {
        return false;
    }
    unsigned count = kind == WORK_QR ? 1U : 3U;
    for (unsigned i = 0U; i < count; ++i) {
        out->slot[i].offset_mm[0] = observation_offsets[kind][i][0];
        out->slot[i].offset_mm[1] = observation_offsets[kind][i][1];
        out->slot[i].observe_pose = pose;
        VisionAlignmentConfig_t *alignment = &out->slot[i].alignment;
        alignment->yaw_mode = VISION_YAW_NONE;
        alignment->map_reference_valid = false;
        if (!alg_vision_transform_from_arm_pose(&pose, &alignment->body_from_tcp)) {
            return false;
        }
        for (unsigned axis = 0U; axis < 3U; ++axis) {
            float reference = work_reference[kind].reference_body[axis];
            float margin = axis == 2U ? 50.0f : 150.0f;
            alignment->desired_reference_body[axis] = reference;
            alignment->min_reference_body[axis] = reference - margin;
            alignment->max_reference_body[axis] = reference + margin;
        }
    }
    /* 即使误改ready，NAN仍不能通过；未知坐标不会变成零偏纠正指令。 */
    return work_reference[kind].ready &&
           isfinite(work_reference[kind].reference_body[0]) &&
           isfinite(work_reference[kind].reference_body[1]) &&
           isfinite(work_reference[kind].reference_body[2]);
}

#define ARM_POSE_TIME_MS 3000U
#define ARM_CLAMP_TIME_MS 1000U
#define ARM_SETTLE_TIME_MS 500U
#define ARM_TX_MARGIN_MS 100U /* 9600bps最长本模块帧小于30ms，预留传输时间 */
#define ARM_SEND_TIMEOUT_MS 1000U
#define ARM_SEND_RETRY_MS 20U

static struct {
    bool active, sent, tried;
    TaskWorkKind_t kind;
    TaskArmStep_t step;
    uint8_t slot, action;
    uint32_t wait_ms, send_ms, retry_ms, action_ms;
} arm_work;

TaskArmResult_t task_work_arm_poll(TaskWorkKind_t kind, TaskArmStep_t step,
                                  uint8_t slot, bool entering)
{
    uint32_t now = HAL_GetTick();
    if (step == ARM_CANCEL) {
        /* 取消软件步骤，不松爪，防止放掉正在搬运的人质/爆炸物。
         * 硬件停止由 task_work_arm_stop_it() 另行发送。
         */
        arm_work.active = false;
        arm_work.sent = false;
        arm_work.tried = false;
        arm_work.action = 0U;
        return ARM_DONE;
    }
    if ((unsigned)kind >= WORK_COUNT ||
        (kind == WORK_QR ? slot != 0U : (slot < 1U || slot > 3U))) {
        return ARM_FAILED;
    }
    if (entering) {
        arm_work.active = true;
        arm_work.kind = kind;
        arm_work.step = step;
        arm_work.slot = slot;
        arm_work.action = 0U;
        arm_work.sent = false;
        arm_work.tried = false;
        arm_work.action_ms = now;
    } else if (!arm_work.active || arm_work.kind != kind ||
               arm_work.step != step || arm_work.slot != slot) {
        return ARM_FAILED;
    }

    /* 当前子动作只发一次，使用无符号tick差值兼容计时回绕。 */
    if (arm_work.sent) {
        if ((uint32_t)(now - arm_work.send_ms) < arm_work.wait_ms) {
            return ARM_WAITING;
        }
        ++arm_work.action;
        arm_work.sent = false;
        arm_work.tried = false;
        arm_work.action_ms = now;
    }

    bool clamp_action = false;
    uint16_t clamp_position = CLAMP_OPEN_POSITION;
    Roboarm_Pose_t pose = ROBOARM_POSE_QR_OBSERVE;
    switch (step) {
    case ARM_OBSERVE:
        if (arm_work.action >= 1U) return ARM_DONE;
        pose = observe_pose_id[kind];
        break;
    case ARM_PICK:
        if (kind != WORK_BOMB && kind != WORK_RESCUE) return ARM_FAILED;
        /* 张爪 -> 下到夹取位 -> 闭爪 -> 抬到QR搬运位。 */
        if (arm_work.action >= 4U) return ARM_DONE;
        if (arm_work.action == 0U) {
            clamp_action = true;
        } else if (arm_work.action == 1U) {
            pose = kind == WORK_BOMB ? ROBOARM_POSE_BOMB_PICK : ROBOARM_POSE_RESCUE_PICK;
        } else if (arm_work.action == 2U) {
            clamp_action = true;
            clamp_position = CLAMP_CLOSE_POSITION;
        }
        break;
    case ARM_PLACE:
        /* 只有爆炸物进入此步骤，且底盘已到第二个爆炸物位置并停稳。 */
        if (kind != WORK_BOMB) return ARM_FAILED;
        if (arm_work.action >= 2U) return ARM_DONE;
        if (arm_work.action == 0U) {
            pose = ROBOARM_POSE_BOMB_PLACE;
        } else {
            clamp_action = true;
        }
        break;
    case ARM_TRAVEL:
        if (arm_work.action >= 1U) return ARM_DONE;
        /* QR姿态收臂；只发ID1..4，不修改ID5夹爪状态。 */
        break;
    default:
        return ARM_FAILED;
    }

    if ((uint32_t)(now - arm_work.action_ms) >= ARM_SEND_TIMEOUT_MS) {
        return ARM_FAILED;
    }
    if (arm_work.tried && (uint32_t)(now - arm_work.retry_ms) < ARM_SEND_RETRY_MS) {
        return ARM_WAITING;
    }
    arm_work.tried = true;
    arm_work.retry_ms = now;
    uint16_t duration = clamp_action ? ARM_CLAMP_TIME_MS : ARM_POSE_TIME_MS;
    int result;
    if (clamp_action) {
        const proto_roboarm_servo_t servo = {JOINT_CLAMP, clamp_position};
        result = app_roboarm_positions_it(&servo, 1U, duration);
    } else {
        result = app_roboarm_pose_it(pose, duration);
    }
    if (result == RET_BUSY) return ARM_WAITING;
    if (result != RET_OK) return ARM_FAILED;
    arm_work.sent = true;
    arm_work.send_ms = now;
    arm_work.wait_ms = duration + ARM_TX_MARGIN_MS + ARM_SETTLE_TIME_MS;
    return ARM_WAITING;
}

void task_work_laser_set(bool on)
{
    /* CubeMX 已配置 PA0 推挽输出；不在此重复初始化引脚。 */
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int task_work_arm_stop_it(void)
{
    uint8_t frame[PROTO_ROBOARM_MAX_FRAME];
    uint8_t len = proto_roboarm_pack_action_group_stop(frame, (uint8_t)sizeof(frame));
    if (len == 0U) {
        return RET_PARAM;
    }
    /* 只停止控制板动作组，不谎称它能中断所有已下发的单舵机插值。
     * 若以后直接控制舵机，ARM_CANCEL 中还需按控制板能力实现保持/停止。
     */
    return bsp_uart_send_it(BSP_UART_ROBOARM, frame, len);
}
