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
    /* 晚上qr85，1400，150，白天40 */
    /* 11月6日75好 */
    [WORK_QR] = {true, {85.0f, 1385.0f, 150.0f}},
    [WORK_BOMB] = {true, {-230.0f, 10.0f, -87.88f}},
    [WORK_TARGET] = {true, {-710.0f, 30.0f, 95.0f}},
    /* 2026-11-03 到位实测；目标坐标为 -arm_origin_relative_qr，而非原始 TCP XYZ。 */
    [WORK_RESCUE] = {true, {-237, 19.8726902f, 33.2348137f}}
};

static const Roboarm_Pose_t observe_pose_id[WORK_COUNT] = {
    ROBOARM_POSE_QR_OBSERVE, ROBOARM_POSE_BOMB_OBSERVE,
    ROBOARM_POSE_TARGET_OBSERVE, ROBOARM_POSE_RESCUE_OBSERVE
};

/* 广工赛合适参数.xlsx，Sheet1!A2:E7，2026-11-03 赛道实测。
 * 人质观察位姿按用户后续复测参数覆盖表中原值。
 * 机械臂坐标 X左/Y前/Z上，mm；pitch 为 rad。
 * 二维码观察与统一搬运姿态在下方独立设置。
 */
static const struct {
    bool provided;
    End_Effector_Pos_t pose;
} task_arm_poses[ROBOARM_POSE_COUNT] = {
    [ROBOARM_POSE_QR_OBSERVE]     = {true, {0.0, 260.0, 60.0, 0.0}},
    [ROBOARM_POSE_BOMB_OBSERVE]   = {true, {154.0, 0.0,   26.0, -1.0 + 0.2}},
    [ROBOARM_POSE_BOMB_PICK]      = {true, {200.0, 0.0, -23.0, -1.25}},
    [ROBOARM_POSE_BOMB_PLACE]     = {true, {-174.0, 0.0, 20.0, -1.2}},
    [ROBOARM_POSE_TARGET_OBSERVE] = {true, {170.0, 0.0,  25.0,  0.0}},
    [ROBOARM_POSE_RESCUE_OBSERVE] = {true, {133.537333615+2.0, 0.0, 84.0 + 5.0, -0.338 - 0.05}},
    [ROBOARM_POSE_RESCUE_PICK]    = {true, {205.0, 0.0,  78.0, -0.5}}
};

/* 排爆/打靶/救援共用的搬运、收臂中间位姿；单位mm，pitch为rad。 */
static const End_Effector_Pos_t task_arm_travel_pose = {0.0, 250.0, 100.0, -0.05};

typedef enum {
    ARM_POSE_ALL,          /* 原四关节同步姿态 */
    ARM_POSE_WITHOUT_YAW,  /* 只动ID2/3/4，保持当前yaw */
    ARM_POSE_YAW_ONLY      /* 只动ID1，保持升降/伸缩/俯仰 */
} ArmPosePart_t;

static int task_arm_cartesian_send_it(const End_Effector_Pos_t *p,
                                     uint16_t duration, ArmPosePart_t part)
{
    if (part == ARM_POSE_ALL) {
        return app_roboarm_servo_move_to_it(p->x, p->y, p->z, p->pitch, duration);
    }
    /* 用目标XYZ/pitch反解，仅下发指定关节；进入观察位与收臂使用相反顺序。 */
    Roboarm_State_t joints;
    if (alg_roboarm_invert(p, &joints) != ROBOARM_KINEMATICS_OK) {
        return RET_ERROR;
    }
    const Roboarm_Servo_t servos[ROBOARM_DOF] = {
        {YAW_SERVO, joints.yaw_angle}, {JOINT1, joints.joint1_angle},
        {JOINT2, joints.joint2_angle}, {JOINT3, joints.joint3_angle}
    };
    return part == ARM_POSE_YAW_ONLY ?
        app_roboarm_servo_move_direct_it(&servos[0], 1U, duration) :
        app_roboarm_servo_move_direct_it(&servos[1], 3U, duration);
}

static int task_arm_pose_send_it(Roboarm_Pose_t pose, uint16_t duration, ArmPosePart_t part)
{
    if ((unsigned)pose >= ROBOARM_POSE_COUNT) {
        return RET_PARAM;
    }
    if (task_arm_poses[pose].provided) {
        return task_arm_cartesian_send_it(&task_arm_poses[pose].pose, duration, part);
    }
    if (part == ARM_POSE_ALL) {
        return app_roboarm_pose_it(pose, duration);
    }
    const uint16_t *positions = app_roboarm_pose_positions(pose);
    if (positions == NULL) return RET_PARAM;
    proto_roboarm_servo_t servos[ROBOARM_DOF];
    unsigned first = part == ARM_POSE_YAW_ONLY ? 0U : 1U;
    unsigned count = part == ARM_POSE_YAW_ONLY ? 1U : 3U;
    for (unsigned i = 0U; i < count; ++i) {
        servos[i] = (proto_roboarm_servo_t){(uint8_t)(first + i + 1U), positions[first + i]};
    }
    return app_roboarm_positions_it(servos, (uint8_t)count, duration);
}

/* 相对各自路线停车点的累计位移，X右/Y前，mm；三类均已由用户确认。 */
static const float observation_offsets[WORK_COUNT][3][2] = {
    [WORK_QR] = {{0, 0}, {0, 0}, {0, 0}},
    [WORK_BOMB] = {{0, 0}, {0, 150}, {0, 300}},
    [WORK_TARGET] = {{0, 0}, {0, 150}, {0, 300}},
    [WORK_RESCUE] = {{0, 0}, {0, 150}, {0, 300}}
};

static bool observation_pose(TaskWorkKind_t kind, End_Effector_Pos_t *pose)
{
    /* 视觉 TCP->底盘变换与本次实际下发使用同一组 XYZ/pitch。 */
    Roboarm_Pose_t id = observe_pose_id[kind];
    if (task_arm_poses[id].provided) {
        *pose = task_arm_poses[id].pose;
        return true;
    }
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
        .scan_dwell_ms = 6000U,
        .arm_timeout_ms = 30000U,
        .laser_ms = 3000U,          /* 激光照射时间，后续按实物判定调试 */
        .scan_speed = 100.0f,
        /* 100mm右移、100mm/s、200mm/s^2实测距离稳定；短距离仍需视觉复测。 */
        .correction_speed = 100.0f,
        .correction_accel = 200.0f,
        .accel = 200.0f,
        .max_correction_mm = 200.0f,
        .max_correction_yaw = 0.017453293f * 20.0f, /* 20度；VISION_YAW_NONE时不使用。 */
        .position_tolerance_mm = 10.0f,
        .yaw_tolerance_rad = 0.017453293f * 5.0f,  /* 5度；VISION_YAW_NONE时不使用。 */
        .height_tolerance_mm =30.0f, /* 仅诊断，不阻止底盘 X/Y 微调或作业。 */
        .max_corrections = 5U
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
    bool travel_pose = false;
    ArmPosePart_t pose_part = ARM_POSE_ALL;
    uint16_t clamp_position = CLAMP_OPEN_POSITION;
    Roboarm_Pose_t pose = ROBOARM_POSE_QR_OBSERVE;
    switch (step) {
    case ARM_OBSERVE: {
        const bool split_observe = kind == WORK_BOMB || kind == WORK_TARGET;
        if (arm_work.action >= (split_observe ? 2U : 1U)) return ARM_DONE;
        pose = observe_pose_id[kind];
        if (split_observe) {
            /* 排爆/打靶观察先yaw后关节；人质和QR观察同步发送ID1/2/3/4。 */
            pose_part = arm_work.action == 0U ? ARM_POSE_YAW_ONLY : ARM_POSE_WITHOUT_YAW;
        }
        break;
    }
    case ARM_PICK:
        if (kind != WORK_BOMB && kind != WORK_RESCUE) return ARM_FAILED;
        /* 爆炸物/人质：张爪 -> 夹取位 -> 闭爪 -> 保持夹取yaw抬臂 -> 单独yaw到搬运姿态。 */
        if (arm_work.action >= 5U) return ARM_DONE;
        if (arm_work.action == 0U) {
            clamp_action = true;
        } else if (arm_work.action == 1U) {
            pose = kind == WORK_BOMB ? ROBOARM_POSE_BOMB_PICK : ROBOARM_POSE_RESCUE_PICK;
        } else if (arm_work.action == 2U) {
            clamp_action = true;
            clamp_position = CLAMP_CLOSE_POSITION;
        } else {
            travel_pose = true;
            pose_part = arm_work.action == 3U ? ARM_POSE_WITHOUT_YAW : ARM_POSE_YAW_ONLY;
        }
        break;
    case ARM_PLACE:
        /* 只有爆炸物进入此步骤，且底盘已到第二个爆炸物位置并停稳。 */
        if (kind != WORK_BOMB) return ARM_FAILED;
        /* 原放置姿态拆成：ID2/3/4到位 -> ID1单独转向 -> 张爪。 */
        if (arm_work.action >= 3U) return ARM_DONE;
        if (arm_work.action < 2U) {
            pose = ROBOARM_POSE_BOMB_PLACE;
            pose_part = arm_work.action == 0U ? ARM_POSE_WITHOUT_YAW : ARM_POSE_YAW_ONLY;
        } else {
            clamp_action = true;
        }
        break;
    case ARM_TRAVEL:
        if (arm_work.action >= (kind == WORK_QR ? 1U : 2U)) return ARM_DONE;
        /* 排爆/打靶/救援回搬运姿态：先抬臂，再单独yaw，不修改ID5夹爪状态。 */
        if (kind != WORK_QR) {
            travel_pose = true;
            pose_part = arm_work.action == 0U ? ARM_POSE_WITHOUT_YAW : ARM_POSE_YAW_ONLY;
        }
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
    } else if (travel_pose) {
        result = task_arm_cartesian_send_it(&task_arm_travel_pose, duration, pose_part);
    } else {
        result = task_arm_pose_send_it(pose, duration, pose_part);
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
    /* 直接通过PC15的5V输出使能控制激光：高电平开启，低电平关闭。 */
    HAL_GPIO_WritePin(OUT5V_EN_GPIO_Port, OUT5V_EN_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
