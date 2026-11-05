#ifndef APP_TASK_WORK_H
#define APP_TASK_WORK_H

#include "alg_vision_align.h"
#include <stdint.h>

/* 摄像头故障时的MCU联调：1=模拟视觉成功/选第1位/不纠偏。
 * 摄像头恢复后改为0并重新编译烧录。启停仍由真实TOTALSTART/TOTALSTOP控制。
 */
#ifndef TASK_WORK_CAMERA_TEST
#define TASK_WORK_CAMERA_TEST 0
#endif

typedef enum { WORK_QR, WORK_BOMB, WORK_TARGET, WORK_RESCUE, WORK_COUNT } TaskWorkKind_t;
typedef enum { WORK_WAITING, WORK_DONE, WORK_FAILED } TaskWorkResult_t;
typedef enum { ARM_OBSERVE, ARM_PICK, ARM_PLACE, ARM_TRAVEL, ARM_CANCEL } TaskArmStep_t;
typedef enum { ARM_WAITING, ARM_DONE, ARM_FAILED } TaskArmResult_t;

typedef struct {
    /* 相对该作业区路线停车点，沿停车时底盘 X右/Y前 的三个观察位置。
     * 数组顺序就是MCU停车位1/2/3；停稳后逐位识别，命中目标即结束扫描。
     * QR 只使用 slot[0]，不执行扫描。
     */
    struct {
        float offset_mm[2];
        End_Effector_Pos_t observe_pose; /* 机械臂指令 XYZ/pitch；位置先留空 */
        VisionAlignmentConfig_t alignment;
    } slot[3];
    uint32_t settle_ms;
    uint32_t scan_dwell_ms; /* 每个停车位的K230识别窗口，ms；非盲目等待/移动计时。 */
    uint32_t arm_timeout_ms;
    uint32_t laser_ms;
    float scan_speed, correction_speed, accel; /* accel用于扫描/换位/返回 */
    float correction_accel; /* 视觉平移纠偏独立加减速度，mm/s^2 */
    /* yaw限幅/容差仅对启用视觉yaw纠偏的slot生效；VISION_YAW_NONE时忽略。 */
    float max_correction_mm, max_correction_yaw;
    float position_tolerance_mm, yaw_tolerance_rad;
    float height_tolerance_mm; /* 高度诊断阈值，不参与底盘到位判定。 */
    uint8_t max_corrections;
} TaskWorkConfig_t;

typedef enum {
    WORK_IDLE, WORK_WAIT_CONFIG, WORK_MOVE_SLOT, WORK_WAIT_MOTION,
    WORK_PREPARE_ARM, WORK_SETTLE, WORK_SEND_SCAN, WORK_SCAN_DWELL,
    WORK_WAIT_SCAN, WORK_SELECT_SLOT, WORK_SEND_LOCATE, WORK_WAIT_LOCATE,
    WORK_ALIGN, WORK_PICK, WORK_PLACE, WORK_LASER, WORK_TRAVEL_ARM,
    WORK_RETURN, WORK_COMPLETE, WORK_ERROR,
    WORK_MOVE_TO_BUCKET, /* 夹球并抬臂后，移到第二个爆炸物位置 */
    WORK_RETURN_YAW /* 返回扫描起点位置后恢复进入扫描时的航向 */
} TaskWorkPhase_t;
typedef enum {
    WORK_ERR_NONE, WORK_ERR_CONFIG, WORK_ERR_TX, WORK_ERR_TIMEOUT,
    WORK_ERR_VISION, WORK_ERR_DATA, WORK_ERR_IMU, WORK_ERR_MOTION,
    WORK_ERR_ALIGNMENT, WORK_ERR_ARM, WORK_ERR_OBSERVATION_MOVED
} TaskWorkError_t;

typedef struct {
    TaskWorkKind_t kind;
    TaskWorkPhase_t phase;
    TaskWorkError_t error;
    uint8_t motion_start_result; /* Chassis_Motion_Start_t，保留里程纠偏启动失败原因 */
    uint8_t motion_result;       /* Chassis_Motion_Result_t，保留取消前的反馈/停滞错误 */
    uint8_t vision_reason; /* 原样保留 K230 reason，9=MODEL_NOT_READY */
    uint8_t selected_slot;
    uint8_t corrections;
    bool request_pending;
    uint8_t request_cmd, request_seq;
    bool alignment_valid;
    VisionAlignment_t alignment; /* 观测时刻的估计；移动/搬动物体后失效 */
    /* 最近有效观测的高度误差；与 alignment 一同保留供调试，不阻止平面修正。 */
    float height_error_mm;
    bool height_out_of_tolerance;
    bool cancel_pending;
    bool cancel_failed;
} TaskWorkStatus_t;

void app_task_work_init(void);
TaskWorkResult_t app_task_work_poll(TaskWorkKind_t kind, uint32_t now);
void app_task_work_on_frame(const proto_vision_frame_t *frame);
/* 立即清除请求与结果、关闭激光；异步取消报文由 service 重试，无阻塞等待。 */
void app_task_work_cancel(uint32_t now);
void app_task_work_service(uint32_t now);
bool app_task_work_cancel_pending(void);
const TaskWorkStatus_t *app_task_work_get_status(void);

/* 用户调试入口，实现在 app_task_work_port.c。没有机构参数时返回 false 并等待。
 * 机械臂 hook 每次 poll 调用；entering 只在该步骤的第一次为 true。
 * hook 必须非阻塞；DONE 必须表示已停稳/动作完成，不能仅表示指令已发出。
 */
bool task_work_load_config(TaskWorkKind_t kind, TaskWorkConfig_t *out);
TaskArmResult_t task_work_arm_poll(TaskWorkKind_t kind, TaskArmStep_t step,
                                  uint8_t slot, bool entering);
void task_work_laser_set(bool on);
int task_work_arm_stop_it(void);
#endif
