#ifndef __APP_CHASSIS_CLOSELOOP_H
#define __APP_CHASSIS_CLOSELOOP_H

#define APP_CHASSIS_MAX_OMEGA_RADPS 1.2f

typedef struct {
    float speed_x;  /* 车体向右速度，mm/s */
    float speed_y;  /* 车体向前速度，mm/s */
    float yaw;      /* 绝对连续航向，rad，逆时针为正 */
    float omega_ff; /* 角速度前馈，rad/s */
} Chassis_CloseLoop_State_t;

/* 初始化后默认目标为零平移速度、0 rad航向，可直接周期调用update。 */
void app_chassis_closeloop_init(void);
/* 保持航向：同时将角速度前馈清零。 */
void app_chassis_closeloop_set_target(float vx, float vy, float yawangle);
/* 跟踪运动中的航向，同时使用规划角速度前馈。 */
void app_chassis_closeloop_set_target_ff(float vx, float vy, float yawangle, float omega_ff);
/* 主循环按控制节拍调用；速度帧由中断发送，串口忙时留待下一控制节拍发送最新目标。 */
void app_chassis_closeloop_update(void);

#endif /* __APP_CHASSIS_CLOSELOOP_H */
