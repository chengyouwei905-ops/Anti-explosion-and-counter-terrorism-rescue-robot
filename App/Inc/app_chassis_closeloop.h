#ifndef __APP_CHASSIS_CLOSELOOP_H
#define __APP_CHASSIS_CLOSELOOP_H

#include "stm32h7xx_hal.h"
#include "app_chassis.h"
#include "app_imu.h"
#include "alg_pid.h"
#include <math.h>

//业务闭环层状态参数
typedef struct{
    float speed_x;
    float speed_y;
    float yaw;
} Chassis_CloseLoop_State_t;

void app_chassis_closeloop_init(void);
void app_chassis_closeloop_set_target(float_t vx, float_t vy,float_t yawangle);
void app_chassis_closeloop_update(void);

#endif /* __APP_CHASSIS_CLOSELOOP_H */
