#ifndef __ALG_CHASSIS_KINEMATICS_H__
#define __ALG_CHASSIS_KINEMATICS_H__

#include "stm32h7xx_hal.h"

#define WHEEL_NUMBERS 4
#define CHASSIS_K_MM 105.0f  /* 底盘几何参数*/

typedef enum{
    LEFT_FRONT_WHELL = 0,
    RIGHT_FRONT_WHELL,
    LEFT_REAR_WHELL,
    RIGHT_REAR_WHELL
} Wheel_ID_t;

/* 以向前为y正方向，向右为x正方向 */
/* 直接就是mm/s，bsp层已做速度单位换算 */
typedef struct{
    float speed_x;
    float speed_y;
    float omega;
} Chassis_State_t;

void alg_chassis_invert(const Chassis_State_t *state, float wheel_speed[WHEEL_NUMBERS]);
void alg_chassis_forward(const float wheel_speed[WHEEL_NUMBERS], Chassis_State_t *state);

#endif /* __ALG_CHASSIS_KINEMATICS_H__ */
