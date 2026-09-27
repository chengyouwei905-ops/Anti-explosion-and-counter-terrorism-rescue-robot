#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"      /* UART_HandleTypeDef 定义在这里 */
#include "proto_chassis.h"
#include "alg_chassis_kinematics.h"

int app_chassis_init(UART_HandleTypeDef *huart);
int app_chassis_move(float speed_x, float speed_y, float omega);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CHASSIS_H */
