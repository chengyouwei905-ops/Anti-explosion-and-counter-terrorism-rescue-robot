#ifndef __TASK_H_
#define __TASK_H_

#include "app_imu.h"                //调用poll驱动角度数据刷新供closeloop使用
#include "app_chassis.h"            //需要调用poll以驱动getdisance供motion使用
#include "app_chassis_closeloop.h"  //需要调用角度环poll供motion使用
#include "app_chassis_motion.h"     //task主要使用调用函数来源之一，直接控制底盘的闭环移动
#include "app_vision.h"             //task主要使用调用函数来源之一，获取视觉信息，通过视觉辅助作业
#include "app_roboarm.h"            //task主要使用调用函数来源之一，直接控制机械臂的移动
#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum {
    TASK_IDLE,
    TASK_MOVE1,
    TASK_SCAN,
    TASK_MOVE2,
    TASK_MOVE3,
    TASK_MOVE4,
    TASK_MOVE5,
    TASK_MOVE6,
    TASK_MOVE7,
    TASK_MOVE8,
    TASK_MOVE9,
    TASK_MOVE10,
    TASK_MOVE11,
    TASK_EXPLOSIVE,
    TASK_MOVE12,
    TASK_CS,
    TASK_MOVE13,
    TASK_MOVE14,
    TASK_MOVE15,
    TASK_RESCUE,
    TASK_MOVE16,
    TASK_END,
    TASK_FAULT,
} Task_State_t;

/* 逐状态执行赛道试跑；作业点停稳 1 秒，具体作业留空后自动继续。 */
void task_poll(void);
Task_State_t task_get_state(void);

/* 在 app_vision_init() 成功后调用，注册视觉帧业务回调。 */
void task_init(void);

/* 完整合法视觉帧的业务回调，由 task_init() 调用 app_vision_set_rx_cb() 注册。 */
void task_vision_on_frame(const proto_vision_frame_t *frame);

#endif /* __TASK_H_ */
