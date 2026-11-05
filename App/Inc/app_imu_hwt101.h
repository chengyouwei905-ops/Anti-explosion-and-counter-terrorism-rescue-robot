/** @brief HWT101航向：UART8中断写BSP环形缓冲区，poll解包并发布快照。 */
#ifndef __APP_IMU_HWT101_H
#define __APP_IMU_HWT101_H

#include <stdbool.h>
#include "bsp_uart.h"
#include "common_def.h"

/* 默认HWT101 TTL：115200/8N1。UART配置由Core负责，不在这里改硬件。
 * 默认10Hz输出也可接收；底盘100Hz控制建议先用维特上位机设为100Hz。 */
#ifndef APP_IMU_HWT101_TIMEOUT_MS
#define APP_IMU_HWT101_TIMEOUT_MS 500U
#endif

/* 当前实车倒装，统一取反后对外仍为逆时针正；正装时改为1.0。 */
#ifndef APP_IMU_HWT101_YAW_SIGN
#define APP_IMU_HWT101_YAW_SIGN (-1.0)
#endif

int app_imu_hwt101_init(UART_HandleTypeDef *huart);
/* 唯一写者，主循环持续调用。RET_OK=更新航向，RET_IDLE=无新角度，
 * RET_TIMEOUT=数据过期/轮询停顿，RET_ERROR=RX溢出，RET_NOTREADY=未初始化。
 * 每次最多处理进入poll时已缓存的字节，不等待串口，不发配置指令。 */
int app_imu_hwt101_poll(void);
/* 连续弧度角，以init后第一帧为0；跨±180度累计，不归一化目标角。
 * 未收到角度/数据过期/溢出时返回false并置0。主循环/普通中断可读，
 * 短临界区复制double并恢复PRIMASK；NMI/HardFault不要调用。
 * 两个有效帧间实际旋转必须小于180度；丢失多圈不能由单圈角恢复。 */
bool app_imu_hwt101_getangle(double *angle_rad);

#endif /* __APP_IMU_HWT101_H */
