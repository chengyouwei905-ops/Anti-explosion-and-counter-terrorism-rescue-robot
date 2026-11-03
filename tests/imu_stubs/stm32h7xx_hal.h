#ifndef TEST_IMU_HAL_H
#define TEST_IMU_HAL_H

#include <stdint.h>

typedef struct { uint32_t unused; } SPI_HandleTypeDef;
void HAL_Delay(uint32_t ms);

extern uint32_t test_imu_primask;
static inline uint32_t __get_PRIMASK(void) { return test_imu_primask; }
static inline void __disable_irq(void) { test_imu_primask = 1U; }
static inline void __set_PRIMASK(uint32_t value) { test_imu_primask = value; }

#endif
