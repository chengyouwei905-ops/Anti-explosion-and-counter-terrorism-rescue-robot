#ifndef TEST_IMU_HAL_H
#define TEST_IMU_HAL_H

#include <stdint.h>

typedef struct { uint32_t unused; } SPI_HandleTypeDef;
void HAL_Delay(uint32_t ms);

#endif
