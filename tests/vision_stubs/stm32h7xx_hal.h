#ifndef TEST_VISION_HAL_H
#define TEST_VISION_HAL_H

#include "../uart_stubs/stm32h7xx_hal.h"

typedef struct { uint32_t unused; } SPI_HandleTypeDef;
uint32_t HAL_GetTick(void);

#endif
