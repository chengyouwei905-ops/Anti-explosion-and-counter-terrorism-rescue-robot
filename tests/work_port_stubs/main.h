#ifndef TEST_WORK_PORT_MAIN_H
#define TEST_WORK_PORT_MAIN_H
#include "stm32h7xx_hal.h"
#define LASER_GPIO_Port ((void *)1)
#define LASER_Pin 1U
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state);
#endif
