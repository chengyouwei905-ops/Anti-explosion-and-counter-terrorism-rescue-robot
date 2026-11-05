#ifndef TEST_HWT101_HAL_H
#define TEST_HWT101_HAL_H
#include "../uart_stubs/stm32h7xx_hal.h"
extern uint32_t test_hwt101_primask;
static inline uint32_t __get_PRIMASK(void) { return test_hwt101_primask; }
static inline void __disable_irq(void) { test_hwt101_primask = 1U; }
static inline void __set_PRIMASK(uint32_t value) { test_hwt101_primask = value; }
#endif
