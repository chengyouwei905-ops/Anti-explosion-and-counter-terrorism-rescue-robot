#ifndef TEST_UART_HAL_H
#define TEST_UART_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_BUSY_TX 0x21U
#define HAL_UART_ERROR_NONE 0U
typedef struct {
    volatile uint32_t gState;
    uint32_t ErrorCode;
    const uint8_t *test_tx_ptr;
    uint16_t test_tx_len;
} UART_HandleTypeDef;
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
#define __HAL_UART_CLEAR_FEFLAG(h) ((void)(h))
#define __HAL_UART_CLEAR_NEFLAG(h) ((void)(h))
#define __HAL_UART_CLEAR_PEFLAG(h) ((void)(h))
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
void HAL_Delay(uint32_t delay);
#endif
