#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "stm32f0xx_hal.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

// Macro để in nhanh
#define UART_LOG(fmt, ...) do { \
    char buf[128]; \
    sprintf(buf, fmt, ##__VA_ARGS__); \
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY); \
} while (0)
#endif
