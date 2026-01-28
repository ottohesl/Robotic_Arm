#ifndef OTTOHESL_H
#define OTTOHESL_H
#define OTTOHESL_H_Vision 7
#include "stdint.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#if (OTTOHESL_H_Vision==1)
#include "stm32f1xx_hal.h"
#endif
#if (OTTOHESL_H_Vision==4)
#include "stm32f4xx_hal.h"
#endif
#if (OTTOHESL_H_Vision==7)
#include "stm32h7xx_hal.h"
#endif



#define OTTOHESL_UART_BUFFER 256


void OTTO_uart(UART_HandleTypeDef *huart, const char *fmt, ...);
void OTTO_uart_dma(UART_HandleTypeDef *huart, const char *fmt, ...);
void uart_debugger(UART_HandleTypeDef *huart,HAL_StatusTypeDef text);
#endif //OTTOHESL_H
