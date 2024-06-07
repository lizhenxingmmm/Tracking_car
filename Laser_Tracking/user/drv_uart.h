#ifndef DRV_UART_H
#define DRV_UART_H

#include "main.h"
#include "usart.h"
#include "dma.h"

void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
void USART_Init(UART_HandleTypeDef *huart);

#endif
