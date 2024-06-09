/**
 * @Author: Li Zhenxing
 * @Date: 2024/6/10 01:27:50
 * @LastEditors: Li Zhenxing
 * @LastEditTime: 2024/6/10 01:27:50
 * Description: 
 * Copyright: Copyright (©)}) 2024 Li Zhenxing. All rights reserved.
 */
#ifndef DRV_UART_H
#define DRV_UART_H

#include "main.h"
#include "usart.h"
#include "dma.h"

//默认所有uart接收内存为2x200
#define UART_RX_BUF_LEN (200)

void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
void USART_Init(UART_HandleTypeDef *huart, void (*decode_func)(volatile uint8_t[]));

#endif
