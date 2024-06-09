#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "drv_uart.h"
#include "N10laser.h"
#include <string.h>
extern volatile uint8_t uart5_rx_buf[2][UART_RX_BUF_LEN];
extern int uart5_rx_data_frame_len;
N10laser N10laser_;

void N10laser_decode(volatile uint8_t buf[]);

extern void StartTrackingTask(void const *argument)
{
    USART_Init(&huart5, N10laser_decode);
    for (;;)
    {
    }
}

void N10laser_decode(volatile uint8_t buf[])
{
    //帧头识别
    if (buf[0] == 0xa5 && buf[1] == 0x5a)
    {
        N10laser_.header1 = buf[0];
        N10laser_.header2 = buf[1];
        N10laser_.frame_len = buf[2];
        N10laser_.period = ((uint16_t)buf[3] << 8) + buf[4];
        N10laser_.start_angle = ((uint16_t)buf[5] << 8) + buf[6];
        for (int i = 0; i < 16; i++)
        {
            N10laser_.pcdt[i].distance = ((uint16_t)buf[7 + 3 * i] << 8) + buf[8 + 3 * i];
            N10laser_.pcdt[i].peak = buf[9 + 3 * i];
        }
        N10laser_.end_angle = ((uint16_t)buf[55] << 8) + buf[56];
        N10laser_.crc_check_sum = buf[57];
    }
}
