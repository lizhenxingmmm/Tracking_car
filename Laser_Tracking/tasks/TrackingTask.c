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

extern void StartTrackingTask(void const *argument)
{
    for (;;)
    {
        memcpy(&N10laser_,(const void*) &uart5_rx_buf[0][UART_RX_BUF_LEN], uart5_rx_data_frame_len);
    }
}
