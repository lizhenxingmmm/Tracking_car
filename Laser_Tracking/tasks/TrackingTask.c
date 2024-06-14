/**
 * @Author: Li Zhenxing
 * @Date: 2024/6/10 01:27:22
 * @LastEditors: Li Zhenxing
 * @LastEditTime: 2024/6/10 01:27:22
 * Description:
 * Copyright: Copyright (©)}) 2024 Li Zhenxing. All rights reserved.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "drv_uart.h"
#include "N10laser.h"
#include <string.h>
#include "crc8_8540.h"
extern volatile uint8_t uart5_rx_buf[2][UART_RX_BUF_LEN];
extern int uart5_rx_data_frame_len;
N10laser N10laser_;
uint32_t error_occur = 0;
uint32_t total_count = 0;
float LossPackRate = 0;
uint16_t forward_angle = 10723; //测出来的
uint16_t start2end = 1200;      //一帧报文扫过的角度
//创建坐标空间
POLAR_COORDINATE polar_coordinate_points[2000];
uint16_t target_index = 0;

void N10laser_decode(volatile uint8_t buf[]);
int partition(POLAR_COORDINATE points[], int low, int high);
void quick_sort(POLAR_COORDINATE points[], int low, int high);
void bubble_sort(POLAR_COORDINATE points[], int low, int high);

void StartTrackingTask(void const *argument)
{
    USART_Init(&huart5, N10laser_decode);
    for (;;)
    {
        //算丢包率
        LossPackRate = (float)error_occur / total_count;
        //整理数据
        // quick_sort(debug_p, 0, 300);
        //递归会出为问题
        bubble_sort(polar_coordinate_points, 0, 1999);
    }
}
void N10laser_decode(volatile uint8_t buf[])
{
    total_count++;
    if (Verify_CRC8_Check_Sum((uint8_t *)buf, uart5_rx_data_frame_len))
    {
        error_occur++;
        return;
    }
    //帧头识别
    if (buf[0] == 0xa5 && buf[1] == 0x5a)
    {
        N10laser_.header1 = buf[0];
        N10laser_.header2 = buf[1];
        N10laser_.frame_len = buf[2];
        N10laser_.period = ((uint16_t)buf[3] << 8) + buf[4];
        N10laser_.start_angle = ((uint16_t)buf[5] << 8) + buf[6];
        N10laser_.end_angle = ((uint16_t)buf[55] << 8) + buf[56];
        for (int i = 0; i < 16; i++)
        {
            N10laser_.pcdt[i].distance = ((uint16_t)buf[7 + 3 * i] << 8) + buf[8 + 3 * i];
            N10laser_.pcdt[i].peak = buf[9 + 3 * i];
            if (N10laser_.pcdt[i].peak > 15) //扫到黑色东西就排除掉
            {
                POLAR_COORDINATE temp_coordinate;
                temp_coordinate.rho = ((float)N10laser_.pcdt[i].distance) / 100;
                temp_coordinate.theta = (float)(N10laser_.start_angle + ((float)i) * start2end / 15) / 100;
                memcpy(&polar_coordinate_points[target_index], &temp_coordinate, sizeof(POLAR_COORDINATE));
                target_index++;
                if (target_index >= 2000)
                {
                    target_index = 0;
                }
            }
        }
        N10laser_.crc_check_sum = buf[57];
    }
}
/**
 * @param low 分区低位索引
 * @param high 高位索引
 * @return  支点（标志位索引）
 * @note high-low为2或3时变成排序，最底层递归
 */
int partition(POLAR_COORDINATE points[], int low, int high)
{
    float pivot = points[high].theta;
    int i = low - 1;
    for (int j = low; j < high; j++)
    {
        if (points[j].theta <= pivot)
        {
            i++;
            POLAR_COORDINATE temp_point;
            //交换位置
            memcpy(&temp_point, &points[i], sizeof(POLAR_COORDINATE));
            memcpy(&points[i], &points[j], sizeof(POLAR_COORDINATE));
            memcpy(&points[j], &temp_point, sizeof(POLAR_COORDINATE));
        }
    }
    POLAR_COORDINATE temp_point;
    memcpy(&temp_point, &points[i + 1], sizeof(POLAR_COORDINATE));
    memcpy(&points[i + 1], &points[high], sizeof(POLAR_COORDINATE));
    memcpy(&points[high], &temp_point, sizeof(POLAR_COORDINATE));
    return i + 1;
}
/**
 * @brief 排序程序，递归排序，结果为升序
 * @param low 0
 * @param high len-1
 */
void quick_sort(POLAR_COORDINATE points[], int low, int high)
{
    if (low < high)
    {
        int pivot = partition(points, low, high);
        quick_sort(points, low, pivot - 1);
        quick_sort(points, pivot + 1, high);
    }
    else
    {
        return;
    }
}
/**
 * @param low 分区低位索引
 * @param high 高位索引
 */
void bubble_sort(POLAR_COORDINATE points[], int low, int high)
{
    POLAR_COORDINATE temp_point;
    for (int i = 0; i <= high - 1; i++)
    {
        for (int j = 0; j <= high - 1 - i; j++)
        {
            if (points[j].theta > points[j + 1].theta)
            {
                memcpy(&temp_point, &points[j + 1], sizeof(POLAR_COORDINATE));
                memcpy(&points[j + 1], &points[j], sizeof(POLAR_COORDINATE));
                memcpy(&points[j], &temp_point, sizeof(POLAR_COORDINATE));
            }
        }
    }
}
