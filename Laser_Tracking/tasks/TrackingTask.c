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
#include "bsp_dwt.h"
#include <math.h>
#include "arm_math.h"

#define RAW_POINTS_DATA_SPACE (1000)
#define FITERED_POINTS_DATA_SPACE (180)

float d_theta = 2 * PI / FITERED_POINTS_DATA_SPACE;

extern volatile uint8_t uart5_rx_buf[2][UART_RX_BUF_LEN];
extern int uart5_rx_data_frame_len;
N10laser N10laser_;
uint32_t error_occur = 0;
uint32_t total_count = 0;
float LossPackRate = 0;
uint16_t forward_angle = 10723; //测出来的
uint16_t start2end;             //一帧报文扫过的角度
//创建坐标空间
//两个空间，防止排序中写入数据导致数据更新出问题
POLAR_COORDINATE polar_coordinate_points[RAW_POINTS_DATA_SPACE]; //未排序坐标空间
POLAR_COORDINATE sorted_points[RAW_POINTS_DATA_SPACE];           //已经排序坐标空间
uint16_t target_index = 0;
POLAR_COORDINATE modified_points[FITERED_POINTS_DATA_SPACE];
float cuvatures[FITERED_POINTS_DATA_SPACE] = {0, 0, 0, 0, 0, 0, 0, 0};

void N10laser_decode(volatile uint8_t buf[]);
int partition(POLAR_COORDINATE points[], int low, int high);
void quick_sort(POLAR_COORDINATE points[], int low, int high);
void bubble_sort(POLAR_COORDINATE points[], int low, int high);
void filter_point(POLAR_COORDINATE sorted_point[], POLAR_COORDINATE modified_points[]);
float GetCuvature(POLAR_COORDINATE *current_point, POLAR_COORDINATE *next_point, POLAR_COORDINATE *next_next_point);

uint32_t timeline;
double dt;

void StartTrackingTask(void const *argument)
{
    USART_Init(&huart5, N10laser_decode);
    DWT_Init(168);
    for (;;)
    {
        //算丢包率
        LossPackRate = (float)error_occur / total_count;
        //整理数据
        // quick_sort(debug_p, 0, 300);
        //递归会出为问题
        memcpy(&sorted_points[0], &polar_coordinate_points[0], RAW_POINTS_DATA_SPACE * sizeof(POLAR_COORDINATE));
        //好像不用排序，难绷
        bubble_sort(sorted_points, 0, 999); //约0.07s更新一次
        filter_point(sorted_points, modified_points);
        //算各点曲率
        for (int i = 0; i < FITERED_POINTS_DATA_SPACE; i++)
        {
            if (i == FITERED_POINTS_DATA_SPACE - 1 - 1)
            {
                cuvatures[i] = GetCuvature(&modified_points[i], &modified_points[i + 1], &modified_points[0]);
            }
            if (i == FITERED_POINTS_DATA_SPACE - 1)
            {
                cuvatures[i] = GetCuvature(&modified_points[i], &modified_points[0], &modified_points[1]);
            }
            else
            {
                cuvatures[i] = GetCuvature(&modified_points[i], &modified_points[i + 1], &modified_points[i + 2]);
            }
        }

        dt = DWT_GetDeltaT(&timeline);
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
        if (N10laser_.end_angle > N10laser_.start_angle)
            start2end = N10laser_.end_angle - N10laser_.start_angle;
        else
            start2end = N10laser_.end_angle + 36000 - N10laser_.start_angle;
        for (int i = 0; i < 16; i++)
        {
            N10laser_.pcdt[i].distance = ((uint16_t)buf[7 + 3 * i] << 8) + buf[8 + 3 * i];
            N10laser_.pcdt[i].peak = buf[9 + 3 * i];
            if (N10laser_.pcdt[i].peak > 15) //扫到黑色东西就排除掉
            {
                POLAR_COORDINATE temp_coordinate;
                temp_coordinate.rho = ((float)N10laser_.pcdt[i].distance) / 100;
                temp_coordinate.theta = (float)(N10laser_.start_angle + ((float)i) * start2end / 15) / 100;
                if (temp_coordinate.theta > 360)
                {
                    temp_coordinate.theta -= 360;
                }
                memcpy(&polar_coordinate_points[target_index], &temp_coordinate, sizeof(POLAR_COORDINATE));
                target_index++;
                if (target_index >= 1000)
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

void filter_point(POLAR_COORDINATE sorted_points_[], POLAR_COORDINATE modified_pts[])
{
    int num[FITERED_POINTS_DATA_SPACE] = {0, 0, 0, 0};
    int index_ = 360 / FITERED_POINTS_DATA_SPACE;
    for (int i = 0; i < RAW_POINTS_DATA_SPACE; i++)
    {
        for (int j = 0; j < FITERED_POINTS_DATA_SPACE / index_; j++)
        {
            if (fabs(sorted_points_[i].theta - (j + 0.5) * index_) < 0.5 * index_ || sorted_points_[i].theta == j * index_)
            {
                num[j]++;
                modified_pts[j].rho += sorted_points_[i].rho;
            }
        }
    }
    for (int i = 0; i < FITERED_POINTS_DATA_SPACE / index_; i++)
    {
        modified_pts[i].theta = i * index_;
        if (num[i] == 0)
        {
            modified_pts[i].rho = 0;
        }
        else
        {
            modified_pts[i].rho /= num[i];
        }
    }
}

//曲率公式：cuvature=d(theta)/ds cuvature=|(dy/dx)**2|/((d(dy/dx)/dx)**2+1)**1.5
//极坐标曲率公式 对于r=f(theta) cuvature=(r**2+2r.**2-rr..)/(r**2+r.**2)**1.5
float GetCuvature(POLAR_COORDINATE *current_point, POLAR_COORDINATE *next_point, POLAR_COORDINATE *next_next_point)
{
    float r = current_point->rho;
    float r_dot = (next_point->rho - current_point->rho) / d_theta;
    float r_dot_dot = ((next_next_point->rho - next_point->rho) / d_theta - r_dot) / d_theta;
    float temp2;
    float temp1 = (r * r + r_dot * r_dot);
    arm_sqrt_f32(temp1, &temp2);
    if (temp1 == 0)
    {
        return 0;
    }
    return (r * r + 2 * r_dot * r_dot - r * r_dot_dot) / (temp1 * temp2);
}
