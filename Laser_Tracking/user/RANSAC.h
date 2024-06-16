#ifndef RANSAC_H
#define RANSAC_H

#include "arm_math.h" //求2次根号等超越函数更快
#include <math.h>
#include <stdlib.h>
#include <time.h>

typedef enum
{
    LINE,
    CIRCLE
} MODEL_TYPE;

typedef enum
{
    RANSAC_ERROR = 0,
    RANSAC_OK = 1,
    RANSAC_NO_RESULT = 2 //没有满意的模型
} RANSAC_STATE;

typedef struct
{
    float x;
    float y;
} CARTESIANCOORDINATE;

typedef struct
{
    float radius;
    CARTESIANCOORDINATE origin;
} CIRCLE_def;

typedef struct
{
    MODEL_TYPE model_type;
    float error_tolerant;    //容限误差
    float priori_w;          //先验的理想模型的内点率
    float accuracy_you_want; //越准越慢 0%~100%
    float threshold;         //内点数量判准
    int iteration_num;       //迭代次数
    //目标模型
    CIRCLE_def target_circle;

} RANSAC_def;

void GetRandomArray(int num[], int len, int type, int min, int max);
void GetCircle(CARTESIANCOORDINATE xy[3], CIRCLE_def *circle);
float GetDistance(CARTESIANCOORDINATE *start, CARTESIANCOORDINATE *end);
RANSAC_STATE RANSAC_iteration_circle(RANSAC_def *ransac_def, CARTESIANCOORDINATE *xy, int dot_num);
void RANSAC_Init(RANSAC_def *ransac_def, MODEL_TYPE model_type, float error_tolerant, float priori_w, float accuracy_you_want, float threshold);

#endif
