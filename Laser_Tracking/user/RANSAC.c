#include "RANSAC.h"

void RANSAC_Init(RANSAC_def *ransac_def, MODEL_TYPE model_type, float error_tolerant, float priori_w, float accuracy_you_want, float threshold)
{
    ransac_def->model_type = model_type;
    ransac_def->error_tolerant = error_tolerant;
    ransac_def->priori_w = priori_w;
    ransac_def->accuracy_you_want = accuracy_you_want;
    ransac_def->threshold = threshold;
}
CIRCLE_def debug_circle;
int debug_random_num[3];
RANSAC_STATE RANSAC_iteration_circle(RANSAC_def *ransac_def, CARTESIANCOORDINATE *xy, int dot_num)
{
    int num_zero;
    if (xy == 0) //空指针
    {
        return RANSAC_ERROR;
    }
    for (int i = 0; i < dot_num; i++)
    {
        if (xy[i].x == 0 && xy[i].y == 0)
        {
            num_zero++;
        }
    }
    if (num_zero > dot_num - 10)
    {
        return RANSAC_ERROR;
    }
    if (ransac_def->model_type == CIRCLE)
    {
        float b = powf(ransac_def->priori_w, 3);
        ransac_def->iteration_num = log(1 - ransac_def->accuracy_you_want) / log(1 - b);
        for (int i = 0; i < (ransac_def->iteration_num); i++)
        {
            CARTESIANCOORDINATE sample_dots[3]; //数据点
            CIRCLE_def fitting_circle;          //拟合圆
            int random_num[3];                  //随机数
            int inner_dot_num = 0;              //写了个逆天bug,好像这个数每次分配都会用相同的内存，必须置零！
            GetRandomArray(random_num, 3, 1, 0, dot_num);
            for (int i = 0; i < 3; i++)
            {
                sample_dots[i] = xy[random_num[i]];
            }
            //笛卡尔坐标系拟合圆
            GetCircle(sample_dots, &fitting_circle);
            while (isnan(fitting_circle.origin.x) || isnan(fitting_circle.origin.x))
            {
                GetRandomArray(random_num, 3, 1, 0, dot_num);
                for (int i = 0; i < 3; i++)
                {
                    random_num[i] += 1;
                    if (random_num[i] >= dot_num)
                    {
                        random_num[i] -= dot_num;
                    }
                    sample_dots[i] = xy[random_num[i]];
                }
                //笛卡尔坐标系拟合圆
                GetCircle(sample_dots, &fitting_circle);
            }
            for (int j = 0; j < dot_num; j++)
            {
                if (j == random_num[0] || j == random_num[1] || j == random_num[2])
                {
                    ; //不做处理
                }
                else
                {
                    if (fabs(GetDistance(&(fitting_circle.origin), &xy[j]) - fitting_circle.radius) <= (ransac_def->error_tolerant))
                    {
                        inner_dot_num++;
                    }
                }
            }
            debug_circle = fitting_circle;
            if (inner_dot_num > (ransac_def->threshold))
            {
                ransac_def->target_circle = fitting_circle;
                return RANSAC_OK; //迭代结束
            }
        }
        return RANSAC_NO_RESULT; //没有得到想要的模型
    }
    else
    {
        return RANSAC_ERROR;
    }
}

/**
 * @return 返回1则成功，0不成功
 */
uint8_t GetCircle(CARTESIANCOORDINATE xy[3], CIRCLE_def *circle)
{
    float x1, y1, x2, y2, x3, y3;
    x1 = xy[0].x;
    x2 = xy[1].x;
    x3 = xy[2].x;
    y1 = xy[0].y;
    y2 = xy[1].y;
    y3 = xy[2].y;
    float a = x1 * (y2 - y3) - y1 * (x2 - x3) + (x2 * y3 - x3 * y2);
    float b = (x1 * x1 + y1 * y1) * (y3 - y2) + (x2 * x2 + y2 * y2) * (y1 - y3) + (x3 * x3 + y3 * y3) * (y2 - y1);
    float c = (x1 * x1 + y1 * y1) * (x2 - x3) + (x2 * x2 + y2 * y2) * (x3 - x1) + (x3 * x3 + y3 * y3) * (x1 - x2);
    float d = (x1 * x1 + y1 * y1) * (x3 * y2 - x2 * y3) + (x2 * x2 + y2 * y2) * (x1 * y3 - x3 * y1) + (x3 * x3 + y3 * y3) * (x2 * y1 - x1 * y2);
    circle->origin.x = -b / (2 * a);
    circle->origin.y = -c / (2 * a);
    arm_sqrt_f32((b * b + c * c - 4 * a * d) / (4 * a * a), &(circle->radius));
    return 1;
}

float GetDistance(CARTESIANCOORDINATE *start, CARTESIANCOORDINATE *end)
{
    float distance;
    arm_sqrt_f32(((start->x - end->x) * (start->x - end->x) + (start->y - end->y) * (start->y - end->y)), &distance);
    return distance;
}

/**
 * @param type 1不允许重复 0允许重复
 */
void GetRandomArray(int num[], int len, int type, int min, int max)
{
    //防傻
    if (min > max)
    {
        return;
    }
    if (type == 0)
    {
        for (int i = 0; i < len; i++)
        {
            num[i] = min + rand() % (max - min);
        }
    }
    if (type == 1)
    {
        if ((max - min) < len - 1)
        {
            return;
        }
        for (int i = 0; i < len; i++)
        {
            num[i] = min + rand() % (max - min);
            int is_repeat = 0;
            for (int j = 0; j < i; j++)
            {
                if (num[i] == num[j])
                {
                    is_repeat = 1;
                    break;
                }
            }
            if (is_repeat)
            {
                i--; //重复了就再来一次
            }
        }
    }
}
