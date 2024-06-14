#ifndef N10LASER_H
#define N10LASER_H

typedef __packed struct
{
    unsigned short distance; //距离
    unsigned char peak;      //强度
} PointCloudData;

typedef __packed struct
{
    unsigned char header1;
    unsigned char header2;
    unsigned char frame_len;
    unsigned short period; //码盘转1/24圈的时间 单位us
    unsigned short start_angle;
    PointCloudData pcdt[16];
    unsigned short end_angle;
    unsigned char crc_check_sum;
} N10laser;

typedef __packed struct
{
    float rho;   //单位毫米
    float theta; //单位°
} POLAR_COORDINATE;

#endif
