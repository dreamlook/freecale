#include "common.h"
#include "include.h"

struct servo_pd
{
    int p;
    int d;
}servopd;

struct servo_error
{
    int present;
    int last;
    int befor;
}error;

struct servo_limit
{
    uint16 max;
    uint16 min;
}limit;

extern uint16 adcm2;


/********************舵机角度控制***********************/
//功能：根据信标的位置进行舵机的角度控制
//传入参数：信标的所在位置的列数
//返回值：舵机的pwm占空比

int16 servo_angle(uint16 y)
{

    uint16 y1=0,y2=0;
    int16 duty,newduty;

    y1 = y-0;
    y2 = adcm2-y;
    error.present = y1-y2;
    

    duty = (int)(servopd.p * error.present+ servopd.d * (error.present - error.last));

    error.befor = error.last;
    error.last = error.present;

    newduty = 1450 + duty;

    if(newduty > limit.max)     newduty = limit.max;
    if(newduty < limit.min)     newduty = limit.min;

    return newduty;
}




