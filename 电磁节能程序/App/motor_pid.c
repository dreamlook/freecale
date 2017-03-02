#include "common.h"
#include "include.h"


enum
{
    PID_START,              //开始PID调节
    PID_STOP,               //禁止PID调节
} pid_status;

//PID_STATUS  pid_status;

struct MOTOR_LIMIT
{
    int16 m_duty_max;
    int16 m_duty_min;
    uint16 m_duty_forward_max;
    uint16 m_duty_forward_min;
    uint16 m_duty_backward_max;
    uint16 m_duty_backward_min;
}motor_limit;

struct MOTOR_PID
{
    float Kp;
    float Ki;
    float Kd;
    float expect_speed;
    float real_speed;
    float error_befor;
    float error_last;
    float error_present;
    float error_min;
    float duty_last;
}motor_pid;

//float var[6];
int16 motor_duty = 0;
//int16 motor_duty_forward = 0;
//int16 motor_duty_backward = 0;
extern uint16 motor_target;

void PID_init(void)
{
    motor_pid.Kp = 15;    //5   //5    //3.6   //2.9  //20                 
    motor_pid.Ki = 1.5;     //0.9 //2.0  //1.5   //1.5  //0.5        
    motor_pid.Kd = 0.0;   //1.0 //1.0  //0.8   //0.8  //0     
    motor_pid.error_min = 10;
    motor_pid.error_present = 0;
    motor_pid.error_last = 0;
    motor_pid.error_befor = 0;
    motor_duty = 0;
}

void speed_pid(float set_speed,float back_speed)
{
    float P = 0,I = 0,D = 0;
    
    motor_pid.error_befor = motor_pid.error_last;
    motor_pid.error_last = motor_pid.error_present;
    motor_pid.error_present = set_speed - back_speed;

    P = motor_pid.Kp * (motor_pid.error_present - motor_pid.error_last);
    I = motor_pid.Ki * motor_pid.error_present;
    D = motor_pid.Kd * (motor_pid.error_present - 2 * motor_pid.error_last + motor_pid.error_befor);
  
    motor_duty += (P + I + D);
    
    if(motor_duty > 9000)
        motor_duty = 9000;
    if(motor_duty < -9000)
        motor_duty = -9000;
    
    if(motor_duty < 0)
    {
        ftm_pwm_duty(FTM3, FTM_CH1, 0);
        ftm_pwm_duty(FTM3, FTM_CH3, (uint16)(-(motor_duty)));
    }
    else 
    {
        ftm_pwm_duty(FTM3, FTM_CH1, (uint16)(motor_duty));
        ftm_pwm_duty(FTM3, FTM_CH3, 0);
    }

/**************************山外示波器*************************/    
    //var[0] = set_speed;            //期望
    //var[1] = back_speed;              //实际
    //var[2] = (float)(motor_duty);                               //占空比
    //var[3] = P;           //P调节         
    //var[4] = I;              //I调节
    //var[5] = D;             //D调节
//    vcan_sendware(var, sizeof(var));

/*************************VisualScope**********************/
//    OutData[0] = motor_pid.expect_speed;             
//    OutData[1] = motor_pid.real_speed;
//    OutData[2] = motor_duty_forward;
//    OutData[3] = motor_duty_backward;
//    OutPut_Data();
//    printf("%f %f %f\r\n",motor_pid.error_present,motor_pid.error_last,motor_pid.error_befor);
}

/**********************角度角度分区************************/
//功能：根据舵机角度控制电机速度
//传入参数：舵机角度
//返回值：无

void angle_speed(uint16 servo_duty)
{
    if((servo_duty>=3500) && (servo_duty<4000))
        motor_target = 80;
    if(((servo_duty>=3000) && (servo_duty<3500)) || ((servo_duty>=4000) && (servo_duty<4500)))
        motor_target = 70;
    if(((servo_duty>=2500) && (servo_duty<3000)) || ((servo_duty>=4500) && (servo_duty<5000)))
        motor_target = 40;
    if((servo_duty<2500) && (servo_duty>5000))
        motor_target = 40;
}

void pid_start(void)
{
    enable_irq(PIT1_IRQn);
}

void pid_stop(void)
{
    ftm_quad_clean(FTM1);
    disable_irq(PIT1_IRQn);
}



