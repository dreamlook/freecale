#ifndef _MOTOR_PID_H_
#define _MOTOR_PID_H_


void PID_init(void);
void speed_pid(float set_speed,float back_speed);
void angle_speed(uint16 servo_duty);
void pid_start(void);
void pid_stop(void);

#endif