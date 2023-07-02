#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

void PID_init(void);
float PID_x_update(float set_x,int actual_x,int flag);
float PID_speed_update(float setspeed,float actualspeed,int flag);
float PID_theta_update(float theta);
float PID_b_update(float b);
void PID_para(int flag_pid,int flag_para,float para);
void PID_reset(void);

extern int x_last_flag;
extern float x_set1;
extern float x_set2;

extern int x_pid_flag;
extern int b_pid_flag;
extern int theta_pid_flag;
extern int setspeed_flag;

extern void Wheel_set(float pwm,int num);

#endif

