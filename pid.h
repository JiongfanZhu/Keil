#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

void PID_init(void);
float PID_x_update(float set_x,int actual_x,int flag);
float PID_speed_update(float setspeed,float actualspeed,float volt,int flag);
float PID_theta_update(float theta);
float PID_b_update(float b);
void PID_para(int flag_pid,int flag_para,float para);

#endif

