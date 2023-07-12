#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

void PID_init(void);
float PID_x_update(float set_x,float actual_x);
float PID_speed_update(float setspeed,float actualspeed);
float PID_theta_update(float set_theta,float theta);
void PID_para(int flag_pid,int flag_para,float para);

#endif

