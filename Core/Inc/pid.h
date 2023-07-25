#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

void PID_init(void);
float PID_x_update(int set_x,int actual_x,uint8_t flag);
float PID_speed_update(float setspeed,float actualspeed,uint8_t flag);
float PID_theta_update(float set_theta,float theta);
float PID_pos_update(float pos);
void PID_para(int flag_pid,int flag_para,float para);

#endif

