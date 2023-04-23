#ifndef __PID_H__
#define __PID_H__

void PID_init(void);
float PID_x_update(float set_x,float actual_x,int flag);
int PID_speed_update(float setspeed,float actualspeed,int volt,int flag);
void PID_para(int flag_pid,int flag_para,float para);

#endif