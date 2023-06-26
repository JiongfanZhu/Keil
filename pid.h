#ifndef __PID_H__
#define __PID_H__

void PID_init(void);
int PID_x_update(int set_x,int actual_x,int flag);
int PID_speed_update(int setspeed,float actualspeed,int volt,int flag);
int PID_theta_update(float theta);
int PID_b_update(float b);
void PID_para(int flag_pid,int flag_para,float para);

#endif
