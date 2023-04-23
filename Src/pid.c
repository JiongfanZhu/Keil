#include "pid.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

struct _pid{
    float SetSpeed;            //定义设定值
    float ActualSpeed;        //定义实际值
    float err;                //定义偏差值
    float err_last;            //定义上一个偏差值
    float err_gain;
		float err_next;
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float voltage;          //定义电压值（控制执行器的变量）
    float integral;            //定义积分值
    float umax;
    float umin;
}pidv1,pidv2,pidx1,pidx2;

struct _pid *pid;

void PID_init(void){
    //printf("PID_init begin \n");
    pidv1.SetSpeed=0.0;    pidv1.ActualSpeed=0.0;
    pidv1.err=0.0;         pidv1.err_last=0.0;
    pidv1.err_next=0.0;    pidv1.voltage=0.0;
    pidv1.integral=0.0;    pidv1.Kp=2;
    pidv1.Ki=0;          pidv1.Kd=0;

    pidv2.SetSpeed=0.0;    pidv2.ActualSpeed=0.0;
    pidv2.err=0.0;         pidv2.err_last=0.0;
    pidv2.err_next=0.0;    pidv2.voltage=0.0;
    pidv2.integral=0.0;    pidv2.Kp=2;
    pidv2.Ki=0;          pidv2.Kd=0;

    pidx1.SetSpeed=0.0;     pidx1.ActualSpeed=0.0;
    pidx1.err=0.0;          pidx1.err_last=0.0;
    pidx1.err_gain=0.0;     pidx1.voltage=0.0;
    pidx1.integral=0.0;     pidx1.Kp=2;
    pidx1.Ki=0;           pidx1.Kd=0;

    pidx2.SetSpeed=0.0;     pidx2.ActualSpeed=0.0;
    pidx2.err=0.0;          pidx2.err_last=0.0;
    pidx2.err_gain=0.0;     pidx2.voltage=0.0;
    pidx2.integral=0.0;     pidx2.Kp=2;
    pidx2.Ki=0;           pidx2.Kd=0;
    //printf("PID_init end \n");
}

float PID_x_update(float set_x,float actual_x,int flag){
    /*return speed depend on x*/
    /*x in camera is decreasing, but x from wheel is increasing,how to deal with speed?*/
    /*set_x is made from picture of camera, actual_x*/
    if(flag==1) // determine wheel
    {
        pid = &pidx1;
    }
    else
    {
        pid = &pidx2;
    }
    /*update set and actual*/
    pid->SetSpeed=set_x;
    pid->ActualSpeed=actual_x;

    /*calculate output speed*/
    pid->err=pid->SetSpeed-pid->ActualSpeed;
    float incrementSpeed=pid->Kp*pid->err+pid->Ki*pid->err_gain+pid->Kd*(pid->err-pid->err_last);
    pid->err_last=pid->err;
    pid->err_gain+=pid->err;

    int speed_x = incrementSpeed; //x->speed
    
    return speed_x;
}

int PID_speed_update(float setspeed,float actualspeed,int volt,int flag)
{
    /*setspeed and actualspeed are RPM(from -360 to 360)*/
    /*pid.voltage is PWM(from -1000 to 1000)*/
    /*PWM-RPM relation may not be linear,that's why PID matters*/
    if(flag==1)
    {
        pid = &pidv1;
    }
    else
    {
        pid = &pidv2;
    }

    /*update set and actual*/
    pid->SetSpeed=setspeed;
    pid->ActualSpeed=actualspeed;

    /*calculate output voltage*/
    pid->err=pid->SetSpeed-pid->ActualSpeed;
    float incrementSpeed=pid->Kp*(pid->err-pid->err_last)+pid->Ki*pid->err+pid->Kd*(pid->err-2*pid->err_last+pid->err_next);
    pid->err_next=pid->err_last;
		pid->err_last=pid->err;
    

    volt = (int)incrementSpeed;
    
    return volt;
}

void PID_para(int flag_pid,int flag_para,float para)
{
    /*PID parameter adjustment*/
    switch (flag_pid)
    {
    case 1: // pid1_v
        pid = &pidv1;
        break;
    case 2: // pid2_v
        pid = &pidv2;
        break;
    case 3: // pid_theta
        pid = &pidx1;
        break;
    case 4: // pid_b
        pid = &pidx2;
        break;
    }
    switch (flag_para)
    {
    case 1:
        pid->Kp = para;
        break;
    case 2:
        pid->Ki = para;
        break;
    case 3:
        pid->Kd = para;
        break;
		case 4:
				pid->SetSpeed = para;
    }
}