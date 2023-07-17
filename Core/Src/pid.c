#include "pid.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

struct _pid{
    float SetSpeed;            	//目标值
    float ActualSpeed;        	//实际值
    float err;                	//当前误差
    float err_last;            	//上次误差
    float err_next;							//上上次误差
    float Kp,Ki,Kd;            	//pid参数
    float voltage;          		//输出值
    float integral;            	//积分值
    float umax;									//上限值
    float umin;									//下限值
}pid_v,pid_theta,pid_x;

struct _pid *pid;

void PID_init(void)
	{
    pid_v.SetSpeed=0.0;    	pid_v.ActualSpeed=0.0;
    pid_v.err=0.0;         	pid_v.err_last=0.0;
    pid_v.err_next=0.0;    	pid_v.voltage=0.0;
    pid_v.integral=0.0;    	pid_v.Kp=88*0.6;
    pid_v.Ki=0;          		pid_v.Kd=950*0.6;
		pid_v.umax=1000;				pid_v.umin=-1000;

    pid_x.SetSpeed=0.0;     pid_x.ActualSpeed=0.0;
    pid_x.err=0.0;          pid_x.err_last=0.0;
    pid_x.err_next=0.0;     pid_x.voltage=0.0;
    pid_x.integral=0.0;    	pid_x.Kp=0;
    pid_x.Ki=0.12;         	pid_x.Kd=0;
		pid_x.umax=1000;				pid_x.umin=-1000;

    pid_theta.SetSpeed=0.0; pid_theta.ActualSpeed=0.0;
    pid_theta.err=0.0;      pid_theta.err_last=0.0;
    pid_theta.err_next=0.0;	pid_theta.voltage=0.0;
		pid_theta.integral=0.0;	pid_theta.Kp=0;
		pid_theta.Ki=0;					pid_theta.Kd=0;
		pid_theta.umax=1000;		pid_theta.umin=-1000;
}

float PID_x_update(float set_x,float actual_x) //位移式
{
    pid = &pid_x;
    /*update set and actual*/
    pid->SetSpeed=set_x;
    pid->ActualSpeed=actual_x;

    /*calculate output speed*/
    pid->err=pid->SetSpeed-pid->ActualSpeed;
	
		//if(pid->err<15)
		pid->integral+=pid->err;
		
		pid->integral *= 0.88;

    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    pid->err_last=pid->err;

    /*check the bound*/
    if(pid->voltage>pid->umax)pid->voltage=pid->umax;
    else if(pid->voltage<pid->umin)pid->voltage=pid->umin;

    return pid->voltage;
}

float PID_speed_update(float setspeed,float actualspeed) //位移式
{
		pid = &pid_v;
    //1600 683.75

    /*update set and actual*/
    pid->SetSpeed=setspeed;
    pid->ActualSpeed=actualspeed;

    pid->err=pid->SetSpeed-pid->ActualSpeed;
    pid->integral+=pid->err;

    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    pid->err_last=pid->err;

    /*check the bound*/
    if(pid->voltage>pid->umax)pid->voltage=pid->umax;
    else if(pid->voltage<pid->umin)pid->voltage=pid->umin;
    
    return pid->voltage;
}

float PID_theta_update(float set_theta,float theta) //位置式
{
    /*calculate output Delta_theta*/
		pid_theta.SetSpeed=set_theta;
		pid_theta.ActualSpeed=theta;
	
    pid_theta.err=pid_theta.SetSpeed-pid_theta.ActualSpeed;
    pid_theta.integral+=pid_theta.err;

    //if(pid_theta.err>=18 || pid_theta.err<=-18 || pid_theta.err*pid_theta.err_last<=0)pid_theta.integral=0; //取消大角度积分

    pid_theta.voltage=pid_theta.Kp*pid_theta.err+pid_theta.Ki*pid_theta.integral+pid_theta.Kd*(pid_theta.err-pid_theta.err_last);
    pid_theta.err_last=pid_theta.err;

    return pid_theta.voltage;
}

void PID_para(int flag_pid,int flag_para,float para)
{
    /*PID parameter adjustment*/
    switch (flag_pid)
    {
    case 1: // pid_v
        pid = &pid_v;
        break;
    case 2: // pid2_v
        pid = &pid_x;
        break;
    case 3: // pid_theta
        pid = &pid_theta;
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
    }
}
