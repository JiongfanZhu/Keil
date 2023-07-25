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
}pid_v_l,pid_v_r,pid_theta,pid_x_l,pid_x_r,pid_pos;

struct _pid *pid;

void PID_init(void)
	{
    pid_v_l.SetSpeed=0.0;    	pid_v_l.ActualSpeed=0.0;
    pid_v_l.err=0.0;         	pid_v_l.err_last=0.0;
    pid_v_l.err_next=0.0;    	pid_v_l.voltage=0.0;
    pid_v_l.integral=0.0;    	pid_v_l.Kp=21.75;
    pid_v_l.Ki=1.954;        pid_v_l.Kd=0.4284;
		pid_v_l.umax=1000;				pid_v_l.umin=-1000;
		
    pid_v_r.SetSpeed=0.0;    	pid_v_r.ActualSpeed=0.0;
    pid_v_r.err=0.0;         	pid_v_r.err_last=0.0;
    pid_v_r.err_next=0.0;    	pid_v_r.voltage=0.0;
    pid_v_r.integral=0.0;    	pid_v_r.Kp=21.75;
    pid_v_r.Ki=1.954;        pid_v_r.Kd=0.4284;
		pid_v_r.umax=1000;				pid_v_r.umin=-1000;

    pid_x_l.SetSpeed=0.0;     pid_x_l.ActualSpeed=0.0;
    pid_x_l.err=0.0;          pid_x_l.err_last=0.0;
    pid_x_l.err_next=0.0;     pid_x_l.voltage=0.0;
    pid_x_l.integral=0.0;    	pid_x_l.Kp=1.1;
    pid_x_l.Ki=0.002642;      pid_x_l.Kd=0.001;
		pid_x_l.umax=20;				pid_x_l.umin=-20;
		
    pid_x_r.SetSpeed=0.0;     pid_x_r.ActualSpeed=0.0;
    pid_x_r.err=0.0;          pid_x_r.err_last=0.0;
    pid_x_r.err_next=0.0;     pid_x_r.voltage=0.0;
    pid_x_r.integral=0.0;    	pid_x_r.Kp=1.1;
    pid_x_r.Ki=0.002642;      pid_x_r.Kd=0.001;
		pid_x_r.umax=20;				pid_x_r.umin=-20;

    pid_theta.SetSpeed=0.0; pid_theta.ActualSpeed=0.0;
    pid_theta.err=0.0;      pid_theta.err_last=0.0;
    pid_theta.err_next=0.0;	pid_theta.voltage=0.0;
		pid_theta.integral=0.0;	pid_theta.Kp=1.208;
		pid_theta.Ki=0;					pid_theta.Kd=0;
		pid_theta.umax=15;		pid_theta.umin=-15;
		
    pid_pos.SetSpeed=25.0; pid_pos.ActualSpeed=0.0;
    pid_pos.err=0.0;      pid_pos.err_last=0.0;
		pid_pos.err_next=0.0;	pid_pos.voltage=0.0;
		pid_pos.integral=0.0;	pid_pos.Kp=1.1;
		pid_pos.Ki=0;			pid_pos.Kd=0;
		pid_pos.umax=15;			pid_pos.umin=-15;
}

float PID_x_update(int set_x,int actual_x,uint8_t flag) //位移式
{
		switch(flag)
		{
			case 1:
				pid = &pid_x_l;
				break;
			default:
				pid = &pid_x_r;
				break;
		}
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

float PID_speed_update(float setspeed,float actualspeed,uint8_t flag) //位移式
{
		switch(flag)
		{
			case 1:
				pid = &pid_v_l;
				break;
			default:
				pid = &pid_v_r;
				break;
		}

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
		pid = &pid_theta;
    /*calculate output Delta_theta*/
		pid->SetSpeed=set_theta;
		pid->ActualSpeed=theta;
	
    pid->err=pid->SetSpeed-pid->ActualSpeed;
    pid->integral+=pid->err;

    //if(pid->err>=18 || pid->err<=-18 || pid->err*pid->err_last<=0)pid->integral=0; //取消大角度积分

    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    pid->err_last=pid->err;
	
    if(pid->voltage>pid->umax)pid->voltage=pid->umax;
    else if(pid->voltage<pid->umin)pid->voltage=pid->umin;

    return pid->voltage;
}

float PID_pos_update(float pos) //位移式
{
    pid = &pid_pos;
    /*update set and actual*/
    pid->ActualSpeed=pos;
	
    pid->err=pid->SetSpeed-pid->ActualSpeed;
    pid->integral+=pid->err;


    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    pid->err_last=pid->err;

    /*check the bound*/
    if(pid->voltage>pid->umax)pid->voltage=pid->umax;
    else if(pid->voltage<pid->umin)pid->voltage=pid->umin;
    
    return pid->voltage;
}

void PID_para(int flag_pid,int flag_para,float para)
{
    /*PID parameter adjustment*/
    switch (flag_pid)
    {
    case 1: // pid_v_l
        pid = &pid_v_l;
        break;
    case 2: // pid2_v
        pid = &pid_x_l;
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
