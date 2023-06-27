#include "pid.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

struct _pid{
    float SetSpeed;            //定义设定值
    float ActualSpeed;        //定义实际值
    float err;                //定义偏差值
    float err_last;            //定义上一个偏差值
    float err_next;
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float voltage;          //定义电压值（控制执行器的变量）
    float integral;            //定义积分值
    float umax;
    float umin;
}pid1_v,pid2_v,pid_theta,pid_b,pid_x1,pid_x2;

struct _pid *pid;

void PID_init(void){
    //printf("PID_init begin \n");
    pid1_v.SetSpeed=0.0;    pid1_v.ActualSpeed=0.0;
    pid1_v.err=0.0;         pid1_v.err_last=0.0;
    pid1_v.err_next=0.0;    pid1_v.voltage=0.0;
    pid1_v.integral=0.0;    pid1_v.Kp=0.5;
    pid1_v.Ki=0.3;          pid1_v.Kd=0.1;

    pid2_v.SetSpeed=0.0;    pid2_v.ActualSpeed=0.0;
    pid2_v.err=0.0;         pid2_v.err_last=0.0;
    pid2_v.err_next=0.0;    pid2_v.voltage=0.0;
    pid2_v.integral=0.0;    pid2_v.Kp=0.5;
    pid2_v.Ki=0.3;          pid2_v.Kd=0.1;

    pid_x1.SetSpeed=0.0;     pid_x1.ActualSpeed=0.0;
    pid_x1.err=0.0;          pid_x1.err_last=0.0;
    pid_x1.err_next=0.0;     pid_x1.voltage=0.0;
    pid_x1.integral=0.0;     pid_x1.Kp=0.05;
    pid_x1.Ki=0.3;           pid_x1.Kd=1;

    pid_x2.SetSpeed=0.0;     pid_x2.ActualSpeed=0.0;
    pid_x2.err=0.0;          pid_x2.err_last=0.0;
    pid_x2.err_next=0.0;     pid_x2.voltage=0.0;
    pid_x2.integral=0.0;     pid_x2.Kp=0.05;
    pid_x2.Ki=0.3;           pid_x2.Kd=1;

    pid_theta.SetSpeed=0.0; pid_theta.ActualSpeed=0.0;
    pid_theta.err=0.0;      pid_theta.err_last=0.0;
    pid_theta.voltage=0.0;  pid_theta.integral=0.0;
    pid_theta.Kp=1.15;      pid_theta.Ki=0.01;
    pid_theta.Kd=0.35;

    pid_b.SetSpeed=200.0; pid_b.ActualSpeed=0.0;
    pid_b.err=0.0;      pid_b.err_last=0.0;
    pid_b.voltage=0.0;  pid_b.integral=0.0;
    pid_b.Kp=0.20;      pid_b.Ki=0.002;
    pid_b.Kd=0.07;
    //printf("PID_init end \n");
}

float PID_x_update(float set_x,uint32_t actual_x,int flag){
    /*return speed depend on x*/
    /*x in camera is decreasing, but x from wheel is increasing,how to deal with speed?*/
    /*set_x is made from picture of camera, actual_x*/
    if(flag==1) // determine wheel
    {
        pid = &pid_x1;
    }
    else
    {
        pid = &pid_x2;
    }
    /*update set and actual*/
    pid->SetSpeed=set_x;
    pid->ActualSpeed=actual_x;

    /*calculate output speed*/
    pid->err=pid->SetSpeed-pid->ActualSpeed;
    float incrementSpeed=pid->Kp*(pid->err-pid->err_next)+pid->Ki*pid->err+
        pid->Kd*(pid->err-2*pid->err_next+pid->err_last);
    pid->err_last=pid->err_next;
    pid->err_next=pid->err;

    //int speed_x= (int)incrementSpeed; //x->speed
    
    return incrementSpeed;
}

float PID_speed_update(float setspeed,float actualspeed,float volt,int flag)
{
    /*setspeed and actualspeed are RPM(from -360 to 360)*/
    /*pid.voltage is PWM(from -1000 to 1000)*/
    /*PWM-RPM relation may not be linear,that's why PID matters*/
    if(flag==1)
    {
        pid = &pid1_v;
    }
    else
    {
        pid = &pid2_v;
    }

    /*update set and actual*/
    pid->SetSpeed=setspeed;
    pid->ActualSpeed=actualspeed;

    /*calculate output voltage*/
    pid->err=pid->SetSpeed-pid->ActualSpeed;
    float incrementSpeed=pid->Kp*(pid->err-pid->err_next)+pid->Ki*pid->err+pid->Kd*(pid->err-2*pid->err_next+pid->err_last);
    pid->err_last=pid->err_next;
    pid->err_next=pid->err;

    volt+=incrementSpeed*25/9; // 360(v)->1000(rpm)

    /*check the bound*/
    if(volt>1000)
    {
        volt=1000;
    }
    else if(volt<-1000)
    {
        volt=-1000;
    }
    
    return volt;
}

float PID_theta_update(float theta){

    /*this function return Delta_rpm*/
    /*theta from -90 to 90*/

    /*update set and actual*/
    //pid_theta.SetSpeed=setspeed;
    if(theta>-60 && theta<60)
    {
        pid_theta.ActualSpeed=theta;
    }
    else
    {
        pid_theta.ActualSpeed=-pid_theta.err;
    }

    /*calculate output Delta_theta*/
    pid_theta.err=-pid_theta.ActualSpeed;
    pid_theta.integral+=pid_theta.err;
		
		//
    if(pid_theta.err>=18 || pid_theta.err<=-18 || pid_theta.err*pid_theta.err_last<=0)pid_theta.integral=0;

    pid_theta.voltage=pid_theta.Kp*pid_theta.err+pid_theta.Ki*pid_theta.integral+pid_theta.Kd*(pid_theta.err-pid_theta.err_last);
		//((pid_theta.err-pid_theta.err_last>-20 && pid_theta.err-pid_theta.err_last<20)?1:0)
    pid_theta.err_last=pid_theta.err;

    // 90(theta)->100%, 80% theta value from -25 to 25
    // speed>0 means clockwise, speed<0 means anticlockwise
    // we need motor_L -> speed_L+Delta_speed, motor_R -> speed_R-Delta_speed
    return pid_theta.voltage;
}

float PID_b_update(float b){

    if(b>0 && b<400)
    {
        pid_b.ActualSpeed=b;
    }
    else
    {
        pid_b.ActualSpeed=pid_b.SetSpeed-pid_b.err_last;
    }
		
    /*calculate output Delta_theta*/
    pid_b.err=pid_b.SetSpeed-pid_b.ActualSpeed;

    /*do not int big err*/
    if(pid_b.err<40 && pid_b.err>-40 && pid_b.err*pid_b.err_last>0)pid_b.integral+=pid_b.err;

    pid_b.voltage=pid_b.Kp*pid_b.err+pid_b.Ki*pid_b.integral+pid_b.Kd*(pid_b.err-pid_b.err_last);
    pid_b.err_last=pid_b.err;

    pid_b.voltage = pid_b.voltage;
    /*if(pid_b.voltage>60)
    {
        pid_b.voltage=60;
    }
    else if(pid_b.voltage<-60)
    {
        pid_b.voltage=-60;
    }*/
    // speed<0 means clockwise, speed>0 means anticlockwise
    // we need motor_L -> speed_L-Delta_speed, motor_R -> speed_R+Delta_speed
    return pid_b.voltage;
}

void PID_para(int flag_pid,int flag_para,float para)
{
    /*PID parameter adjustment*/
    switch (flag_pid)
    {
    case 1: // pid1_v
        pid = &pid1_v;
        break;
    case 2: // pid2_v
        pid = &pid2_v;
        break;
    case 3: // pid_theta
        pid = &pid_theta;
        break;
    case 4: // pid_b
        pid = &pid_b;
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
