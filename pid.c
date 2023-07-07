#include "pid.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

struct _pid{
    float SetSpeed;            //?????趨?
    float ActualSpeed;        //????????
    float err;                //????????
    float err_last;            //?????????????
    float err_next;
    float Kp,Ki,Kd;            //????????????????????
    float voltage;          //?????????????????????????
    float integral;            //????????
    float umax;
    float umin;
}pid1_v,pid2_v,pid_theta,pid_b,pid_x1,pid_x2,pid_pos;

struct _pid *pid;

void PID_init(void){
    pid1_v.SetSpeed=0.0;    pid1_v.ActualSpeed=0.0;
    pid1_v.err=0.0;         pid1_v.err_last=0.0;
    pid1_v.err_next=0.0;    pid1_v.voltage=0.0;
    pid1_v.integral=0.0;    pid1_v.Kp=0.25;
    pid1_v.Ki=0.002;          pid1_v.Kd=0.004;

    pid2_v.SetSpeed=0.0;    pid2_v.ActualSpeed=0.0;
    pid2_v.err=0.0;         pid2_v.err_last=0.0;
    pid2_v.err_next=0.0;    pid2_v.voltage=0.0;
    pid2_v.integral=0.0;    pid2_v.Kp=0.25;
    pid2_v.Ki=0.002;          pid2_v.Kd=0.004;

    pid_x1.SetSpeed=0.0;     pid_x1.ActualSpeed=0.0;
    pid_x1.err=0.0;          pid_x1.err_last=0.0;
    pid_x1.err_next=0.0;     pid_x1.voltage=0.0;
    pid_x1.integral=0.0;     pid_x1.Kp=0.3;
    pid_x1.Ki=0;           pid_x1.Kd=0;

    pid_x2.SetSpeed=0.0;     pid_x2.ActualSpeed=0.0;
    pid_x2.err=0.0;          pid_x2.err_last=0.0;
    pid_x2.err_next=0.0;     pid_x2.voltage=0.0;
    pid_x2.integral=0.0;     pid_x2.Kp=0.3;
    pid_x2.Ki=0;           pid_x2.Kd=0;

    pid_theta.SetSpeed=0.0; pid_theta.ActualSpeed=0.0;
    pid_theta.err=0.0;      pid_theta.err_last=0.0;
    pid_theta.voltage=0.0;  pid_theta.integral=0.0;
    pid_theta.Kp=1.65;      pid_theta.Ki=0.003;
    pid_theta.Kd=0.04;

    pid_b.SetSpeed=135.0; pid_b.ActualSpeed=0.0;
    pid_b.err=0.0;      pid_b.err_last=0.0;
    pid_b.voltage=0.0;  pid_b.integral=0.0;
    pid_b.Kp=0.3;      pid_b.Ki=0;
    pid_b.Kd=0.015;

    pid_pos.SetSpeed=0.0; pid_pos.ActualSpeed=0.0;
    pid_pos.err=0.0;      pid_pos.err_last=0.0;
    pid_pos.voltage=0.0;  pid_pos.integral=0.0;
    pid_pos.Kp=1.7;      pid_pos.Ki=0.002;
    pid_pos.Kd=0.01;
}

float PID_x_update(float set_x,int actual_x,int flag) //位移式
{
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
    //float incrementSpeed=pid->Kp*(pid->err-pid->err_next)+pid->Ki*pid->err+
    //    pid->Kd*(pid->err-2*pid->err_next+pid->err_last);
    //pid->err_last=pid->err_next;
    //pid->err_next=pid->err;
//-0.35*(set_x>0&&flag==1||set_x<0&&flag==2)
    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    pid->err_last=pid->err;

    /*check the bound*/
    if(pid->voltage>200)
    {
        pid->voltage=200;
    }
    else if(pid->voltage<-200)
    {
        pid->voltage=-200;
    }

    
    return pid->voltage;
}

float PID_speed_update(float setspeed,float actualspeed,int flag) //位移式
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
    //pid->err=pid->SetSpeed-pid->ActualSpeed;
    //pid->voltage=pid->Kp*(pid->err-pid->err_next)+pid->Ki*pid->err+pid->Kd*(pid->err-2*pid->err_next+pid->err_last);
    //pid->err_last=pid->err_next;
    //pid->err_next=pid->err;

    //volt+=incrementSpeed*25/9; // 360(v)->1000(rpm)

    pid->err=pid->SetSpeed-pid->ActualSpeed;
    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    pid->err_last=pid->err;

    /*check the bound*/
    if(pid->voltage>800)
    {
        pid->voltage=800;
    }
    else if(pid->voltage<-800)
    {
        pid->voltage=-800;
    }
    
    return pid->voltage;
}

float PID_theta_update(float theta) //位置式
{

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

    if(pid_theta.err>=18 || pid_theta.err<=-18 || pid_theta.err*pid_theta.err_last<=0)pid_theta.integral=0; //取消大角度积分

    pid_theta.voltage=pid_theta.Kp*pid_theta.err+pid_theta.Ki*pid_theta.integral+pid_theta.Kd*(pid_theta.err-pid_theta.err_last);
    pid_theta.err_last=pid_theta.err;

    // 90(theta)->100%, 80% theta value from -25 to 25
    // speed>0 means clockwise, speed<0 means anticlockwise
    // we need motor_L -> speed_L+Delta_speed, motor_R -> speed_R-Delta_speed
    return pid_theta.voltage;
}

float PID_b_update(float b) //增量式
{

    if(b>0 && b<300)
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
    //if(pid_b.err<40 && pid_b.err>-40 && pid_b.err*pid_b.err_last>0)pid_b.integral+=pid_b.err; //小截距积分

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

float PID_pos_update(float pos) //位移式
{
    /*setspeed and actualspeed are RPM(from -360 to 360)*/
    /*pid.voltage is PWM(from -1000 to 1000)*/
    /*PWM-RPM relation may not be linear,that's why PID matters*/
    pid = &pid_pos;

    /*update set and actual*/
    pid->ActualSpeed=pos;

    /*calculate output voltage*/
    //pid->err=pid->SetSpeed-pid->ActualSpeed;
    //pid->voltage=pid->Kp*(pid->err-pid->err_next)+pid->Ki*pid->err+pid->Kd*(pid->err-2*pid->err_next+pid->err_last);
    //pid->err_last=pid->err_next;
    //pid->err_next=pid->err;

    //volt+=incrementSpeed*25/9; // 360(v)->1000(rpm)

    pid->err=pid->SetSpeed-pid->ActualSpeed;
    pid_theta.integral+=pid_theta.err;

    //if(pid->err>=20 || pid->err<=-20 || pid->err*pid->err_last<=0)pid->integral=0; //取消大积分

    pid->voltage=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
    pid->err_last=pid->err;

    /*check the bound*/
    if(pid->voltage>500)
    {
        pid->voltage=500;
    }
    else if(pid->voltage<-500)
    {
        pid->voltage=-500;
    }
    
    return pid->voltage;
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
