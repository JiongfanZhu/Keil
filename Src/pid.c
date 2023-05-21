#include "pid.h"
#include "usart.h"

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
}pidv1,pidv2,pidx1,pidx2,pid_theta,pid_b;

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

    pid_theta.SetSpeed=0.0; pid_theta.ActualSpeed=0.0;
    pid_theta.err=0.0;      pid_theta.err_last=0.0;
    pid_theta.voltage=0.0;  pid_theta.integral=0.0;
    pid_theta.Kp=0.10;      pid_theta.Ki=0.00;
    pid_theta.Kd=0.00;

    pid_b.SetSpeed=200.0; pid_b.ActualSpeed=0.0;
    pid_b.err=0.0;      pid_b.err_last=0.0;
    pid_b.voltage=0.0;  pid_b.integral=0.0;
    pid_b.Kp=0.10;      pid_b.Ki=0.00;
    pid_b.Kd=0.00;
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

int PID_theta_update(float theta){

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

int PID_b_update(int b){

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
        pid = &pidv1;
        break;
    case 2: // pid2_v
        pid = &pidv2;
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

float Get_Data(void)
{
	uint8_t data_Start_Num = 0; // 记录数据位开始的地方
	uint8_t data_End_Num = 0; // 记录数据位结束的地方
	uint8_t data_Num = 0; // 记录数据位数
	uint8_t minus_Flag = 0; // 判断是不是负数
	float data_return = 0; // 解析得到的数据
	for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
	{
		if(rData1[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
		if(rData1[i] == ' ')
		{
			data_End_Num = i - 1;
			break;
		}
	}
	if(rData1[data_Start_Num] == '-') // 如果是负数
	{
		data_Start_Num += 1; // 后移一位到数据位
		minus_Flag = 1; // 负数flag
	}
		data_Num = data_End_Num - data_Start_Num + 1;
	if(data_Num == 4) // 数据共4位
	{
		data_return = (rData1[data_Start_Num]-48) +
		(rData1[data_Start_Num+2]-48)*0.1f +
		(rData1[data_Start_Num+3]-48)*0.01f;
	}
	else if(data_Num == 5) // 数据共5位
	{
		data_return = (rData1[data_Start_Num]-48) +
		(rData1[data_Start_Num+2]-48)*0.1f +
		(rData1[data_Start_Num+3]-48)*0.01f +
		(rData1[data_Start_Num+4]-48)*0.001f;
	}
//	else if(data_Num == 6) // 数据共6位
//	{
//		data_return = (rData1[data_Start_Num]-48) +
//		(rData1[data_Start_Num+1]-48)*0.1f +
//		(rData1[data_Start_Num+2]-48)*0.01f +
//		(rData1[data_Start_Num+4]-48)*0.001f +
//		(rData1[data_Start_Num+5]-48)*0.0001f;
//	}
	if(minus_Flag == 1) data_return = -data_return;
	// printf("data=%.2f\r\n",data_return);
	return data_return;
}

void USART_PID_Adjust(void)
{
	float data_Get = Get_Data(); // 存放接收到的数据
	// printf("data=%.2f\r\n",data_Get);
		
		if(rData1[0]=='P' && rData1[1]=='v' && rData1[2]=='1')      // 速度环P1
			PID_para(1,1,data_Get);
		else if(rData1[0]=='I' && rData1[1]=='v' && rData1[2]=='1') // 速度环I1
			PID_para(1,2,data_Get);
		else if(rData1[0]=='D' && rData1[1]=='v' && rData1[2]=='1') // 速度环D1
			PID_para(1,3,data_Get);
        else if(rData1[0]=='P' && rData1[1]=='v' && rData1[2]=='2') // 速度环P2
			PID_para(2,1,data_Get);
		else if(rData1[0]=='I' && rData1[1]=='v' && rData1[2]=='2') // 速度环I2
			PID_para(2,2,data_Get);
		else if(rData1[0]=='D' && rData1[1]=='v' && rData1[2]=='2') // 速度环D2
			PID_para(2,3,data_Get);
//		else if(rData1[0]=='S' && rData1[1]=='p' && rData1[2]=='e') //目标速度
//			PID_para(1,4,data_Get);
//		else if(rData1[0]=='P' && rData1[1]=='o' && rData1[2]=='s') //目标位置
////			PID_para(3,4,data_Get);
//			set_x1 = data_Get;
		
        else if(rData1[0]=='P' && rData1[1]=='t') // 角度环P
			PID_para(3,1,data_Get);
		else if(rData1[0]=='I' && rData1[1]=='t') // 角度环I
			PID_para(3,2,data_Get);
		else if(rData1[0]=='D' && rData1[1]=='t') // 角度环D
			PID_para(3,3,data_Get);
        else if(rData1[0]=='P' && rData1[1]=='b') // 截距环P
			PID_para(4,1,data_Get);
		else if(rData1[0]=='I' && rData1[1]=='b') // 截距环I
			PID_para(4,2,data_Get);
		else if(rData1[0]=='D' && rData1[1]=='b') // 截距环D
			PID_para(4,3,data_Get);
//		else if((rData1[0]=='S' && rData1[1]=='p') && rData1[2]=='e') //目标速度
//			PID_para(2,4,data_Get);
//		else if((rData1[0]=='P' && rData1[1]=='o') && rData1[2]=='s') //目标位置
////			PID_para(4,4,data_Get);
//			set_x2 = data_Get;
}