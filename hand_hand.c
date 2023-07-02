/*
 * hand_hand.c
 *
 *  Created on: 2023年6月6日
 *      Author: LENOVO
 */

#include "hand_hand.h"

#define TURN_X 2000
#define L_ROUND_X 1250
#define R_ROUND_X 1250
#define STRAIGHT_X1 1250
#define STRAIGHT_X2 1300
#define STOP 15
#define SPEED_B 50
#define UART_CNT 10

uint8_t status_hand = 4;    //单片机当前状态
uint8_t route[10] = {0};    //小车路口决策记录(0直行,1左转,2右转)
int route_len = 0;      //小车路口经过次数
extern uint8_t route_flag;     //药品状态标志位
extern int uart_flag;

uint8_t recognize_flag = 0; //识别请求标识
uint8_t turn_route_flag = 0; //闭环指示(0直行,1左转,2右转,3掉头)
uint8_t x_task_flag = 0; //直线闭环开启指示
uint8_t turn_task_flag = 0; //转向闭环开启指示
uint8_t task_flag = 0; //闭环完成指示
uint8_t target = 0; //目标点标识
uint8_t next_q = 0; //被子车信息阻塞的下一状态
int stop_count = 1; //距离被子车阻塞的路口数
uint8_t turn_flag = 0; //是否完成第一次转向
uint8_t cross_flag = 0; //是否为第一个路口
int stop_judge = STOP; //停止判定次数

void StatusReset(void)
{
    //status_hand的几种状态
    //0:正常运行状态/指令屏蔽状态
    //1:正在停止状态
    //2:停止状态/树莓派等待状态
    //3:闭环动作状态
    //4:用户指令等待状态/初始状态
    //5:子车阻塞状态
    status_hand = 4;
    //PID_reset();
    route_len = 0; //清空决策数
    memset(route,0,sizeof(route)); //清空决策记录
    route_flag = 0; //无药品放置
    recognize_flag = 0; //无识别请求发出
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //熄灭
    target = 0;
    next_q = 0;
    stop_count = 1;
    turn_flag = 0;
    cross_flag = 0;
    uart_flag = UART_CNT;
    stop_judge = STOP;
    UARTCharPutNonBlocking(UART5_BASE, 'R'); //向树莓派发送复位信息
}

/*
树莓派发送指令集:
    "s ":小车停止;
    "l ":小车左转;
    "r ":小车右转;
    "S ":小车直行;
    "b ":小车掉头(到达药房);
    "X ":树莓派已记录目标点(X是1~8的数字);

树莓派接受指令集:
    'd':树莓派进行识别;
    'r':树莓派进行巡线;
    'R':树莓派复位;

message含义:
    0:定时中断进入;
    1:树莓派串口信息;
    2:目标病房信息;
    3:子车释放阻塞信号;
    
    4:巡线测试;


双车通信指令集:
    "X ":母车完成卸药,子车取药指令;
    "1 ":向子车发送母车病房信息;
    "Q1 ":向母车发送对应题目信息;
    "ok ":对母车/子车的阻塞释放信号;

PS:树莓派向小车发送信号后,总是回到指令等待状态
   但初始化时树莓派的状态还需要商榷(?)
*/

void StatusDeal(uint8_t message) //message=0表示无串口信息,否则有串口信息
{
    switch(status_hand)
    {
        case 0:     //正常运行状态
        UARTprintf("case 0\r\n");
            if(message == 1 && strcmp((char*)rData5,"s ") == 0) //识别到停止标识,树莓派发送停止指令后自动待机
            {
                //UARTprintf("stop\r\n");
                status_hand = 1; //修改为正在停止状态1
                //status_hand = 6;
                /*关闭巡线相关pid*/
                //PID_reset();
                b_pid_flag = 0;
                setspeed_flag = 0;
                theta_pid_flag = 0;
                Wheel_set(0,1);
                Wheel_set(0,2);
            }
            break;
        case 1:     //正在停止状态,需要检查是否已经停下,停止不使用闭环
            UARTprintf("case 1\r\n");
            if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B) //已停止
            {
                stop_judge--;
            }
            else
            {
                stop_judge = STOP;
            }
            if(stop_judge <=0)
            {
                stop_judge = STOP;
                //PID_reset();
                status_hand = 2; //修改为停止状态2
                recognize_flag = 0; //识别请求复位
                if(route_flag == 1 && cross_flag == 1) //有药品,即送药过程,且不为第一次停止
                {
                    recognize_flag = 1; //识别请求发送标识置位
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //发送识别请求
                    UARTprintf("d send\r\n");
                }
                else if(cross_flag == 0) //还没有停止过
                {
                    //UARTprintf("first turn\r\n");
                    switch(target)
                    {
                        case 1:
                            turn_route_flag = 1; //目标病房为1,应在左侧
                            break;
                        case 2:
                            turn_route_flag = 2; //目标病房为2,应在右侧
                            break;
                        default:
                            turn_route_flag = 0; //不是近端药房,直行
                    }
                    status_hand = 3; //直接进入闭环
                    cross_flag = 1; //标识位置位
                }
                else if(question==2 || question==1) //仅在提高进行阻塞
                {
                    stop_count--; //同时也用于对子车释放阻塞
                    if(stop_count==0 && question==2) //仅在第二问对母车阻塞
                    {
                        next_q = status_hand;
                        status_hand = 5; //阻塞
                    }
                }
            }
            else
            {
                //UARTprintf("not stop yet\r\n");
            }
            break;
        case 2:     //停止状态/树莓派等待状态
            UARTprintf("case 2\r\n");
            if(message != 1 && recognize_flag == 1)
            {
                UARTCharPutNonBlocking(UART5_BASE, 'd'); //发送识别请求
                UARTprintf("d send again\r\n");
            }
            if(message == 1 && recognize_flag == 1) //有串口信息且发送了识别请求->送药过程
            {
                if(strcmp((char*)rData5,"l ")==0) //左转
                {
                    if(turn_flag == 0) //第一次转向
                    {
                        UARTprintf("l ");
                        turn_flag = 1;
                    }
                    route[route_len] = 1;
                    route_len++;
                    turn_route_flag = 1;
                }
                else if(strcmp((char*)rData5,"r ")==0) //右转
                {
                    if(turn_flag == 0) //第一次转向
                    {
                        UARTprintf("r ");
                        turn_flag = 1;
                    }
                    route[route_len] = 2;
                    route_len++;
                    turn_route_flag = 2;
                }
                else if(strcmp((char*)rData5,"S ")==0) //直行
                {
                    route[route_len] = 0;
                    route_len++;
                    turn_route_flag = 0;
                }
                else if(strcmp((char*)rData5,"b ")==0) //掉头,到达目标药房
                {
                    route_len--; //删去一个长度,当前route_len对应最后一个结点信息,准备出栈
                    turn_route_flag = 3;
                    LED_flag = 1; //红灯亮(基础部分)
                }
                status_hand = 3; //修改为闭环动作状态3
            }
            else if(route_len == -2) //经过所有记录路口并停止->到起点了
            {
                status_hand = 4; //等待,要求小车面向药房
                LED_flag = 2; //绿灯亮
            }
            else if(message == 0 && recognize_flag == 0) //无串口信息且未发送识别请求->回家过程中
            {
                switch(route[route_len]) //提取该路口转向信息
                {
                    case 0: //直线
                        turn_route_flag = 0;
                        break;
                    case 1: //左转,返回时右转
                        turn_route_flag = 2;
                        break;
                    case 2: //右转,返回时左转
                        turn_route_flag = 1;
                        break;
                }
                route_len--; //删去一个路口
                status_hand = 3; //修改为闭环动作状态3
            }
            x_task_flag = 0; //重置闭环有关标识
            turn_task_flag = 0;
            task_flag = 0;
            break;
        case 3:     //闭环动作状态
        UARTprintf("case 3\r\n");
            if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B) //停止说明已完成上一闭环任务或从状态2转换至3
            {
                stop_judge--;
            }
            else
            {
                stop_judge = STOP;
            }
            if(stop_judge <= 0)
            {
                stop_judge = STOP;
                //PID_reset();
                //UARTprintf("case 3\r\n");
                switch(turn_route_flag) //提取闭环指示
                {
                    case 0: //直线
                        if(x_task_flag == 0) //未开启直线闭环
                        {
                            //UARTprintf("step1\r\n");

                            //PID_reset();
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else
                        {
                            //PID_reset();
                            //UARTprintf("step2\r\n");
                            task_flag = 1;
                        }
                        break;
                    case 1: //左转
                        if(x_task_flag == 0) //未开启直线闭环
                        {
                            //PID_reset();
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //未开启转向闭环
                        {
                            //PID_reset();
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = L_ROUND_X;
                            x_set2 = -L_ROUND_X;
                            turn_task_flag = 1;
                        }
                        else //闭环完成
                        {
                            task_flag = 1;
                        }
                        break;
                    case 2: //右转
                        if(x_task_flag == 0) //未开启直线闭环
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //未开启转向闭环
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = -R_ROUND_X;
                            x_set2 = R_ROUND_X;
                            turn_task_flag = 1;
                        }
                        else //闭环完成
                        {
                            task_flag = 1;
                        }
                        break;
                    case 3: //掉头,仅在到达药房时执行这一分支
                        if(turn_task_flag == 0 && route_flag == 0) //未开启转向闭环,且药品已卸除
                        {
                            if(question != 0)
                            {
                                UARTprintf("X "); //子车取药指令
                            }
                            LED_flag = 0; //熄灭
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1 && route_flag == 0) //转向闭环完成,且药品已卸除
                        {
                            task_flag = 1; //闭环完成
                        }
                        break;
                }
                if(task_flag == 1) //完成闭环任务
                {
                    //UARTprintf("task done\r\n");
                    UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
                    if((stop_count==-1 && question==2)||(stop_count==0 && question==1))UARTprintf("ok "); //在被阻塞后的第一个路口完成动作后,释放子车阻塞信号
                    status_hand = 7; //修改为正常运行状态
                    /*开启需要的pid*/
                    //PID_reset();
                    x_pid_flag = 0;
                    uart_flag = UART_CNT;

                    //b_pid_flag = 1;
                    //theta_pid_flag = 1;
                    //setspeed_flag = 1;
                }
            }
            break;

        case 4:     //指令等待状态/初始状态
        UARTprintf("case 4\r\n");
            if(message == 2) //树莓派完成识别
            {
                target = rData5[0]-'0';
                UARTprintf("%d ",target); //发送母车病房信息
            }
            if(target != 0 && route_flag == 1) //药品完成装载且树莓派已完成识别
            {
                status_hand = 7; //修改为正常运行状态
                /*开启巡线对应pid*/
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
                UARTprintf("start\r\n");
                uart_flag = UART_CNT;
            }
            break;
        case 5:     //子车阻塞状态
            if(message == 3) //阻塞信号被释放
            {
                status_hand = next_q;
                next_q = 0;
            }
            break;
        case 6:     //测试状态
            //PID_reset();
            if(message == 4)
            {
                status_hand = 7;
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
                uart_flag = UART_CNT;
            }
            break;
        case 7:     //循迹屏蔽状态
            UARTprintf("case 7\r\n");
            if(message == 1)
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
                uart_flag = UART_CNT;
            }
            if(uart_flag <= 0)
            {
                status_hand = 0;
                x_pid_flag = 0;
                //PID_reset();
                setspeed_flag = 1;
                b_pid_flag = 1;
                theta_pid_flag = 1;
            }
            else
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
            }
            break;
    }
}
