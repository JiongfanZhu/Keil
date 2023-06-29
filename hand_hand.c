/*
 * hand_hand.c
 *
 *  Created on: 2023年6月6日
 *      Author: LENOVO
 */

#include "hand_hand.h"

uint8_t status_hand = 0;    //单片机当前状态
extern uint8_t route_flag;     //药品状态标志位

uint8_t turn_route_flag = 0; //闭环指示(0直行,1左转,2右转,3掉头)
uint8_t x_task_flag = 0; //直线闭环开启指示
uint8_t turn_task_flag = 0; //转向闭环开启指示
uint8_t task_flag = 0; //闭环完成指示
uint8_t target_flag = 0; //目标点确定标识

uint8_t question = 0; //题目选择
uint8_t mom_car = 0; //母车病房信息
uint8_t mom_drug = 0; //母车卸药标识
uint8_t mom_decision = 0; //母车第一次转向信息(0左转,1右转)
uint8_t decision_flag = 0; //是否服从母车转向
uint8_t back_flag = 0; //绕路标识
uint8_t next_q = 0; //被阻塞的下一状态
uint8_t auto_count = 0; //自动巡航路口数

#define TURN_X 400
#define ROUND_X 100
#define STRAIGHT_X 100

void StatusReset(void)
{
    //status_hand的几种状态
    //0:正常运行状态/指令屏蔽状态
    //1:正在停止状态
    //2:停止状态/树莓派等待状态
    //3:闭环动作状态
    //4:用户指令等待状态/初始状态
    //5:子车阻塞状态
    status_hand = 5;
    x_pid_flag = 0;
    route_flag = 0; //无药品放置
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //熄灭
    target_flag = 0;
    question = 0;
    mom_car = 0;
    mom_drug = 0;
    back_flag = 0;
    next_q = 0;
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
            if(message == 1 && strcmp((char*)rData5,"s ") == 0) //识别到停止标识,树莓派发送停止指令后自动待机
            {
                status_hand = 1; //切换为正在停止状态
                /*开启需要的pid*/
                b_pid_flag = 0;
                theta_pid_flag = 0;
                setspeed_flag = 0;
            }
            break;
        case 1:     //正在停止状态
            if(speed1 == 0 && speed2 ==0) //已经停止
            {
                status_hand = 2; //切换为停止状态2
                if(back_flag == 1) //绕路标识置位
                {
                    next_q = 3;
                    status_hand = 5;
                    turn_route_flag = 3; //被阻塞后首先掉头
                    back_flag = 0;
                }
                else //一般性循迹
                {
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //发送识别请求
                }
            }
            break;
        case 2:     //停止状态
            if(message == 1) //有串口信息
            {
                status_hand = 3; //闭环状态3
                if(decision_flag == 1 && question == 1) //母车转向信息有效(第一问)
                {
                    turn_route_flag = 2 - mom_decision; //与母车转向相反
                    decision_flag = 0;
                }
                else if(strcmp((char*)rData5,"l ")==0) //左转
                {
                    if(question == 2) //第二问
                    {
                        if(decision_flag == 1) //母车转向标识是否有效
                        {
                            if(mom_decision == 0) //母车也左转,冲突
                            {
                                turn_route_flag = 2; //子车先右转
                                back_flag = 1; //绕路标识置位
                            }
                            else //母车右转,不冲突
                            {
                                turn_route_flag = 1;
                                back_flag = 0;
                            }
                            decision_flag = 0; //母车转向生效一次后无效
                        }
                        else //正常转向
                        {
                            turn_route_flag = 1;
                        }  
                    }
                    else if(auto_count > 0) //自动巡航过程
                    {
                        auto_count--; //巡航路口-1
                        turn_route_flag = 0; //保持直行
                    }
                    else if(auto_count == 0) //巡航结束
                    {
                        auto_count--; //巡航路口-1
                        turn_route_flag = 3; //掉头
                    }
                }
                else if(strcmp((char*)rData5,"r ")==0) //右转
                {
                    if(question == 2)
                    {
                        if(decision_flag == 1)
                        {
                            if(mom_decision == 0)
                            {
                                turn_route_flag = 2;
                                back_flag = 0;
                            }
                            else
                            {
                                turn_route_flag = 1;
                                back_flag = 1;
                            }
                            decision_flag = 0;
                        }
                        else
                        {
                            turn_route_flag = 2;
                        }
                    }
                    else if(auto_count > 0)
                    {
                        auto_count--;
                        turn_route_flag = 0;
                    }
                    else if(auto_count == 0)
                    {
                        turn_route_flag = 3;
                    }
                }
                else if(strcmp((char*)rData5,"S ")==0) //直行
                {
                    if(question == 2)
                    {
                        turn_route_flag = 0;
                    }
                    else if(auto_count > 0)
                    {
                        auto_count--;
                        turn_route_flag = 0;
                    }
                    else if(auto_count == 0)
                    {
                        turn_route_flag = 3;
                    }
                }
                else if(strcmp((char*)rData5,"b ")==0) //掉头,到药房
                {
                    if(question == 2)LED_flag = 1; //点亮红灯
                    status_hand = 5; //阻塞进程
                }
            }
            x_task_flag = 0; //相关闭环标识重置
            turn_task_flag = 0;
            task_flag = 0;
            break;
        case 3:     //闭环状态
            if(speed1 == 0 && speed2 == 0) //已完成上一闭环动作
            {
                switch(turn_route_flag) //闭环标识
                {
                    case 0: //直行
                        if(x_task_flag == 0) //未完成直行动作
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else
                        {
                            task_flag = 1;
                        }
                        break;

                    case 1: //左转
                        if(x_task_flag == 0) //未完成直行动作
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //未完成转向动作
                        {
                            x_set1 += -ROUND_X;
                            x_set2 += ROUND_X;
                            turn_task_flag = 1;
                        }
                        else
                        {
                            task_flag = 1;
                        }
                        break;

                    case 2: //右转
                        if(x_task_flag == 0)
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0)
                        {
                            x_set1 += ROUND_X;
                            x_set2 += -ROUND_X;
                            turn_task_flag = 1;
                        }
                        else
                        {
                            task_flag = 1;
                        }
                        break;

                    case 3: //掉头
                        if(turn_task_flag == 0)
                        {
                            LED_flag = 0; //熄灭
                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1)
                        {
                            task_flag = 1;
                        }
                        break;
                }
                if(task_flag == 1)
                {
                    if(auto_count == 0 && question == 1) //自动巡航结束(第一问),阻塞并点灯
                    {
                        next_q = 0;
                        status_hand = 5;
                        LED_flag = 2; //亮黄灯
                        auto_count--; //巡航路口-1
                    }
                    else
                    {
                        UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送循迹请求
                        status_hand = 0; //切换为状态0
                        /*开启必要的pid*/
                        x_pid_flag = 0;
                        b_pid_flag = 1;
                        theta_pid_flag = 1;
                        setspeed_flag = 1;
                    }
                }
            }

        case 4:     //指令等待模式
            if(message == 1 && rData5[0]>='1' && rData5[0]<='8' && rData5[1] == ' ') //子车病房信息
            {
                target_flag = rData5[0] - '0'; //提取信息
                if(mom_car == target_flag) //同一病房
                {
                    question = 1;
                    UARTprintf("Q1 ");
                }
                else
                {
                    question = 2;
                    UARTprintf("Q2 ");
                }
            }
            else if(message == 2)
            {
                mom_car = rData1[0]; //母车信息
            }

            /*满足提高部分的发车条件*/
            if((route_flag == 1 && question == 1)||(mom_drug == 1 && question == 2))
            {
                status_hand = 0; //切换为状态0
                if(question == 1)auto_count = 2; //第一问启用自动巡航
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线指令
                /*开启必要的pid*/
                b_pid_flag = 1;
                theta_pid_flag = 1;
                setspeed_flag = 1;
            }
            break;
        case 5: //阻塞
            if(message == 4) // 母车释放阻塞信号
            {
                status_hand = next_q;
                next_q = 0;
            }
            break;
    }
}



