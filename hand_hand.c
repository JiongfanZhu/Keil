/*
 * hand_hand.c
 *
 *  Created on: 2023年6月6日
 *      Author: LENOVO
 */

#include "hand_hand.h"

uint8_t status_hand = 0;    //单片机当前状态
extern uint8_t route_flag;     //药品状态标志位

uint8_t recognize_flag = 0; //识别请求标识
uint8_t turn_route_flag = 0; //闭环指示(0直行,1左转,2右转,3掉头)
uint8_t x_task_flag = 0; //直线闭环开启指示
uint8_t turn_task_flag = 0; //转向闭环开启指示
uint8_t task_flag = 0; //闭环完成指示
uint8_t target_flag = 0; //目标点标识

uint8_t uart_flag = 0; //双车通信标识,标记母车是否已经经过路口
uint8_t car_flag = 0; //母车转向标识(0左转,1右转)
uint8_t question = 0; //题目标识
uint8_t mom_car = 0; //母车目标点记录
uint8_t mom_drug = 0; //母车卸药标识
uint8_t mom_decision = 0; //母车第一次转向决策(0左转,1右转)
uint8_t decision_flag = 0; //决定当前转向决策是否有效
uint8_t back_flag = 0; //子车是否需要返回绕路标识
uint8_t next_q = 0; //被阻塞的次态
uint8_t auto_count = 0; //自动巡航计数

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
    //5:阻塞状态
    status_hand = 5;
    x_pid_flag = 0;
    route_len = 0; //清空决策数
    memset(route,0,sizeof(route)); //清空决策记录
    route_flag = 0; //无药品放置
    recognize_flag = 0; //无识别请求发出
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //熄灭
    target_flag = 0;
    uart_flag = 0;
    car_flag = 0;
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
    "X ":树莓派已记录目标病房(X为一数字);

树莓派接受指令集:
    'd':树莓派进行识别;
    'r':树莓派进行巡线;
    'R':树莓派进行复位;

message含义:
    0:定时中断进入;
    1:树莓派串口信息进入;
    2:母车病房信息(题目选择);
    3:母车卸药完成;
    4:母车阻塞信号释放;

双车通信指令集:
    "X ":母车完成卸药,子车取药指令;
    "1 ":向子车发送母车病房信息;
    "Q1 "/"Q2 ":向母车发送对应题目信息;
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
                status_hand = 1; //修改为正在停止状态1
                /*关闭巡线相关pid*/
                b_pid_flag = 0;
                theta_pid_flag = 0;
                setspeed_flag = 0;
            }
            break;
        case 1:     //正在停止状态,需要检查是否已经停下,停止不使用闭环
            if(speed1 == 0 && speed2 ==0) //已停止
            {
                status_hand = 2; //修改为停止状态2
                recognize_flag = 1; //识别请求发送标识置位
                if(back_flag == 1) //绕路置位,需要阻塞,等待母车释放
                {
                    next_q = 3;
                    status_hand = 5;
                    turn_route_flag = 3; //下次释放阻塞后直接掉头
                }
                else //一般性的循迹
                {
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //发送识别请求
                }
            }
            break;
        case 2:     //停止状态/树莓派等待状态
            if(message == 1 && recognize_flag == 1) //有串口信息且发送了识别请求->送药过程
            {
                if(strcmp((char*)rData5,"l ")==0) //左转
                {
                    if(question == 2) //第二问
                    {
                        if(decision_flag == 1) //母车转向标识有效
                        {
                            if(mom_decision == 0) //母车也左转,路线冲突
                            {
                                turn_route_flag = 2; //子车先右转
                                back_flag = 1; //绕路标识置位 
                            }
                            else //母车右转,路线不冲突
                            {
                                turn_route_flag = 1;
                                back_flag = 0;
                            }
                            decision_flag = 0; //执行一次后无效母车的转向限制
                        }
                        else
                        {
                            turn_route_flag = 1;
                        }  
                    }
                    else if(auto_count > 0) //第一问自动巡航
                    {
                        auto_count--; //巡航路口-1
                        turn_route_flag = 0; //屏蔽转向,直行
                    }
                    else if(auto_count == 0) //到达自选点
                    {
                        turn_route_flag = 3; //掉头
                    }
                }
                else if(strcmp((char*)rData5,"r ")==0) //右转
                {
                    if(question == 2) //第二问
                    {
                        if(decision_flag == 1) //母车转向标识有效
                        {
                            if(mom_decision == 0) //母车左转,路线不冲突
                            {
                                turn_route_flag = 2;
                                back_flag = 0;
                            }
                            else //母车也右转,路线冲突
                            {
                                turn_route_flag = 1; //子车先左转
                                back_flag = 1; //绕路标识置位 
                            }
                            decision_flag = 0; //执行一次后无效母车的转向限制
                        }
                        else
                        {
                            turn_route_flag = 2;
                        }
                    }
                    else if(auto_count > 0) //第一问自动巡航
                    {
                        auto_count--; //巡航路口-1
                        turn_route_flag = 0; //屏蔽转向,直行
                    }
                    else if(auto_count == 0) //到达自选点
                    {
                        turn_route_flag = 3; //掉头
                    }
                }
                else if(strcmp((char*)rData5,"S ")==0) //直行
                {
                    if(question == 2)
                    {
                        turn_route_flag = 0;
                    }
                    else if(auto_count > 0) //第一问自动巡航
                    {
                        auto_count--; //巡航路口-1
                        turn_route_flag = 0; //屏蔽转向,直行
                    }
                    else if(auto_count == 0) //到达自选点
                    {
                        turn_route_flag = 3; //掉头
                    }
                }
                status_hand = 3; //修改为闭环动作状态3
            }
            x_task_flag = 0; //重置闭环有关标识
            turn_task_flag = 0;
            task_flag = 0;
            break;
        case 3:     //闭环动作状态
            if(speed1 == 0 && speed2 == 0) //停止说明已完成上一闭环任务或从状态2转换至3
            {
                switch(turn_route_flag) //提取闭环指示
                {
                    case 0: //直线
                        if(x_task_flag == 0) //未开启直线闭环
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
                        if(x_task_flag == 0) //未开启直线闭环
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //未开启转向闭环
                        {
                            x_set1 += -ROUND_X;
                            x_set2 += ROUND_X;
                            turn_task_flag = 1;
                            //x_task_flag = 0; //还需要执行一段直线闭环
                        }
                        else //闭环完成
                        {
                            task_flag = 1;
                        }
                        break;

                    case 2: //右转
                        if(x_task_flag == 0) //未开启直线闭环
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //未开启转向闭环
                        {
                            x_set1 += ROUND_X;
                            x_set2 += -ROUND_X;
                            turn_task_flag = 1;
                            //x_task_flag = 0; //还需要执行一段直线闭环
                        }
                        else //闭环完成
                        {
                            task_flag = 1;
                        }
                        break;

                    case 3: //掉头
                        if(turn_task_flag == 0) //未开启转向闭环,且药品已卸除
                        {
                            LED_flag = 0; //熄灭
                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1) //转向闭环完成,且药品已卸除
                        {
                            task_flag = 1; //闭环完成
                        }
                        break;
                }
                if(task_flag == 1)
                {
                    UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
                    status_hand = 0; //修改为正常运行状态
                    if(auto_count == 0 && question == 1) // 自选点掉头完成(第一问)
                    {
                        next_q = 0;
                        status_hand = 5;
                        LED_flag = 2; //点亮黄灯
                    }
                    else
                    {
                        /*开启需要的pid*/
                        x_pid_flag = 0;
                        b_pid_flag = 1;
                        theta_pid_flag = 1;
                        setspeed_flag = 1;
                    }
                }
            }

        case 4:     //指令等待状态/初始状态
            if(message == 1 && rData5[0]>='1' && rData5[0]<='8' && rData5[1] == ' ') //树莓派完成识别
            {
                target_flag = rData5[0] - '0'; // 已知目标病房,下一步等待药品装载
                if(mom_car == target_flag) // 同一目标病房
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
                mom_car = rData1[0];
            }

            /*药品完成装载且已知目标病房(Q1)或母车完成卸药(Q2)*/
            if((route_flag == 1 && question == 1)||(mom_drug == 1 && question == 2))
            {
                status_hand = 0; //修改为正常运行状态
                if(question == 1)auto_count = 2; //两个停止点后阻塞
                /*开启巡线对应pid*/
                b_pid_flag = 1;
                theta_pid_flag = 1;
                setspeed_flag = 1;
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
            }
            break;
        case 5: //阻塞状态
            if(message == 4) // 阻塞释放
            {
                status_hand = next_q;
                next_q = -1;
            }
            break;
    }
}



