/*
 * hand_hand.c
 *
 *  Created on: 2023年6月6日
 *      Author: LENOVO
 */

#include "hand_hand.h"

uint8_t status_hand = 0;    //单片机当前状态
uint8_t route[10] = {0};    //小车路口决策记录(0直行,1左转,2右转)
int route_len = 0;      //小车路口经过次数
extern uint8_t route_flag;     //药品状态标志位

uint8_t recognize_flag = 0; //识别请求标识
uint8_t turn_route_flag = 0; //闭环指示(0直行,1左转,2右转,3掉头)
uint8_t x_task_flag = 0; //直线闭环开启指示
uint8_t turn_task_flag = 0; //转向闭环开启指示
uint8_t task_flag = 0; //闭环完成指示
uint8_t target_flag = 0; //目标点确定

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
    status_hand = 4;
    x_pid_flag = 0;
    route_len = 0; //清空决策数
    memset(route,0,sizeof(route)); //清空决策记录
    route_flag = 0; //无药品放置
    recognize_flag = 0; //无识别请求发出
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //熄灭
    UARTCharPutNonBlocking(UART5_BASE, 'r'); //向树莓派发送复位信息
}

void StatusDeal(uint8_t message) //message=0表示无串口信息,否则有串口信息
{
    switch(status_hand)
    {
        case 0:     //正常运行状态
            if(message == 1 && strcmp((char*)rData5,"s ") == 0) //识别到停止标识
            {
                status_hand = 1; //修改为正在停止状态1
                b_pid_flag = 0; //关闭所有pid(xpid在运行时应是关闭状态)
                theta_pid_flag = 0;
                setspeed_flag = 0;
            }
            break;
        case 1:     //正在停止状态,需要检查是否已经停下,停止不使用闭环
            if(speed1 == 0 && speed2 ==0) //已停止
            {
                //x_pid_flag = 0;
                status_hand = 2; //修改为停止状态2
                recognize_flag = 0; //识别请求清空
                if(route_flag == 1) //有药品,即送药过程
                {
                    recognize_flag = 1; //识别请求置位
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //发送识别请求
                }
            }
            break;
        case 2:     //停止状态/树莓派等待状态
            if(message == 1 && recognize_flag == 1) //有串口信息且发送了识别请求->送药过程
            {
                if(strcmp((char*)rData5,"l ")==0) //左转
                {
                    route[route_len] = 1;
                    route_len++;
                    turn_route_flag = 1;
                }
                else if(strcmp((char*)rData5,"r ")==0) //右转
                {
                    route[route_len] = 2;
                    route_len++;
                    turn_route_flag = 2;
                }
                else if(strcmp((char*)rData5,"s ")==0) //直线
                {
                    route[route_len] = 0;
                    route_len++;
                    turn_route_flag = 0;
                }
                else if(strcmp((char*)rData5,"b ")==0) //掉头,目标药房
                {
                    route_len--; //删去一个长度,当前route_len对应最后一个结点信息
                    turn_route_flag = 3;
                    LED_flag = 1; //红灯亮
                }
                status_hand = 3; //修改为闭环动作状态3
            }
            else if(route_len == -1) //经过所有记录路口并停止->到起点了
            {
                status_hand = 4; //等待
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
                            x_set1 = -ROUND_X;
                            x_set2 = ROUND_X;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                            //x_task_flag = 0; //还需要执行一段直线闭环
                        }
                        else //三段闭环完成
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
                            x_set1 = ROUND_X;
                            x_set2 = -ROUND_X;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                            //x_task_flag = 0; //还需要执行一段直线闭环
                        }
                        else //闭环完成
                        {
                            task_flag = 1;
                        }
                        break;

                    case 3: //掉头
                        if(turn_task_flag == 0 && route_flag == 0) //未开启转向闭环,且药品已卸除
                        {
                            LED_flag = 0; //熄灭
                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1 && route_flag == 0) //转向闭环完成,且药品已卸除
                        {
                            task_flag = 1; //闭环完成
                        }
                        break;
                }
                if(task_flag == 1)
                {
                    UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
                    status_hand = 0; //修改为正常运行状态
                    x_pid_flag = 0;
                    b_pid_flag = 1;
                    theta_pid_flag = 1;
                    setspeed_flag = 1;
                }
            }

        case 4:     //指令等待状态/初始状态
            if(message == 1 && strcmp((char*)rData5,"X ") == 0) //树莓派完成识别
            {
                status_hand = 0; //修改为正常运行状态
                b_pid_flag = 1;
                theta_pid_flag = 1;
                setspeed_flag = 1;
            }
            break;
    }
}



