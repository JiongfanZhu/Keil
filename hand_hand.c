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
uint8_t target_flag = 0; //目标点标识

uint8_t uart_flag = 0; //双车通信标识,标记母车是否已经经过路口
uint8_t car_flag = 0; //母车转向标识(0左转,1右转)
uint8_t question_flag = 0; //题目标识
uint8_t mom_car = 0; //母车目标点记录
uint8_t mom_drug = 0; //母车卸药标识
uint8_t count_corner = 0; //执行串口等待的路口数

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
    //5:自动巡航状态
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
    question_flag = 0;
    mom_car = 0;
    mom_drug = 0;
    UARTCharPutNonBlocking(UART5_BASE, 'R'); //向树莓派发送复位信息
}

/*
树莓派发送指令集:
    "s ":小车停止;
    "l ":小车左转;
    "r ":小车右转;
    "S ":小车直行;
    "b ":小车掉头(到达药房);
    "X ":树莓派已记录目标点(X为一数字);

树莓派接受指令集:
    'd':树莓派进行识别;
    'r':树莓派进行巡线;
    'R':树莓派进行复位;

message含义:
    0:定时中断进入;
    1:树莓派串口信息进入;
    2:母车目标信息(题目选择);
    3:母车出现转向;
    4:母车卸药完成;

PS:树莓派向小车发送信号后,总是回到指令等待状态
   但初始化时树莓派的状态还需要商榷(?)
*/

int UART_block(uint8_t message) //查看双车通信信息
{
    switch (message)
    {
        case 0: //定时中断
            break;
        case 1: //树莓派信息
            break;
        case 2: //母车病房信息
            mom_car = rData1[0] - '0';
            break;
        case 3: //母车转向信息

            break;
        case 4: //母车卸药完成
            mom_drug = 1;
            break;
    }
}

void StatusDeal(uint8_t message) //message=0表示无串口信息,否则有串口信息
{
    if(UART_block(message) == 0);

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
                counter_corner++;
                status_hand = 2; //修改为停止状态2
                recognize_flag = 0; //识别请求复位
                if(route_flag == 1) //有药品,即送药过程
                {
                    recognize_flag = 1; //识别请求发送标识置位
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
                    LED_flag = 1; //红灯亮
                }
                status_hand = 3; //修改为闭环动作状态3
            }
            else if(route_len == -1) //经过所有记录路口并停止->到起点了
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
                    /*开启需要的pid*/
                    x_pid_flag = 0;
                    b_pid_flag = 1;
                    theta_pid_flag = 1;
                    setspeed_flag = 1;
                }
            }

        case 4:     //指令等待状态/初始状态
            if(message == 1 && rData5[0]>='1' && rData5[0]<='8' && rData5[1] == ' ') //树莓派完成识别
            {
                target_flag = rData5[0] - '0'; // 已知目标病房,下一步等待药品装载
                if(mom_car == target_flag) // 同一目标病房
                {
                    question_flag = 1;
                }
                else
                {
                    question_flag = 2;
                }
                counter_corner = 0;
            }

            /*药品完成装载且树莓派已完成识别或母车完成卸药*/
            if((route_flag == 1 && question_flag == 1)||(mom_drug == 1 && question_flag = 2))
            {
                status_hand = 0; //修改为正常运行状态
                /*开启巡线对应pid*/
                b_pid_flag = 1;
                theta_pid_flag = 1;
                setspeed_flag = 1;
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
            }
            break;
    }
}



