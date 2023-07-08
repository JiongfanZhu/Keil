/*
 * hand_hand.c
 *
 *  Created on: 2023年6月6日
 *      Author: LENOVO
 */

#include "hand_hand.h"

#define TURN_X 1650
#define L_ROUND_X 950
#define R_ROUND_X 950
#define STRAIGHT_X1 1100
#define STRAIGHT_X2 1100
#define STOP 5
#define SPEED_B 70
#define UART_CNT 10
#define TIME 50
#define TIME_STOP 50

uint8_t status_hand = 4;    //单片机当前状态
uint8_t route[10] = {0};    //小车路口决策记录(0直行,1左转,2右转)
int route_len = 0;      //小车路口经过次数
extern uint8_t route_flag;     //药品状态标志位
extern int uart_flag;
extern uint8_t question;
extern uint8_t conflict_flag;

uint8_t turn_route_flag = 0; //闭环指示(0直行,1左转,2右转,3掉头)
uint8_t x_task_flag = 0; //直线闭环开启指示
uint8_t turn_task_flag = 0; //转向闭环开启指示
uint8_t task_flag = 0; //闭环完成指示
uint8_t target = 0; //目标点标识
uint8_t next_q = 0; //被子车信息阻塞的下一状态
int stop_count = 1; //距离被子车阻塞的路口数
uint8_t turn_flag = 0; //是否完成第一次转向
uint8_t cross = 0; //忽略路口数
uint8_t cross_flag = 0; //是否经过所有路口
int stop_judge = STOP; //停止判定次数
uint8_t yellow_light = 0; //标识是否到达过自选点
uint8_t back_flag = 0; //第一问是否已经转向过
uint8_t block_flag = 0; //阻塞信号标识
uint8_t back_count = 0;

void StatusReset(void)
{
    //status_hand的几种状态
    //0:正常运行状态/指令屏蔽状态
    //1:正在停止状态
    //2:停止状态/树莓派等待状态
    //3:闭环动作状态
    //4:用户指令等待状态/初始状态
    //5:子车阻塞状态
    //6:停止开环状态
    //7:循迹屏蔽状态

    status_hand = 4;
    route_len = 0; //清空决策数
    memset(route,0,sizeof(route)); //清空决策记录
    route_flag = 0; //无药品放置
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //熄灭
    target = 0;
    next_q = 0;
    stop_count = 1;
    turn_flag = 0;
    cross = 0;
    uart_flag = UART_CNT;
    stop_judge = STOP;
    pid_flag = 0;
    keep = 0;
    pid_reset_flag = 0;
    back_flag = 0;
    block_flag = 0;
    question = 0;
    back_count = 0;
    conflict_flag = 0;
    //UARTCharPutNonBlocking(UART5_BASE, 'R'); //向树莓派发送复位信息
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
    
    4:X;
    5:ok;


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
        //UARTprintf("case 0\r\n");
            if(keep > 0) //屏蔽挂起过程中但被触发(被定时中断触发)
            {
                if(message == 1)keep = 5;
                //UARTprintf("keep=%d\r\n",keep);
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //保持巡线
            }
            else if((message==1 && time_count==0 && question==2)||(message==1 && time_count==0 && question==1 && strcmp((char*)rData5,"s ")==0)) //识别到停止标识,树莓派发送停止指令后自动待机
            {

                if(cross>0) //经过路口
                {
                    //UARTprintf("cross\r\n");
                    cross--;
                    keep = 5; //屏蔽
                    UARTCharPutNonBlocking(UART5_BASE, 'r');
                }
                else
                {
                    //UARTprintf("crossover\r\n");
                    stop_judge = STOP;
                    time_count = TIME_STOP;
                    pos_pid_flag = 0;
                    setspeed_flag = 0;
                    status_hand = 1;
                }
            }

            else if(question == 1 && message == 1 && time_count==0 && strcmp((char*)rData5,"b ") == 0)
            {
                //UARTprintf("back\r\n");
                stop_judge = STOP;
                time_count = TIME_STOP;
                
                pos_pid_flag = 0;
                setspeed_flag = 0;
                status_hand = 3;
                turn_route_flag = 3;

                x_task_flag = 0; //重置闭环有关标识
                turn_task_flag = 0;
                task_flag = 0;
            }
            break;
        case 1:     //正在停止状态,需要检查是否已经停下,停止不使用闭环
            //if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B && time_count == -1) //已停止
            if(time_count == 0) //已停止
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
                //UARTprintf("already stop\r\n");
                //status_hand = 2; //修改为停止状态2
                if(question==1)
                {
                    if(route_flag != 0 && yellow_light == 0) //母车有转向信息,且自选点未完成
                    {
                        turn_route_flag = 3 - route_flag; //子车反向转
                        if(route_flag == 0)turn_route_flag = 1; //无消息默认左转
                    }
                    else if(yellow_light == 1) //自选点完成,且母车已释放阻塞信息
                    {   
                        turn_route_flag = 0;
                    }
                    status_hand = 3;
                    time_count = TIME;
                    stop_judge = STOP;
                    x_task_flag = 0; //重置闭环有关标识
                    turn_task_flag = 0;
                    task_flag = 0;
                }
                else if(question == 2)
                {
                    switch(route_flag)
                    {
                        case 1:
                            route_len = 0;
                            route[0] = 1;
                            route[1] = 1;
                            route_flag = 0; //重置
                            break;
                        case 2:
                            route_len = 0;
                            route[0] = 1;
                            route[1] = 2;
                            route_flag = 0; //重置
                            break;
                        case 3:
                            route_len = 0;
                            route[0] = 2;
                            route[1] = 1;
                            route_flag = 0; //重置
                            break;
                        case 4:
                            route_len = 0;
                            route[0] = 2;
                            route[1] = 2;
                            route_flag = 0; //重置
                            break;
                        default:
                            break;
                    }

                    if(route_flag == 0)
                    {
                        if(back_flag == 0) //不冲突
                        {
                            turn_route_flag = route[route_len];
                        }
                        else //冲突
                        {
                            turn_route_flag = 3 - route[route_len]; //走相反方向
                        }
                        route_len++;
                        status_hand = 3;
                        time_count = TIME;
                        stop_judge = STOP;
                        x_task_flag = 0; //重置闭环有关标识
                        turn_task_flag = 0;
                        task_flag = 0;
                    }
                }
            }
            break;
        /*case 2:     //停止状态/树莓派等待状态
            //UARTprintf("case 2\r\n");
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
                UARTprintf("decision:%d\r\n",turn_route_flag);
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
            break;*/
        case 3:     //闭环动作状态
        //UARTprintf("case 3\r\n");
            if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B && time_count==0) //停止说明已完成上一闭环任务或从状态2转换至3
            {
                stop_judge--;
            }
            else
            {
                stop_judge = STOP;
            }
            if(stop_judge<=0 && time_count==0)
            {
                //stop_judge = STOP;
                //UARTprintf("mission start\r\n");
                switch(turn_route_flag) //提取闭环指示
                {
                    case 0: //直线
                        if(x_task_flag == 0) //未开启直线闭环
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
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
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //未开启转向闭环
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

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
                            time_count = TIME;
                            stop_judge = STOP;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //未开启转向闭环
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

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
                        if(x_task_flag == 0) //未开启直线闭环
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;
                            if(question == 2)
                            {
                                x_set1 = 4000;
                                x_set2 = 3900;
                            }
                            else
                            {
                                x_set1 = STRAIGHT_X1;
                                x_set2 = STRAIGHT_X2;
                            }
                            x_task_flag = 1;

                            if(question == 1 && yellow_light == 1) //第一问完成自选点
                            {
                                turn_task_flag = 1; //跳过转向
                                task_flag = 1;
                            }
                        }
                        else if(turn_task_flag == 0) //未开启转向闭环,且药品已卸除
                        {
                            route_flag = 0;
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            turn_task_flag = 1;

                        }
                        else //转向闭环完成,且药品已卸除
                        {
                            if(question == 1) //第一问未完成自选点
                            {
                                if(yellow_light == 0)
                                {
                                    LED_flag = 2;
                                    //UARTprintf("light on\r\n");
                                }

                                if(message == 4) //母车完成卸药
                                {
                                    //UARTprintf("light off\r\n");
                                    LED_flag = 0; //灭灯
                                    yellow_light = 1; //自选点任务完成
                                }
                                else if(message == 5) //母车释放阻塞
                                {
                                    //UARTprintf("car release\r\n");

                                    cross = 1;
                                    task_flag = 1;
                                }
                            }
                            else if(question == 2)
                            {
                                task_flag = 1; //闭环完成
                            }
                        }
                        break;
                }
                if(task_flag == 1) //完成闭环任务
                {
                    //UARTprintf("task done\r\n");
                    /*开启需要的pid*/
                    //x_pid_flag = 0;
                    uart_flag = UART_CNT;
                    x_task_flag = 0; //重置闭环有关标识
                    turn_task_flag = 0;
                    task_flag = 0;
                    pos = 0;
                    pos_speed = 0;
                    if(question == 1)
                    {
                        if(back_flag == 0 && yellow_light == 1) //刚完成自选点
                        {
                            //UARTprintf("back set\r\n");
                            status_hand = 7; //修改为正常运行状态
                            back_flag = 1;
                            UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
                        }
                        else if(back_flag == 1) //完成自选点
                        {
                            status_hand = 8;
                        }
                        else
                        {
                            status_hand = 7; //修改为正常运行状态
                            UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求                       
                        }
                    }
                    else if(question == 2)
                    {
                        if(block_flag == 1) //阻塞
                        {
                            //UARTprintf("block\r\n");
                            block_flag = 0;
                            back_flag = 0;
                            status_hand = 5;
                            next_q = 7;
                            cross = 1; //阻塞后需要经过一个路口
                            keep = 0;
                        }
                        else if(back_flag == 0) //不冲突或冲突结束 
                        {
                            //UARTprintf("no conflict\r\n");
                            if(conflict_flag == 0 && question==2)
                            {
                                status_hand = 7;
                                UARTprintf("ok ");
                                conflict_flag = 1;
                            }
                            else
                            {
                                if(turn_route_flag == 0) //没有有效的转向信息了,到终点
                                {
                                    status_hand = 8;
                                }
                                else
                                {
                                    status_hand = 7;
                                }
                            }

                        }
                        else
                        {
                            block_flag = 1; //完成下一动作后进入阻塞状态
                            UARTprintf("ok "); //向母车释放阻塞信号
                            /*立即转向*/
                            x_task_flag = 0;
                            turn_task_flag = 0;
                            task_flag = 0;

                            turn_route_flag = 3;
                            time_count = TIME;
                            stop_judge = STOP;
                        }
                    }
                }
            }
            break;

        case 4:     //指令等待状态/初始状态
            if((question == 1 && Drug_Read() == 1)||(question == 2 && message == 4)) //第一问 或 子车启动信号
            {
                //UARTprintf("go\r\n");
                cross = question; //经过路口数
                keep = 0;
                status_hand = 7;
                block_flag = 0;
                if(question == 1)
                {
                    yellow_light = 0;
                    back_flag = 0;
                }
                /*开启巡线对应pid*/
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
                uart_flag = UART_CNT;
            }
            else //母车有传输转弯数据
            {
                //UARTprintf("wait!\r\n");
            }
            break;
        case 5:     //母车阻塞状态
            if(message == 5) //阻塞信号被释放
            {
                status_hand = next_q;
                next_q = 0;
                if(status_hand == 7)UARTCharPutNonBlocking(UART5_BASE, 'r'); //发送巡线请求
            }
            break;
        case 7:     //循迹屏蔽状态
            //UARTprintf("case 7\r\n");
            if(message == 1 && (strcmp((char*)rData5,"s ") == 0 || strcmp((char*)rData5,"b ") == 0)) //残留帧造成停止
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
                //UARTprintf("error stop\r\n");
                uart_flag = UART_CNT;
            }
            else if(uart_flag == 0) //当前循迹帧稳定
            {
                //UARTprintf("goto case 0\r\n");
                uart_flag = -1;
                status_hand = 0;
                x_pid_flag = 0;
                data_flag = 0;
                pid_reset_flag = 1;
                pid_flag = 1;
                pos = 0;
                pos_speed = 0;

                setspeed_flag = 1;
                pos_pid_flag = 1;
            }
            else
            {
                //UARTprintf("r printf\r\n");
                UARTCharPutNonBlocking(UART5_BASE, 'r');
            }
            break;
        case 8: //啥都不干
            if(question == 2)LED_flag = 1;
            //UARTprintf("do nothing\r\n");
            break;
    }
}
