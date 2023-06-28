/*
 * hand_hand.h
 *
 *  Created on: 2023年6月6日
 *      Author: LENOVO
 */

#ifndef HAND_HAND_H_
#define HAND_HAND_H_
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "pid.h"

extern int x_pid_flag;
extern int b_pid_flag;
extern int theta_pid_flag;
extern int setspeed_flag;
extern uint8_t LED_flag;

extern float x_set1; //位移闭环设定值
extern float x_set2;
extern float speed1; //读取当前实际速度
extern float speed2;
extern uint8_t rData5[30];

void StatusReset(void); //状态重置
void StatusDeal(uint8_t message); //状态更改与任务分配
int UART_block(uint8_t message); //双车通信的屏蔽程序




#endif /* HAND_HAND_H_ */
