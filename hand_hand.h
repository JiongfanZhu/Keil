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
extern int pos_pid_flag;
extern int setspeed_flag;
extern uint8_t LED_flag;
extern uint8_t question;
extern uint32_t x_last1;
extern uint32_t x_last2;
extern uint8_t pid_flag;
extern uint8_t data_flag;
extern uint8_t pid_reset_flag;
extern uint8_t time_count;
extern float setspeed;

extern float x_set1; //位移闭环设定值
extern float x_set2;
extern float speed1; //读取当前实际速度
extern float speed2;
extern float pos;
extern float pos_speed;
extern uint8_t rData5[30];
extern uint8_t keep;

void StatusReset(void); //状态重置
void StatusDeal(uint8_t message); //状态更改与任务分配
uint8_t Drug_Read(void);

#endif /* HAND_HAND_H_ */
