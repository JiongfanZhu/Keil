/*
 * hand_hand.h
 *
 *  Created on: 2023��6��6��
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

extern float x_set1; //λ�Ʊջ��趨ֵ
extern float x_set2;
extern float speed1; //��ȡ��ǰʵ���ٶ�
extern float speed2;
extern uint8_t rData5[30];

void StatusReset(void); //״̬����
void StatusDeal(uint8_t message); //״̬�������������
int UART_block(uint8_t message); //˫��ͨ�ŵ����γ���




#endif /* HAND_HAND_H_ */
