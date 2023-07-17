/*
 * my_uart.h
 *
 *  Created on: 2023Äê6ÔÂ6ÈÕ
 *      Author: LENOVO
 */

#ifndef MY_UART_H_
#define MY_UART_H_

#include "pid.h"
#include <stdint.h>

extern uint8_t rData1[30];

float Get_Data(void);
void USART_PID_Adjust(void);

#endif /* MY_UART_H_ */
