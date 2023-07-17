/*
 * my_hmi.h
 *
 *  Created on: 2023年7月16日
 *      Author: LENOVO
 */

#ifndef MY_HMI_H_
#define MY_HMI_H_

#include <stdint.h>
#include "usart.h"

typedef struct		//用于定义问题与相关标识位
{
	uint8_t reset; 			//模式复位标识
	uint8_t Q1;					//状态1标识
	uint8_t Q2;					//状态2标识
	uint8_t Q3;					//状态3标识
	uint8_t Q4;					//状态3标识
	uint8_t data_send; 	//数据发送标识
	uint8_t order_rec; 	//命令接受标识
}_Q;

void HMISends(char *buf1); //发送数据
void HMISendb(uint8_t k); //连续发送三个字节数据(用于接受数据中止)
void HMISendstart(void); //串口屏接受初始化
void HMIReset(void); //问题状态复位


#endif /* MY_HMI_H_ */
