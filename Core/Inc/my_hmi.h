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
	uint8_t Q;					//问题标识
}_Q;

void HMISends(char *buf1); //发送数据
void HMISendb(uint8_t k); //连续发送三个字节数据(用于接受数据中止)
void HMISendstart(void); //串口屏接受初始化
void HMIReset(void); //问题状态复位


#endif /* MY_HMI_H_ */
