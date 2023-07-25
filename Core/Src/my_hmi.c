/*
 * my_hmi.c
 *
 *  Created on: 2023年7月16日
 *      Author: LENOVO
 */

#include "my_hmi.h"
_Q Q;

void HMISends(char *buf1)		  //字符串发送函数
{
	uint8_t i=0;
	while(1)
	{
		if(buf1[i]!=0)
		{
			HAL_UART_Transmit(&huart1,&(buf1[i]),1,0xffff);
			i++;
		}
		else return;
	}
}


void HMISendb(uint8_t k)		         //字节发送函数
{		 
	uint8_t i;
	for(i=0;i<3;i++)
	{
		if(k!=0)
		{
			HAL_UART_Transmit(&huart1,&(k),1,0xffff);
		}
		else return;
	} 
}

void HMISendstart(void)
{
	HMIReset();
	HAL_Delay(200);
	HMISendb(0xff);
	HAL_Delay(200);
}

void HMIReset(void)
{
	Q.Q = 0;
}
