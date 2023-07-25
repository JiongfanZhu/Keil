/*
 * my_hmi.h
 *
 *  Created on: 2023��7��16��
 *      Author: LENOVO
 */

#ifndef MY_HMI_H_
#define MY_HMI_H_

#include <stdint.h>
#include "usart.h"

typedef struct		//���ڶ�����������ر�ʶλ
{
	uint8_t Q;					//�����ʶ
}_Q;

void HMISends(char *buf1); //��������
void HMISendb(uint8_t k); //�������������ֽ�����(���ڽ���������ֹ)
void HMISendstart(void); //���������ܳ�ʼ��
void HMIReset(void); //����״̬��λ


#endif /* MY_HMI_H_ */
