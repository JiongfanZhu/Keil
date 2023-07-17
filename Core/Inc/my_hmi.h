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
	uint8_t reset; 			//ģʽ��λ��ʶ
	uint8_t Q1;					//״̬1��ʶ
	uint8_t Q2;					//״̬2��ʶ
	uint8_t Q3;					//״̬3��ʶ
	uint8_t Q4;					//״̬3��ʶ
	uint8_t data_send; 	//���ݷ��ͱ�ʶ
	uint8_t order_rec; 	//������ܱ�ʶ
}_Q;

void HMISends(char *buf1); //��������
void HMISendb(uint8_t k); //�������������ֽ�����(���ڽ���������ֹ)
void HMISendstart(void); //���������ܳ�ʼ��
void HMIReset(void); //����״̬��λ


#endif /* MY_HMI_H_ */
