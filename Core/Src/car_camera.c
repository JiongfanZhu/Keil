/*
 * car_carmera.c
 *
 *  Created on: 2023��7��16��
 *      Author: LENOVO
 */

#include "car_camera.h"

#define SPEED_B 10
#define SPEED 25
#define STOP 20
#define TURN_L 275
#define TURN_R 255

uint8_t status = 0; //��ǰ״̬
int stop_count = 0;
/*
message:
		0:�޺���
		1:���յ���Ŀ��Ϣ,ֱ�ӳ���
		2:Ѳ��״̬����ݮ��ֹͣ�ź� "s "
		3:���׶�ֹͣ�ź� "c "
*/

void Status_Deal(uint8_t message)
{
	static uint8_t x_step1 = 0; //�ջ�������ʶ
	static uint8_t x_step2 = 0;
	static uint8_t x_step3 = 0;
	
    switch(status)
    {
        case 0: //��ʼ״̬
			if(message==1) //��Ŀѡ�� ����Ѳ��
			{
				HAL_UART_Transmit(&huart3,(uint8_t*)"r",1,100);
				x_pid = 0;
				HAL_Delay(1000);
				wheel_enable = 1;
				setspeed_l = SPEED;
				setspeed_r = SPEED;
				HAL_Delay(500); 
				pos_pid_flag = 1;
				status = 1;
			}
			break;
    
        case 1: //Ѳ��״̬
			if(message==2) //ֹͣ��ʶ
			{
				setspeed_l = 0;
				setspeed_r = 0;
				pos_pid_flag = 0;
				wheel_enable = 0;
				HAL_Delay(1000);
				
				stop_count = STOP;
				status = 2;
				
				x_step1 = 0;
				x_step2 = 0;
				x_step3 = 0;
				
			}
			break;
					
		case 2:
			if(stop_count <=0)
			{
				stop_count = STOP;
				if(x_step1 == 0)
				{
					x_step1 = 1;
				}
				else if(x_step2 == 0)
				{
					x_step2 = 1;
				}
				else if(x_step3 == 0)
				{
					x_step3 = 1;
				}
			}
			break;
					
		case 3:
			if(message==3)
			{			
				x_step1 = 0;
				x_step2 = 0;
				x_step3 = 0;
			}
			break;
			
		case 4:
			if(Q.Q==3)
			{
			}
			break;
		case 5:
			break;
	}		
		
}