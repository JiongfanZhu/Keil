/*
 * car_carmera.c
 *
 *  Created on: 2023年7月16日
 *      Author: LENOVO
 */

#include "car_camera.h"

#define SPEED_B 10
#define SPEED 25
#define STOP 20
#define TURN_L 275
#define TURN_R 255

uint8_t status = 0; //当前状态
int stop_count = 0;
/*
message:
		0:无含义
		1:接收到题目信息,直接出发
		2:巡线状态下树莓派停止信号 "s "
		3:入库阶段停止信号 "c "
*/

void Status_Deal(uint8_t message)
{
	static uint8_t x_step1 = 0; //闭环动作标识
	static uint8_t x_step2 = 0;
	static uint8_t x_step3 = 0;
	
    switch(status)
    {
        case 0: //初始状态
			if(message==1) //题目选择 进入巡线
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
    
        case 1: //巡线状态
			if(message==2) //停止标识
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