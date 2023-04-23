/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "pid.h"
#include "stdio.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_buf1[1];
uint8_t rData1[40];
uint8_t rDataCount1 = 0;
int rDataFlag1 = 0;

uint8_t rx_buf3[1];
uint8_t rData3[40];
uint8_t rDataCount3 = 0;
int rDataFlag3 = 0;

float set_x1 = 310;
float set_x2 = 243;

float x1 = 0;
float x2 = 0;
float last_x1 = 0;
float last_x2 = 0;

int pid_flag = 0;
int data_flag = 0;

char * endptr;

const float POINT[9][2]= // 9个目标点作为常量使用
{
{310,243},
{214,140},
{407,148.5},
{204,336.5},
{404,342},
{0,0},
{0,0},
{0,0},
{0,0},
};

int destination_flag = 0; //目标位置，0对应点1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void USART_PID_Adjust(void);
float Get_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fgetc(FILE *f) {      
	uint8_t ch = 0;
	HAL_UART_Receive(&huart1,&ch,1,0xffff);
	return ch;
}

int fputc(int ch, FILE *f) {      
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xffff);
	return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	void Servo_pwm(int num,int pwm);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); // 舵机两路pwm输出
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);                // sysclock
	PID_init();
	
	while(HAL_UART_Receive_IT(&huart1, rx_buf1, 1) != HAL_OK); // user bluetooth
	while(HAL_UART_Receive_IT(&huart3, rx_buf3, 1) != HAL_OK); // berry pie
	
	//_HAL_TIM_SET_COMPARE 占空比控制
	//这里使用的装载值为60000(pwm周期为20ms)，0~90度对应0.5ms~1.5ms，即装载值对应1500~4500
	//_HAL_TIM_SET_AUTORELOAD 周期控制
	//Servo_pwm(1,stable_pwm1);
	//Servo_pwm(2,stable_pwm2);
	HAL_Delay(2000);
//	set_x1 = POINT[0][0];
//	set_x2 = POINT[0][1];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		for(int i=0;i<500;i++)
//		{
//			Servo_pwm(1,3000+i);
//			Servo_pwm(2,3000+i);
//			HAL_Delay(10);
//		}
//		for(int i=0;i<1000;i++)
//		{
//			Servo_pwm(1,3500-i);
//			Servo_pwm(2,3500-i);
//			HAL_Delay(10);
//		}
//		for(int i=0;i<500;i++)
//		{
//			Servo_pwm(1,2500+i);
//			Servo_pwm(2,2500+i);
//			HAL_Delay(10);
//		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Servo_pwm(int num,int pwm) // pwm scale:1500~4500
{
	uint8_t temp = 0x00000000U;
	int stable_pwm = 0;
	switch(num)
	{
		case 1:
			temp = TIM_CHANNEL_1;
			stable_pwm = 3200;
			break;
		case 2:
			temp = TIM_CHANNEL_2;
			stable_pwm = 3100;
			break;
	}
	if(pwm<stable_pwm-300)
	{
		pwm = stable_pwm-300;
	}
	else if(pwm>stable_pwm+300)
	{
		pwm = stable_pwm+300;
	}
	__HAL_TIM_SET_COMPARE(&htim3,temp,pwm);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		static int i = 0;
		static float v1 = 0;
		static float v2 = 0;
		static float v_x1 = 0;
		static float v_x2 = 0;
		static int pwm1 = 3200;
		static int pwm2 = 3100;
    if (htim == (&htim4)) // internal clock -> 10ms
    {	
			/*setspeed PID processing*/
			if(i%10==0 && pid_flag) // 0.1s x pid (Using "%" matters!)
			{
				v_x1 = PID_x_update(set_x1,x1,1); //speed determined by x
				v_x2 = PID_x_update(set_x2,x2,2);
			}

			if(i%5==0 && pid_flag && data_flag) // v pid avaliable(T=0.03s)
			{
				
				v1 = (x1 - last_x1);
				v2 = (x2 - last_x2);
				data_flag = 0;
//				last_x1 = x1;
//				last_x2 = x2;
				pwm1 = 3200+PID_speed_update(v_x1,v1,pwm1,1);
				pwm2 = 3100+PID_speed_update(v_x2,v2,pwm2,2);
				printf("%f,%f,%d,%d,%f,%f\n",v_x1,v_x2,pwm1,pwm2,v1,v2);
				
				Servo_pwm(1,pwm1);
				Servo_pwm(2,pwm2);
			}

			if(i>100) // 1s reset i
			{
				i=0;
			}
			i++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	float x1_buff = 0;
	float x2_buff = 0;
	if(huart == &huart3) // berry pie message
	{
		//DataGet1(rx_buf1[0]);
		//树莓派数据格式："x1_rate x2_rate "，最后有空格
		rData3[rDataCount3]=rx_buf3[0];
  		if(rx_buf3[0]!=0x00)
		{
    		rDataCount3++;
    		if(rx_buf3[0]==0x20)// (ascii)0x14 = " "
			{  
				//printf("%s",rData);
				rDataFlag3 = 1;
				rDataCount3 = 0;
				
				x1_buff = strtof(rData3,&endptr);
				endptr++;
				x2_buff = strtod(endptr,NULL);
				endptr = rData3;
				if(x1_buff>0 && x2_buff>0)
				{
					//位置信息更新
					last_x1 = x1;
					last_x2 = x2;
					x1 = 2*x1_buff;
					x2 = 2*x2_buff;
					data_flag = 1;
				}
				//printf("%f %f\r\n",x1,x2);
				
				for(int i=0;i<40;i++)rData3[i]='\0'; // clear buffer
    		}
  		}
		while(HAL_UART_Receive_IT(&huart3, rx_buf3, 1) != HAL_OK);
	}
	
	if(huart == &huart1) // user message
	{
		//DataGet1(rx_buf1[0]);
		rDataCount1++;
		rData1[rDataCount1-1]=rx_buf1[0];  		
    		if(rx_buf1[0]==0x20)// (ascii)0x14 = " "
			{  
				//printf("%s",rData);
				printf("RXLen=%d\r\n",rDataCount1);
				for(int i=0;i<rDataCount1;i++) printf("UART rData1[%d] =%c\r\n",i,rData1[i]);			
//						if(strcmp((char*)rData1,"2task ")==0) //point 1
//						{
//							set_x1 = POINT[0][0];
//							set_x2 = POINT[0][1];
//						}
//						else if(strcmp((char*)rData1,"3task ")==0) //point 2
//						{
//							set_x1 = POINT[1][0];
//							set_x2 = POINT[1][1];
//						}
//						else if(strcmp((char*)rData1,"4task ")==0) //point 3
//						{
//							set_x1 = POINT[2][0];
//							set_x2 = POINT[2][1];
//						}
						if(strcmp((char*)rData1,"stop ")==0) //point 3
						{
							pid_flag = 0;
							Servo_pwm(1,3150);
							Servo_pwm(2,3100);
						}
						else if(strcmp((char*)rData1,"start ")==0) //point 3
						{
							pid_flag = 1;
						}
						else // para set
						{
							USART_PID_Adjust();//数据解析和参数赋值函数
						}
					memset(rData1,0,sizeof(rData1)); //清空缓存数组
					rDataCount1=0; //清空接收长度
  		}
			rx_buf1[0]=0;
		while(HAL_UART_Receive_IT(&huart1, rx_buf1, 1) != HAL_OK);
	}
}

/*
* 解析出rData1中的数据
* 返回解析得到的数据
*/
float Get_Data(void)
{
	uint8_t data_Start_Num = 0; // 记录数据位开始的地方
	uint8_t data_End_Num = 0; // 记录数据位结束的地方
	uint8_t data_Num = 0; // 记录数据位数
	uint8_t minus_Flag = 0; // 判断是不是负数
	float data_return = 0; // 解析得到的数据
	for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
	{
		if(rData1[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
		if(rData1[i] == ' ')
		{
			data_End_Num = i - 1;
			break;
		}
	}
	if(rData1[data_Start_Num] == '-') // 如果是负数
	{
		data_Start_Num += 1; // 后移一位到数据位
		minus_Flag = 1; // 负数flag
	}
		data_Num = data_End_Num - data_Start_Num + 1;
	if(data_Num == 4) // 数据共4位
	{
		data_return = (rData1[data_Start_Num]-48) +
		(rData1[data_Start_Num+2]-48)*0.1f +
		(rData1[data_Start_Num+3]-48)*0.01f;
	}
	else if(data_Num == 5) // 数据共5位
	{
		data_return = (rData1[data_Start_Num]-48) +
		(rData1[data_Start_Num+2]-48)*0.1f +
		(rData1[data_Start_Num+3]-48)*0.01f +
		(rData1[data_Start_Num+4]-48)*0.001f;
	}
//	else if(data_Num == 6) // 数据共6位
//	{
//		data_return = (rData1[data_Start_Num]-48) +
//		(rData1[data_Start_Num+1]-48)*0.1f +
//		(rData1[data_Start_Num+2]-48)*0.01f +
//		(rData1[data_Start_Num+4]-48)*0.001f +
//		(rData1[data_Start_Num+5]-48)*0.0001f;
//	}
	if(minus_Flag == 1) data_return = -data_return;
	// printf("data=%.2f\r\n",data_return);
	return data_return;
}

void USART_PID_Adjust(void)
{
	float data_Get = Get_Data(); // 存放接收到的数据
	// printf("data=%.2f\r\n",data_Get);
		if(rData1[0]=='P' && rData1[1]=='x' && rData1[2]=='1') // 位置环P
			PID_para(3,1,data_Get);
		else if(rData1[0]=='I' && rData1[1]=='x' && rData1[2]=='1') // 位置环I
			PID_para(3,2,data_Get);
		else if(rData1[0]=='D' && rData1[1]=='x' && rData1[2]=='1') // 位置环D
			PID_para(3,3,data_Get);
		else if(rData1[0]=='P' && rData1[1]=='v' && rData1[2]=='1') // 速度环P
			PID_para(1,1,data_Get);
		else if(rData1[0]=='I' && rData1[1]=='v' && rData1[2]=='1') // 速度环I
			PID_para(1,2,data_Get);
		else if(rData1[0]=='D' && rData1[1]=='v' && rData1[2]=='1') // 速度环D
			PID_para(1,3,data_Get);
//		else if(rData1[0]=='S' && rData1[1]=='p' && rData1[2]=='e') //目标速度
//			PID_para(1,4,data_Get);
//		else if(rData1[0]=='P' && rData1[1]=='o' && rData1[2]=='s') //目标位置
////			PID_para(3,4,data_Get);
//			set_x1 = data_Get;
		
		
		else if(rData1[0]=='P' && rData1[1]=='x' && rData1[2]=='2') // 位置环P
			PID_para(4,1,data_Get);
		else if(rData1[0]=='I' && rData1[1]=='x' && rData1[2]=='2') // 位置环I
			PID_para(4,2,data_Get);
		else if(rData1[0]=='D' && rData1[1]=='x' && rData1[2]=='2') // 位置环D
			PID_para(4,3,data_Get);
		else if(rData1[0]=='P' && rData1[1]=='v' && rData1[2]=='2') // 速度环P
			PID_para(2,1,data_Get);
		else if(rData1[0]=='I' && rData1[1]=='v' && rData1[2]=='2') // 速度环I
			PID_para(2,2,data_Get);
		else if(rData1[0]=='D' && rData1[1]=='v' && rData1[2]=='2') // 速度环D
			PID_para(2,3,data_Get);
//		else if((rData1[0]=='S' && rData1[1]=='p') && rData1[2]=='e') //目标速度
//			PID_para(2,4,data_Get);
//		else if((rData1[0]=='P' && rData1[1]=='o') && rData1[2]=='s') //目标位置
////			PID_para(4,4,data_Get);
//			set_x2 = data_Get;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
