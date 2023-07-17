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
#include "stdio.h"

#include "my_uart.h"
#include "pid.h"
#include "my_hmi.h"
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

/*串口数据处理变量区*/
uint8_t rx_buf1;
uint8_t rData1[30];
uint8_t rDataCount1 = 0;
int rDataFlag1 = 0;

uint8_t rx_buf2;
uint8_t rData2[30];
uint8_t rDataCount2 = 0;
int rDataFlag2 = 0;

uint8_t rx_buf3;
uint8_t rData3[30];
uint8_t rDataCount3 = 0;
int rDataFlag3 = 0;

/*串口屏用缓冲区*/
unsigned char buf[64];

/*问题与相关标识位结构体*/
extern _Q Q;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Wheel(int pwm,uint8_t flag); //电机驱动函数
int GetTimEnCoder(uint8_t flag); //电机编码器读取函数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*串口重定向*/
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	/*开启定时器2,定时器3编码器*/
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	/*开启定时器4中断*/
	HAL_TIM_Base_Start_IT(&htim4);
	/*开启定时器4所有PWM*/
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	/*开启定时器1所有PWM*/
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	/*关闭定时器1所有输出*/
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	
	/*两电机输出为0*/
	Wheel(0,1);
	Wheel(0,2);
	
	/*开启所有串口中断*/
	while(HAL_UART_Receive_IT(&huart1, &rx_buf1, 1) != HAL_OK)printf("usart1 ok\r\n");
	while(HAL_UART_Receive_IT(&huart2, &rx_buf2, 1) != HAL_OK)printf("usart2 ok\r\n");
	while(HAL_UART_Receive_IT(&huart3, &rx_buf3, 1) != HAL_OK)printf("usart3 ok\r\n");
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
void Wheel(int pwm,uint8_t flag) //直流电机驱动
{
	uint32_t Wheel_pwm_1 = 0;
	uint32_t Wheel_pwm_2 = 0;
	
	switch(flag)
	{
		case 1: //左轮
			Wheel_pwm_1 = TIM_CHANNEL_1;
			Wheel_pwm_2 = TIM_CHANNEL_2;
			break;
		default: //右轮
			Wheel_pwm_1 = TIM_CHANNEL_3;
			Wheel_pwm_2 = TIM_CHANNEL_4;
			break;
	}
	/*pwm限幅*/
	if(pwm>1000)
	{
		pwm = 1000;
	}
	else if(pwm<-1000)
	{
		pwm = -1000;
	}

	/*高电平停止*/
	if(pwm==0)
	{
		__HAL_TIM_SET_COMPARE(&htim4,Wheel_pwm_1,1000);
		__HAL_TIM_SET_COMPARE(&htim4,Wheel_pwm_2,1000);
	}
	else
	{
		if(pwm>0)
		{
			__HAL_TIM_SET_COMPARE(&htim2,Wheel_pwm_1,1000-pwm);
			__HAL_TIM_SET_COMPARE(&htim3,Wheel_pwm_2,1000);
		}
		else if(pwm<0)
		{
			__HAL_TIM_SET_COMPARE(&htim2,Wheel_pwm_1,1000);
			__HAL_TIM_SET_COMPARE(&htim3,Wheel_pwm_2,1000+pwm);
		}	
	}
}

int GetTimEnCoder(uint8_t flag) //电机编码器读取
{
	TIM_HandleTypeDef *TIM_p;
	
	switch(flag)
	{
		case 1: //左轮编码器
			TIM_p = &htim2;
			break;
		default:
			TIM_p = &htim3;
			break;
	}
	
	int iTimerEncoder = (short)(__HAL_TIM_GET_COUNTER(TIM_p));
	__HAL_TIM_SET_COUNTER(TIM_p,0); //编码器复位
	return iTimerEncoder;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //定时中断回调函数
{
	static int i = 0;

	if (htim == (&htim4)) // internal clock -> 1ms
	{
		if(i%5 == 0)
		{

		}
		if(i>=100)i=0;
		i++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //串口回调函数
{
	if(huart == &huart1)
	{
		rData1[rDataCount1]=rx_buf1;
  	if(rx_buf1!=0x00)
		{
    	rDataCount1++;
    	if(rx_buf1==0x20)// (ascii)0x20 = " "
			{  
      	rDataFlag1 = 1;
				rDataCount1 = 0;
				/*if(strcmp((char*)rData1,"0 ")==0)
				{
					
				}
				else */
				if(rData1[0]==0xEE) //复位标识
				{
					HMIReset();
				}
				else if(rData1[0]>=0x01 && rData1[0]<=0x04 && Q.order_rec==1)
				{
					sprintf((char *)buf,"t0.txt=\"receive %d\"",rData1[0]);
					HMISends((char *)buf);
					HMISendb(0xff);
				}
				memset(rData1,0,sizeof(rData1)); //清空缓存数组
    	}
  	}
		while(HAL_UART_Receive_IT(&huart1, &rx_buf1, 1) != HAL_OK);
	}
	else if(huart == &huart2)
	{
		rData2[rDataCount2]=rx_buf2;
  	if(rx_buf2!=0x00)
		{
    	rDataCount2++;
    	if(rx_buf2==0x20)// (ascii)0x20 = " "
			{  
      	rDataFlag2 = 1;
				rDataCount2 = 0;
				if(strcmp((char*)rData2,"0 ")==0)
				{
					
				}
				memset(rData2,0,sizeof(rData2)); //清空缓存数组
    	}
  	}
		while(HAL_UART_Receive_IT(&huart2, &rx_buf2, 1) != HAL_OK);
	}
	else if(huart == &huart3)
	{
		rData3[rDataCount3]=rx_buf3;
  	if(rx_buf3!=0x00)
		{
    	rDataCount3++;
    	if(rx_buf3==0x20)// (ascii)0x20 = " "
			{  
      	rDataFlag3 = 1;
				rDataCount3 = 0;
				if(strcmp((char*)rData3,"0 ")==0)
				{
					
				}
				memset(rData3,0,sizeof(rData3)); //清空缓存数组
    	}
  	}
		while(HAL_UART_Receive_IT(&huart3, &rx_buf3, 1) != HAL_OK);
	}
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
