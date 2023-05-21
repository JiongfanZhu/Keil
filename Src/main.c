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
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "pid.h"
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
int cont_value1 = 0;
int cont_value2 = 0;

uint8_t rData1[MAX_REC_LENGTH] = {0};		//串口数据存储BUFF		长度2048
uint8_t rDataFlag1 = 0;							//串口接收完成标志符
uint16_t rDataCount1 = 0;						//串口长度计数
uint8_t rx_buf1[REC_LENGTH] = {0};			//串口数据接收暂存BUFF	长度1

extern osSemaphoreId UART1_flagHandle;	//操作系统定义的二值信号量

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	while(HAL_UART_Receive_IT(&huart1, rx_buf1, 1) != HAL_OK); //开启中断
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口1接收回调函数
{
	if(huart == &huart1) // user message
	{
		rData1[rDataCount1]=rx_buf1[0];
		rDataCount1++;
		if(rx_buf1[0]==0x20)// (ascii)0x14 = " ",结束标识
			{
				rDataFlag1 = 1;							//接收标识置位
				osSemaphoreRelease(UART1_flagHandle);		//释放一次二值信号量，进入串口任务
  		}
		while(HAL_UART_Receive_IT(&huart1, rx_buf1, 1) != HAL_OK); //开启中断
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/*motor pulse count*/
	/*the htim capture here refer to htim1*/
	/*positive means forward,negative means back*/
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
      /* Get the 1st Input Capture value */
        cont_value1 += HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)*2-1;
		//x_cont1 += x_pid_flag*(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)*2-1);
  }
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
      /* Get the 2nd Input Capture value */
        cont_value2 -= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)*2-1;
		//x_cont2 -= x_pid_flag*(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)*2-1);
  }
}

void Wheel(int num,int pwm)
{
	/*wheel side is forward*/
	/*pwm from -1000 to 1000(due to Counter Period)*/
	/*num=1 means wheel1,num=0 means wheel2*/
	
	int direct1 = 0; // direction 1
	int direct2 = 0; // direction 1
	
	if(pwm>1000)
	{
		pwm = 1000;
	}
	else if(pwm<-1000)
	{
		pwm = -1000;
	}
	
	if(pwm>0)
	{
		direct1 = 1; // forward
		direct2 = 1;
	}
	else if(pwm<0)
	{
		direct1 = 0; // back
		direct2 = 0;
	}
	else
	{
		direct1 = 1;
		direct2 = 0;
	}
	
	switch(num) // select wheel
	{
		case 1:
			switch(direct1)
			{
				case 1:
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000-pwm);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
					break;
				case 0:
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-pwm);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
					break;
			}
			break;
		case 2:
			switch(direct2)
			{
				case 1:
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,pwm);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
					break;
				case 0:
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1000+pwm);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
					break;
			}
			break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
