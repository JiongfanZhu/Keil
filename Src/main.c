/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdlib.h"
#include "stdio.h"
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
int light_mode = 0; // 0->auto 1->man
int sign_mode = 0; // 0->auto 1->man
int light_flag = 0; //red->0 yellow->1 green->2
int sign_flag = 0; // 0->0 1->180

uint8_t rData1[40];
uint8_t rx_buf1[1];
uint8_t rDataCount1 = 0;
int rDataFlag1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	    // TIM2_CH1(pwm)
	HAL_TIM_Base_Start_IT(&htim4);                // sysclock
	while(HAL_UART_Receive_IT(&huart1, rx_buf1, 1) != HAL_OK);
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
void Light(int light_flag)
{
	switch(light_flag)
	{
		case 0: //red
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
			break;
		case 1: //yellow
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
			break;
		case 2: //green
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
			break;
	}
}

void Servo_Control(uint16_t angle)
{
	float temp;
	temp = (1.0/9.0) * angle + 5.0;
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,(uint16_t)temp);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		static int i_l = 0;
		static int i_s = 0;

    if (htim == (&htim4)) // internal clock -> 10ms
    {	
			if(light_mode == 0) // auto
			{
				if(i_l%300 == 0) // 3s
				{
					light_flag++;
					light_flag %= 3;
					Light(light_flag); // light update
				}
				i_l++;
			}
			else
			{
				i_l = 0;
			}
			
			if(sign_mode == 0) // auto
			{
				if(i_s%500 == 0) // 5s
				{
					sign_flag = 1-sign_flag;
					Servo_Control((uint16_t)(180*sign_flag)); // servo update
				}
				i_s++;
			}
			else
			{
				i_s = 0;
			}
			
			
			
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		//DataGet1(rx_buf1[0]);
	
		rData1[rDataCount1]=rx_buf1[0];
  		if(rx_buf1[0]!=0x00)
		{
    		rDataCount1++;
    		if(rx_buf1[0]==0x20)// (ascii)0x14 = " "
			{  
				//printf("%s",rData);
      			rDataFlag1 = 1;
						rDataCount1 = 0;
						
            if(light_mode == 1) // light control
            {
              if(strcmp((char*)rData1,"L0 ")==0) // red
						  {
							  //memset(s,'\0',strlen(s));
							  printf("red\n");
								light_flag = 0;
						  }
						  else if(strcmp((char*)rData1,"L1 ")==0) // yellow
						  {
							  printf("yellow\n");
								light_flag = 1;
						  }
              else if(strcmp((char*)rData1,"L2 ")==0) // green
						  {
							  printf("green\n");
								light_flag = 2;
						  }
							Light(light_flag);
            }
						else if(sign_mode == 1) // sign control
            {
              if(strcmp((char*)rData1,"S0 ")==0) // A side
						  {
							  //memset(s,'\0',strlen(s));
							  printf("Servo1\n");
								sign_flag = 0;
						  }
						  else if(strcmp((char*)rData1,"S1 ")==0) // B side
						  {
							  printf("Servo2\n");
								sign_flag = 1;
						  }
							Servo_Control((uint16_t)(180*sign_flag)); // servo update
            }
            else // change mode
            {
              if(strcmp((char*)rData1,"switchL ")==0) // change light mode
							{
								light_mode = 1-light_mode;
								printf("light mode %d\n",light_mode);
							}
							if(strcmp((char*)rData1,"switchS ")==0) // change Servo mode
							{
								sign_mode = 1-sign_mode;
								printf("sign mode %d\n",sign_mode);
							}
            }
    		}
  		}

		while(HAL_UART_Receive_IT(&huart1, rx_buf1, 1) != HAL_OK);
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
