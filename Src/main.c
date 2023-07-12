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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

#include "my_uart.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_INDEX_MAX 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_buf3;
uint8_t rData3[30];
uint8_t rDataCount3 = 0;
int rDataFlag3 = 0;

int iTimerEncoder;

float ADC_Value = 0;
float last_ADC_Value = 0; //上一次角度
float ADC_buff[ADC_INDEX_MAX] = {0};
uint8_t ADC_index = 0;

int setspeed = 0;

uint8_t test_flag = 1;      //测试模式
uint8_t pid_reset_flag = 0;
uint8_t x_pid_flag = 0;
uint8_t theta_pid_flag = 0;
uint8_t setspeed_flag = 0;

float theta0 = 1440; //稳定位置

uint8_t x_last_flag = 0;
uint8_t data_flag = 0;

uint8_t question = 0;
uint8_t abs_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Wheel(int pwm);
int GetTimEnCoder(void);
void Code_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fgetc(FILE *f) {      
	uint8_t ch = 0;
	HAL_UART_Receive(&huart3,&ch,1,0xffff);
	return ch;
}

int fputc(int ch, FILE *f) {      
	HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xffff);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	    // TIM2_CH1(pwm)
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	    // TIM3_CH2(pwm)
  HAL_TIM_Base_Start_IT(&htim4);                // sysclock
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); //wheel_counter
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_IT(&hadc1);
	PID_init();
	Code_Init();
	
	while(HAL_UART_Receive_IT(&huart3, &rx_buf3, 1) != HAL_OK); // user
	
	//test_flag = 0;
	if(test_flag==1)
	{
		x_pid_flag = 1;
		x_last_flag = 0;
	}
	else
	{
		x_pid_flag = 1;
		x_last_flag = 0;
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*if(data_flag == 0) //数据被利用
		{
			HAL_ADC_Start_IT(&hadc1); //开启中断
		}*/
		
		/*if(test_flag==1)
		{
			Wheel(0);
			HAL_Delay(2000);
			Wheel(2000);
			HAL_Delay(2000);
			Wheel(0);
			HAL_Delay(2000);
			Wheel(-2000);
			HAL_Delay(2000);
		}*/
		

		//printf("Wheel=%d\r\n",GetTimEnCoder());
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Code_Init() //全局初始化
{
	__HAL_TIM_SET_COUNTER(&htim1,0); //重设编码器初值
	pid_reset_flag = 0; //相关变量清空
	Wheel(0); //电机停转
	data_flag = 0;
	ADC_Value = 0;
	x_pid_flag = 0;
	HAL_ADC_Start_IT(&hadc1); //开启中断
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //获取adc值
{
		//HAL_ADC_Stop_IT(&hadc1); //关闭中断
		if(data_flag == 0) //数据被利用
		{
			last_ADC_Value = ADC_Value; //上一次角度
			ADC_Value = HAL_ADC_GetValue(&hadc1)-theta0;
			if(ADC_Value>2125)
			{
				ADC_Value = -ADC_Value;
			}
			ADC_Value = ADC_Value*360.0/4096.0;
			if(ADC_Value>200 || ADC_Value<-200)ADC_Value=last_ADC_Value;
			data_flag = 1;
		}
		//HAL_ADC_Start_IT(&hadc1); //开启中断
}


int GetTimEnCoder() //读取编码器值
{
	iTimerEncoder = (short)(__HAL_TIM_GET_COUNTER(&htim1));
	__HAL_TIM_SET_COUNTER(&htim1,0);
	return iTimerEncoder;
}

void Wheel(int pwm) //直流电机驱动
{
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
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000);
	}
	else
	{
		if(pwm>0)
		{
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000-pwm);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000);
		}
		else if(pwm<0)
		{
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1000);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1000+pwm);
		}	
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int i = 0;
	
	static float x_theta = 0;
	static float theta_pwm = 0;
	static int pwm = 0;
	static int x_mark = 0;
	
	static uint8_t abs_count1 = 0;
	static uint8_t abs_count2 = 0;
	static uint8_t abs_count3 = 0;

	static uint16_t x_last = 0;
	static int theta_last = 0;

	static int speed = 0; //电机速度
	static int last_speed = 0; //上一次电机速度
	static float theta_speed = 0; //角速度

	if (htim == (&htim4)) // internal clock -> 1ms
	{
		
		/*if(x_last_flag==0)
		{
			x_mark = GetTimEnCoder();
			x_last_flag = 1;
		}
		else */
		if(x_pid_flag==1 && i%15==0) // 电机位置环
		{	
			speed = GetTimEnCoder();
			//if(speed>=10000 || speed<=-10000)speed=last_speed;
			
			//x_last = GetTimEnCoder();
			//last_speed = speed;
			
			x_theta = PID_x_update(0,speed); //用当前编码盘数据作位移数据
		}
			
		pwm = (int)PID_speed_update(x_theta*x_pid_flag,ADC_Value);
		data_flag = 0;
		HAL_ADC_Start_IT(&hadc1); //开启中断
		
		if(ADC_Value>-60 && ADC_Value<60)
		{
			Wheel(pwm);
		}
		else
		{
			Wheel(0);
		}

		if(test_flag == 1 && i%5 == 0)
		{
			//printf("%d\n",speed);
			printf("%.2f,%d,%.2f\n",ADC_Value,speed,x_theta);
			//printf("%f,%f\n",ADC_Value,theta_speed);
		}
		if(i>=100)
		{
			i=0;
		}
		i++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //串口回调函数
{
	if(huart == &huart3)
	{
		rData3[rDataCount3]=rx_buf3;
  		if(rx_buf3!=0x00)
		{
    		rDataCount3++;
    		if(rx_buf3==0x20)// (ascii)0x20 = " "
			{  
      			rDataFlag3 = 1;
				rDataCount3 = 0;
						if(strcmp((char*)rData3,"stop ")==0) // find stop flag
						{

						}
						memset(rData3,0,sizeof(rData3)); //清空缓存数组
					//for(int i=0;i<40;i++)rData3[i]='\0'; // clear buffer
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
