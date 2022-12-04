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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "pid.h"
#include "math.h"
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

char *c;
uint8_t rx_buf3[1];
uint8_t rx_buf1[1];
//uint8_t rxbuf3[1];

uint8_t rData1[40];
uint8_t rData3[40];
uint8_t rDataCount1 = 0;
uint8_t rDataCount3 = 0;
int rDataFlag1 = 0;
int rDataFlag3 = 0;

/*rpm and x of motor(from -360 to 360)*/
int setspeed1 = 160;
int setspeed2 = 160;
int x_set1 = 0;
int x_set2 = 0;
float theta = 0;
int b = 200;
int theta_speed = 0;
int b_speed = 0;
int string_flag = 0;
int location[4] = {0};

/*pulses of each round*/
const int round_pulse = 390;

/*pid flag*/
int x_pid_flag = 0;
int theta_pid_flag = 0;
int b_pid_flag = 0;
int setspeed_flag = 1;
int question_flag = 0; // 0 means Q1, 1 means Q2

/*count pulse of motor1 and motor2*/
int cont_value1 = 0;
int cont_value2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_CHANGE(int uart_flag);
void Wheel(int num,int pwm);
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
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	    // TIM2_CH1(pwm)
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	    // TIM2_CH2(pwm)
  	HAL_TIM_Base_Start_IT(&htim4);                // sysclock
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); //tim1 channel1 capture enable
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING); //tim1 channel2 capture enable
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1); //tim1 channel1 enable
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2); //tim1 channel2 enable
	PID_init();
	
	while(HAL_UART_Receive_IT(&huart3, rx_buf3, 1) != HAL_OK);
	while(HAL_UART_Receive_IT(&huart1, rx_buf1, 1) != HAL_OK);
	
	Wheel(1,0);
	Wheel(2,0);
	
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		static int i = 0;
		static float speed1 = 0;
		static float speed2 = 0;
		static int x_speed1 = 0;
		static int x_speed2 = 0;
		static int x_cont1 = 0;
		static int x_cont2 = 0;
		static int pwm1 = 0;
		static int pwm2 = 0;

    if (htim == (&htim4)) // internal clock -> 10ms
    {	
			/*setspeed PID processing*/
			/*Delta_speed>0 means clockwise*/
			if(i%10==0) // 0.10s theta pid (Using "%" matters!)
			{
				if(theta_pid_flag==1)
				{
					theta_speed = PID_theta_update(theta);
				}
				else
				{
					theta_speed = 0;
				}
				
				if(b_pid_flag==1)
				{
					b_speed = PID_b_update(b,theta);
				}
				else
				{
					b_speed = 0;
				}
			}

			if(i%8==0) // 0.08s x pid (Using "%" matters!)
			{
				if(x_pid_flag==1)
				{
					x_speed1 = PID_x_update(x_set1,cont_value1,1);
					x_speed2 = PID_x_update(x_set2,cont_value2,2);
				}
				else
				{
					x_speed1 = 0;
					x_speed2 = 0;
				}
			}

			if(i%2==0) // speed pid avaliable(T=0.02s)
			{
				if(x_pid_flag==0)
				{
					/*present speed*/
					speed1 = cont_value1*60.0/(0.02*round_pulse);
					speed2 = cont_value2*60.0/(0.02*round_pulse);
					cont_value1 = 0;
					cont_value2 = 0;
				}
				else
				{
					/*present speed*/
					speed1 = (cont_value1-x_cont1)*60.0/(0.02*round_pulse);
					speed2 = (cont_value2-x_cont2)*60.0/(0.02*round_pulse);
					x_cont1 = cont_value1;
					x_cont2 = cont_value2;
					if(speed1 == 0 && speed2 == 0)x_pid_flag = 0;
				}
				pwm1 = PID_speed_update(setspeed1*setspeed_flag+theta_speed-b_speed+x_speed1,speed1,pwm1,1);
				pwm2 = PID_speed_update(setspeed2*setspeed_flag-theta_speed+b_speed+x_speed2,speed2,pwm2,2);
				Wheel(1,pwm1);
				Wheel(2,pwm2);
			}

			if(i>100) // 0.5s reset i
			{
				/*return pulse count*/
				//printf("%d, %d\r\n",pwm1,pwm2);
				i=0;
			}
			i++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		//DataGet1(rx_buf1[0]);
	
		rData3[rDataCount3]=rx_buf3[0];
  		if(rx_buf3[0]!=0x00)
		{
    		rDataCount3++;
    		if(rx_buf3[0]==0x0A)// (ascii)0x0A = "\n"
			{  
				//printf("%s",rData);
      			rDataFlag3 = 1;
						rDataCount3 = 0;
				
						if(string_flag == 1)
						{
							location[0] = atoi((char*)rData3);
							char* p = strchr((char*)rData3,' ');
							p++;
							location[1] = atoi((char*)p);
							string_flag++;
						}
						else if(string_flag == 2)
						{
							location[2] = atoi((char*)rData3);
							char* p = strchr((char*)rData3,' ');
							p++;
							location[3] = atoi((char*)p);
							string_flag=0;
							
							double theta0 = 180/3.1415926*atan((1.0*location[0]-1.0*location[2])/(1.0*location[1]-1.0*location[3]));
							int mean_x = (location[0]+location[2])/2;
							int mean_y = (location[1]+location[3])/2;
							x_set1 = (mean_y+(mean_x-150)+theta0/2)*10;
							x_set2 = (mean_y-(mean_x-150)-theta0/2)*10;
							
							printf("set1=%d,set2=%d",x_set1,x_set2);
							
							x_pid_flag = 1;

							switch(question_flag)// restart or stop depend on the question
							{
								case 0: // Q1
									break;
								case 1: // Q2
									HAL_Delay(5000);
									setspeed_flag = 1;
									theta_pid_flag = 1;
									b_pid_flag = 1;
									x_pid_flag = 0;
									break;
							}
						}
						
						if(strcmp((char*)rData3,"stop ")==0) // start
						{
							string_flag = 1;
							setspeed_flag = 0;
							theta_pid_flag = 0;
							b_pid_flag = 0;
						}
						else if(string_flag==0)
						{
							theta = strtod((char*)rData3,&c);
							/*read b*/
							b = atoi(++c);
							if(theta_pid_flag == 1)printf("%.2f,%d\r\n",theta,b);
						}
					for(int i=0;i<40;i++)rData3[i]='\0'; // clear buffer
    		}
  		}
		while(HAL_UART_Receive_IT(&huart3, rx_buf3, 1) != HAL_OK);
	}
	
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
						
						if(strcmp((char*)rData1,"start ")==0) // start
						{
							//memset(s,'\0',strlen(s));
							printf("start\n");
							setspeed_flag = 1;
							x_pid_flag = 0;
							theta_pid_flag = 1;
							b_pid_flag = 1;
						}
						else if(strcmp((char*)rData1,"stop ")==0) // stop
						{
							printf("stop\n");
							setspeed_flag = 0;
							x_pid_flag = 0;
							theta_pid_flag = 0;
							b_pid_flag = 0;
						}
						else if(strcmp((char*)rData1,"switch ")==0) // switch question
						{
							question_flag = 1 - question_flag;
							printf("question %d\n",question_flag+1);
						}
						for(int i=0;i<40;i++)rData1[i]='\0'; // clear buffer
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
