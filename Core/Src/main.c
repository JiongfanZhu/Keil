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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "my_uart.h"
#include "pid.h"
#include "my_hmi.h"
#include "car_camera.h"
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

/*uint8_t rx_buf2;
uint8_t rData2[30];
uint8_t rDataCount2 = 0;
int rDataFlag2 = 0;*/

uint8_t rx_buf3;
uint8_t rData3[30];
uint8_t rDataCount3 = 0;
int rDataFlag3 = 0;

/*串口2DMA数据*/
uint8_t rx_buffer[255];
volatile uint8_t rx_len = 0; 
volatile uint8_t recv_end_flag = 0; 

/*串口屏用缓冲区*/
unsigned char buf[64];

/*问题与相关标识位结构体*/
extern _Q Q;
uint8_t test = 0; //测试标识
uint8_t message = 0; //通讯信号
uint8_t debug = 0; //调试用标识位
extern int stop_count; //缓冲标识
extern uint8_t status; //当前状态

/*左右轮速度*/
int speed_l = 0;
int speed_r = 0;

/*pid相关参数*/
uint8_t x_pid = 0; //位移pid启动
uint8_t x_start = 0; //位移pid初始化
uint8_t wheel_enable = 0; //电机旋转使能
uint8_t pos_pid_flag = 0; //位置环pid标识

/*循迹相关参数*/
float position = 0;
uint8_t data_flag = 0; //循迹信息有效标识
char *endptr;

/*电机运动相关设定值*/
int setspeed_l = 0;
int setspeed_r = 0;
int x_set_l = 0;
int x_set_r = 0;
int x_cont_L = 0; //左右轮位移记录
int x_cont_R = 0;

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
  MX_DMA_Init();
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
	
	/*开启串口1,串口3中断*/
	while(HAL_UART_Receive_IT(&huart1, &rx_buf1, 1) != HAL_OK)printf("usart1 ok\r\n");
	while(HAL_UART_Receive_IT(&huart3, &rx_buf3, 1) != HAL_OK)printf("usart3 ok\r\n");
	
	/*使能串口2DMA*/
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  //HAL_UART_Receive_DMA(&huart2,rx_buffer,255);
	
	if(test==1)
	{

	}
	else
	{

	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*DMA测试*/
		/*if(recv_end_flag==1)
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13); //LED灯反转
			recv_end_flag = 0;
			printf("has received the data\r\n");
			printf("data:%s length:%x\r\n",rx_buffer,rx_len);
		}*/
		
		if(test==1) //测试模式
		{
			switch(debug)
			{
				case 0:
					wheel_enable = 0;
					setspeed_l = 0;
					setspeed_r = 0;
					pos_pid_flag = 0;
					x_pid = 0;
					break;
				case 1:
					/*电机直线测试程序*/
					wheel_enable = 2; //传递函数获取
					setspeed_l = 400;
					setspeed_r = 400;
					HAL_Delay(2000);
					setspeed_l = 0;
					setspeed_r = 0;
					HAL_Delay(2000);
					setspeed_l = -400;
					setspeed_r = -400;
					HAL_Delay(2000);
					setspeed_l = 0;
					setspeed_r = 0;
					HAL_Delay(2000);
					//debug=0;
					break;
				case 3:
					/*转向测试(xpid)*/
					x_set_l = 240;
					x_set_r = -240;
					x_pid = 1;
					x_start = 1;
					wheel_enable = 1;
					HAL_Delay(2000);
					x_pid = 0;
				
					x_set_l = -240;
					x_set_r = 240;
					x_pid = 1;
					x_start = 1;
					wheel_enable = 1;
					HAL_Delay(2000);
					x_pid = 0;
					debug=0;
					break;
			}
		}
		else
		{
			if(stop_count>0)stop_count--;
		}
		HAL_Delay(1); //主循环仅有赋值语句情况下必须添加,否侧进不去
		//printf("yes\r\n");
		//HAL_UART_Transmit(&huart1,(uint8_t*)"s",1,100);

		
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
			__HAL_TIM_SET_COMPARE(&htim4,Wheel_pwm_1,1000-pwm);
			__HAL_TIM_SET_COMPARE(&htim4,Wheel_pwm_2,1000);
		}
		else if(pwm<0)
		{
			__HAL_TIM_SET_COMPARE(&htim4,Wheel_pwm_1,1000);
			__HAL_TIM_SET_COMPARE(&htim4,Wheel_pwm_2,1000+pwm);
		}	
	}
}

int GetTimEnCoder(uint8_t flag) //电机编码器读取
{
	TIM_HandleTypeDef *TIM_p;
	static int *add_enptr;
	static int *coder_enptr;
	
	static int tim_add_l = 0;
	static int tim_add_r = 0;
	static int last_coder_l = 0;
	static int last_coder_r = 0;
	
	switch(flag)
	{
		case 1: //左轮编码器
			TIM_p = &htim2;
			add_enptr = &tim_add_l;
			coder_enptr = &last_coder_l;
			break;
		default:
			TIM_p = &htim3;
			add_enptr = &tim_add_r;
			coder_enptr = &last_coder_r;
			break;
	}
	
	int iTimerEncoder = (short)(__HAL_TIM_GET_COUNTER(TIM_p));
	if(TIM_p == &htim2)iTimerEncoder = -iTimerEncoder; //左轮硬件错连
	if(iTimerEncoder>32000 && *coder_enptr<-32000)(*add_enptr)--;
	else if(iTimerEncoder<-32000 && *coder_enptr>32000)(*add_enptr)++;
	*coder_enptr = iTimerEncoder;
	//__HAL_TIM_SET_COUNTER(TIM_p,0); //编码器复位
	return iTimerEncoder+65536*(*add_enptr);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //定时中断回调函数
{
	static int i = 0;
	static float pos_speed = 0;
	/*左右轮xpid计算速度*/
	static float x_speed_l = 0;
	static float x_speed_r = 0;
	/*左右轮pwm*/
	static int pwm_l = 0;
	static int pwm_r = 0;
	/*上次编码器值*/
	static int last_x_l = 0;
	static int last_x_r = 0;

	if (htim == (&htim4)) // internal clock -> 2ms
	{	
		if(stop_count>0)stop_count--; //通讯程序延时
		
		Status_Deal(0);
		
		/*位置环pid控制*/
		if(data_flag == 1 && pos_pid_flag == 1)
		{
			pos_speed = PID_pos_update(position);
			data_flag = 0;
		}
		
		/*电机速度获取*/
		if(i%3==0) //4ms更新一次速度
		{
			speed_l = GetTimEnCoder(1)-last_x_l;
			speed_r = GetTimEnCoder(2)-last_x_r;
			
			last_x_l+=speed_l;
			last_x_r+=speed_r;
		
			//printf("%d,%d\n",last_x_l,speed_l);
		}

		
		/*位移pid*/
		if(x_pid == 1)
		{
			if(x_start == 1) //位移pid初始化
			{
				x_cont_L = GetTimEnCoder(1);
				x_cont_R = GetTimEnCoder(2);
				x_start = 0;
			}
			else if(i%5==0) //10ms更新一次
			{
				/*位移计算(速度积分)*/
				x_cont_L = GetTimEnCoder(1);
				x_cont_R = GetTimEnCoder(2);
				x_speed_l = PID_x_update(x_set_l,x_cont_L,1);
				x_speed_r = PID_x_update(x_set_r,x_cont_R,2);
			}
		}
		
		/*电机pwm计算*/
		pwm_l = PID_speed_update(setspeed_l+x_speed_l*x_pid+pos_speed*pos_pid_flag,speed_l,1);
		pwm_r = PID_speed_update(setspeed_r+x_speed_r*x_pid-pos_speed*pos_pid_flag,speed_r,2);
		
		/*电机使能*/
		if(wheel_enable == 1) //pwm控制
		{
			Wheel(pwm_l,1);
			Wheel(pwm_r,2);
		}
		else if(wheel_enable == 2) //传递函数获取
		{
			Wheel(setspeed_l,1);
			Wheel(setspeed_r,2);
		}
		else //电机停转
		{
			Wheel(0,1);
			Wheel(0,2);
		}
		
		if(i>=100)i=0;
		else i++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //串口回调函数
{
	if(huart == &huart1) //用户串口(蓝牙/串口屏)
	{
		rData1[rDataCount1]=rx_buf1;
  	if(rx_buf1!=0x00)
		{
    	rDataCount1++;
    	if(rx_buf1==0x20)// (ascii)0x20 = " "
			{  
      	rDataFlag1 = 1;
				rDataCount1 = 0;
				if(strcmp((char*)rData1,"0 ")==0) //复位指令
				{
					HMIReset(); //问题复位
					
					message = 0; //通讯复位
					status = 0; //状态复位
					stop_count = 0;
					
					wheel_enable = 0; //禁用电机
					
					setspeed_l = 0; //巡线和位置环复位
					setspeed_r = 0;
					pos_pid_flag = 0;
					
					x_pid = 0; //xpid复位
					x_start = 0;
					x_set_l = 0;
					x_set_r = 0;
					x_cont_L = 0;
					x_cont_R = 0;
					
					debug = 0; //测试复位
					test = 0;
					
					HAL_UART_Transmit(&huart3,(uint8_t*)"R",1,100); //树莓派复位
				}
				else if(rData1[0] >= '1' && rData1[0]<='3' && rData1[1]==' ') //题目信息
				{
					HAL_UART_Transmit(&huart3,rData1,1,50); //发送对应题目至树莓派
					message = 1;
					HMIReset();
					Q.Q=rData1[0]-'0'; //题目标识赋值
				}
				else if(strcmp((char*)rData1,"t1 ")==0) //测试开启模式
				{
					test = 1;
				}
				else if(strcmp((char*)rData1,"t0 ")==0) //测试关闭模式
				{
					test = 0;
				}
				else if(rData1[0] == 'D' && rData1[2] == ' ') //Debug信息("Dx ")
				{
					if(test==1) //仅在测试模式下进行debug
					{
						debug = rData1[1] - '0';
					}
					else //用于反馈?
					{
						
					}
				}
				memset(rData1,0,sizeof(rData1)); //清空缓存数组
    	}
  	}
		while(HAL_UART_Receive_IT(&huart1, &rx_buf1, 1) != HAL_OK);
	}
	/*else if(huart == &huart2)
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
	}*/
	else if(huart == &huart3) //树莓派串口
	{
		rData3[rDataCount3]=rx_buf3;
  	if(rx_buf3!=0x00)
		{
    	rDataCount3++;
    	if(rx_buf3==0x20)// (ascii)0x20 = " "
			{  
      	rDataFlag3 = 1;
				rDataCount3 = 0;
				if(strcmp((char*)rData3,"s ")==0) //循迹停止信号
				{
					
				}
				else
				{
					position = strtof((char*)rData3,&endptr);
					endptr = (char*)rData3;
					data_flag = 1; // 数据有效
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
