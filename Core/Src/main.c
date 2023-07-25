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

/*�������ݴ��������*/
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

/*����2DMA����*/
uint8_t rx_buffer[255];
volatile uint8_t rx_len = 0; 
volatile uint8_t recv_end_flag = 0; 

/*�������û�����*/
unsigned char buf[64];

/*��������ر�ʶλ�ṹ��*/
extern _Q Q;
uint8_t test = 0; //���Ա�ʶ
uint8_t message = 0; //ͨѶ�ź�
uint8_t debug = 0; //�����ñ�ʶλ
extern int stop_count; //�����ʶ
extern uint8_t status; //��ǰ״̬

/*�������ٶ�*/
int speed_l = 0;
int speed_r = 0;

/*pid��ز���*/
uint8_t x_pid = 0; //λ��pid����
uint8_t x_start = 0; //λ��pid��ʼ��
uint8_t wheel_enable = 0; //�����תʹ��
uint8_t pos_pid_flag = 0; //λ�û�pid��ʶ

/*ѭ����ز���*/
float position = 0;
uint8_t data_flag = 0; //ѭ����Ϣ��Ч��ʶ
char *endptr;

/*����˶�����趨ֵ*/
int setspeed_l = 0;
int setspeed_r = 0;
int x_set_l = 0;
int x_set_r = 0;
int x_cont_L = 0; //������λ�Ƽ�¼
int x_cont_R = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Wheel(int pwm,uint8_t flag); //�����������
int GetTimEnCoder(uint8_t flag); //�����������ȡ����
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*�����ض���*/
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
	
	/*������ʱ��2,��ʱ��3������*/
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	
	/*������ʱ��4�ж�*/
	HAL_TIM_Base_Start_IT(&htim4);
	/*������ʱ��4����PWM*/
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	/*������ʱ��1����PWM*/
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
	/*�رն�ʱ��1�������*/
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	
	/*��������Ϊ0*/
	Wheel(0,1);
	Wheel(0,2);
	
	/*��������1,����3�ж�*/
	while(HAL_UART_Receive_IT(&huart1, &rx_buf1, 1) != HAL_OK)printf("usart1 ok\r\n");
	while(HAL_UART_Receive_IT(&huart3, &rx_buf3, 1) != HAL_OK)printf("usart3 ok\r\n");
	
	/*ʹ�ܴ���2DMA*/
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
		/*DMA����*/
		/*if(recv_end_flag==1)
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13); //LED�Ʒ�ת
			recv_end_flag = 0;
			printf("has received the data\r\n");
			printf("data:%s length:%x\r\n",rx_buffer,rx_len);
		}*/
		
		if(test==1) //����ģʽ
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
					/*���ֱ�߲��Գ���*/
					wheel_enable = 2; //���ݺ�����ȡ
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
					/*ת�����(xpid)*/
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
		HAL_Delay(1); //��ѭ�����и�ֵ�������±������,������ȥ
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
void Wheel(int pwm,uint8_t flag) //ֱ���������
{
	uint32_t Wheel_pwm_1 = 0;
	uint32_t Wheel_pwm_2 = 0;
	
	switch(flag)
	{
		case 1: //����
			Wheel_pwm_1 = TIM_CHANNEL_1;
			Wheel_pwm_2 = TIM_CHANNEL_2;
			break;
		default: //����
			Wheel_pwm_1 = TIM_CHANNEL_3;
			Wheel_pwm_2 = TIM_CHANNEL_4;
			break;
	}
	/*pwm�޷�*/
	if(pwm>1000)
	{
		pwm = 1000;
	}
	else if(pwm<-1000)
	{
		pwm = -1000;
	}

	/*�ߵ�ƽֹͣ*/
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

int GetTimEnCoder(uint8_t flag) //�����������ȡ
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
		case 1: //���ֱ�����
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
	if(TIM_p == &htim2)iTimerEncoder = -iTimerEncoder; //����Ӳ������
	if(iTimerEncoder>32000 && *coder_enptr<-32000)(*add_enptr)--;
	else if(iTimerEncoder<-32000 && *coder_enptr>32000)(*add_enptr)++;
	*coder_enptr = iTimerEncoder;
	//__HAL_TIM_SET_COUNTER(TIM_p,0); //��������λ
	return iTimerEncoder+65536*(*add_enptr);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //��ʱ�жϻص�����
{
	static int i = 0;
	static float pos_speed = 0;
	/*������xpid�����ٶ�*/
	static float x_speed_l = 0;
	static float x_speed_r = 0;
	/*������pwm*/
	static int pwm_l = 0;
	static int pwm_r = 0;
	/*�ϴα�����ֵ*/
	static int last_x_l = 0;
	static int last_x_r = 0;

	if (htim == (&htim4)) // internal clock -> 2ms
	{	
		if(stop_count>0)stop_count--; //ͨѶ������ʱ
		
		Status_Deal(0);
		
		/*λ�û�pid����*/
		if(data_flag == 1 && pos_pid_flag == 1)
		{
			pos_speed = PID_pos_update(position);
			data_flag = 0;
		}
		
		/*����ٶȻ�ȡ*/
		if(i%3==0) //4ms����һ���ٶ�
		{
			speed_l = GetTimEnCoder(1)-last_x_l;
			speed_r = GetTimEnCoder(2)-last_x_r;
			
			last_x_l+=speed_l;
			last_x_r+=speed_r;
		
			//printf("%d,%d\n",last_x_l,speed_l);
		}

		
		/*λ��pid*/
		if(x_pid == 1)
		{
			if(x_start == 1) //λ��pid��ʼ��
			{
				x_cont_L = GetTimEnCoder(1);
				x_cont_R = GetTimEnCoder(2);
				x_start = 0;
			}
			else if(i%5==0) //10ms����һ��
			{
				/*λ�Ƽ���(�ٶȻ���)*/
				x_cont_L = GetTimEnCoder(1);
				x_cont_R = GetTimEnCoder(2);
				x_speed_l = PID_x_update(x_set_l,x_cont_L,1);
				x_speed_r = PID_x_update(x_set_r,x_cont_R,2);
			}
		}
		
		/*���pwm����*/
		pwm_l = PID_speed_update(setspeed_l+x_speed_l*x_pid+pos_speed*pos_pid_flag,speed_l,1);
		pwm_r = PID_speed_update(setspeed_r+x_speed_r*x_pid-pos_speed*pos_pid_flag,speed_r,2);
		
		/*���ʹ��*/
		if(wheel_enable == 1) //pwm����
		{
			Wheel(pwm_l,1);
			Wheel(pwm_r,2);
		}
		else if(wheel_enable == 2) //���ݺ�����ȡ
		{
			Wheel(setspeed_l,1);
			Wheel(setspeed_r,2);
		}
		else //���ͣת
		{
			Wheel(0,1);
			Wheel(0,2);
		}
		
		if(i>=100)i=0;
		else i++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //���ڻص�����
{
	if(huart == &huart1) //�û�����(����/������)
	{
		rData1[rDataCount1]=rx_buf1;
  	if(rx_buf1!=0x00)
		{
    	rDataCount1++;
    	if(rx_buf1==0x20)// (ascii)0x20 = " "
			{  
      	rDataFlag1 = 1;
				rDataCount1 = 0;
				if(strcmp((char*)rData1,"0 ")==0) //��λָ��
				{
					HMIReset(); //���⸴λ
					
					message = 0; //ͨѶ��λ
					status = 0; //״̬��λ
					stop_count = 0;
					
					wheel_enable = 0; //���õ��
					
					setspeed_l = 0; //Ѳ�ߺ�λ�û���λ
					setspeed_r = 0;
					pos_pid_flag = 0;
					
					x_pid = 0; //xpid��λ
					x_start = 0;
					x_set_l = 0;
					x_set_r = 0;
					x_cont_L = 0;
					x_cont_R = 0;
					
					debug = 0; //���Ը�λ
					test = 0;
					
					HAL_UART_Transmit(&huart3,(uint8_t*)"R",1,100); //��ݮ�ɸ�λ
				}
				else if(rData1[0] >= '1' && rData1[0]<='3' && rData1[1]==' ') //��Ŀ��Ϣ
				{
					HAL_UART_Transmit(&huart3,rData1,1,50); //���Ͷ�Ӧ��Ŀ����ݮ��
					message = 1;
					HMIReset();
					Q.Q=rData1[0]-'0'; //��Ŀ��ʶ��ֵ
				}
				else if(strcmp((char*)rData1,"t1 ")==0) //���Կ���ģʽ
				{
					test = 1;
				}
				else if(strcmp((char*)rData1,"t0 ")==0) //���Թر�ģʽ
				{
					test = 0;
				}
				else if(rData1[0] == 'D' && rData1[2] == ' ') //Debug��Ϣ("Dx ")
				{
					if(test==1) //���ڲ���ģʽ�½���debug
					{
						debug = rData1[1] - '0';
					}
					else //���ڷ���?
					{
						
					}
				}
				memset(rData1,0,sizeof(rData1)); //��ջ�������
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
				memset(rData2,0,sizeof(rData2)); //��ջ�������
    	}
  	}
		while(HAL_UART_Receive_IT(&huart2, &rx_buf2, 1) != HAL_OK);
	}*/
	else if(huart == &huart3) //��ݮ�ɴ���
	{
		rData3[rDataCount3]=rx_buf3;
  	if(rx_buf3!=0x00)
		{
    	rDataCount3++;
    	if(rx_buf3==0x20)// (ascii)0x20 = " "
			{  
      	rDataFlag3 = 1;
				rDataCount3 = 0;
				if(strcmp((char*)rData3,"s ")==0) //ѭ��ֹͣ�ź�
				{
					
				}
				else
				{
					position = strtof((char*)rData3,&endptr);
					endptr = (char*)rData3;
					data_flag = 1; // ������Ч
				}
				memset(rData3,0,sizeof(rData3)); //��ջ�������
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
