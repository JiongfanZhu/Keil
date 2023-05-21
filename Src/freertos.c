/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "stdio.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define round_pulse 390
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
int setspeed = 160;
int x_set1 = 0;
int x_set2 = 0;
float theta = 0;
int b = 200;
int theta_speed = 0;
int b_speed = 0;

int x_pid_flag = 0;
int theta_pid_flag = 0;
int b_pid_flag = 0;
int setspeed_flag = 1;

extern int cont_value1;
extern int cont_value2;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId UART1_task1Handle;
osThreadId UART1_task2Handle;
osThreadId PID_task1Handle;
osSemaphoreId UART1_flagHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void Wheel(int num,int pwm);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void UART1_send(void const * argument);
void UART1_receive(void const * argument);
void PID_update(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of UART1_flag */
  osSemaphoreDef(UART1_flag);
  UART1_flagHandle = osSemaphoreCreate(osSemaphore(UART1_flag), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UART1_task1 */
  osThreadDef(UART1_task1, UART1_send, osPriorityIdle, 0, 128);
  UART1_task1Handle = osThreadCreate(osThread(UART1_task1), NULL);

  /* definition and creation of UART1_task2 */
  osThreadDef(UART1_task2, UART1_receive, osPriorityIdle, 0, 128);
  UART1_task2Handle = osThreadCreate(osThread(UART1_task2), NULL);

  /* definition and creation of PID_task1 */
  osThreadDef(PID_task1, PID_update, osPriorityIdle, 0, 128);
  PID_task1Handle = osThreadCreate(osThread(PID_task1), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_UART1_send */
/**
* @brief Function implementing the UART1_task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART1_send */
void UART1_send(void const * argument)
{
  /* USER CODE BEGIN UART1_send */
  /* Infinite loop */
  for(;;)
  {
////////////////////////////////////////////////
		//串口打印测试程序
		portDISABLE_INTERRUPTS();
		printf("hello world!\r\n");						
		portENABLE_INTERRUPTS();
////////////////////////////////////////////////
    osDelay(500);
  }
  /* USER CODE END UART1_send */
}

/* USER CODE BEGIN Header_UART1_receive */
/**
* @brief Function implementing the UART1_task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART1_receive */
void UART1_receive(void const * argument)
{
  /* USER CODE BEGIN UART1_receive */
  /* Infinite loop */
  for(;;)
  {
		if(osSemaphoreWait(UART1_flagHandle,50)!=0)							//等待二值信号量,接收时间超过50ms就返回
		{
			if(rDataFlag1 == 1)																//数据接收完成
			{
/////////////////////////////////////////////////////////////////
				//将接受数据传回测试程序
				portDISABLE_INTERRUPTS();
				for(int i = 0; i<rDataCount1; i++)							//打印接收数组存储的内容
					printf("%c",rData1[i]);	
				printf("\r\n");																//打印完成换行
				portENABLE_INTERRUPTS();
				rDataFlag1 = 0;																//接收标志清零
				rDataCount1 = 0;															//接收计数清零
				memset(rData1 ,0, MAX_REC_LENGTH);						//清空接收数组
///////////////////////////////////////////////////////////////////
			}
		}
    osDelay(1);
  }
  /* USER CODE END UART1_receive */
}

/* USER CODE BEGIN Header_PID_update */
/**
* @brief Function implementing the PID_task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PID_update */
void PID_update(void const * argument)
{
  /* USER CODE BEGIN PID_update */
	static int i = 0;
	static float speed1 = 0;
	static float speed2 = 0;
	static int x_speed1 = 0;
	static int x_speed2 = 0;
	static int x_cont1 = 0;
	static int x_cont2 = 0;
	static int pwm1 = 0;
	static int pwm2 = 0;
  /* Infinite loop */
  for(;;)
  {
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
					b_speed = PID_b_update(b);
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
				pwm1 = PID_speed_update(setspeed*setspeed_flag+theta_speed-b_speed+x_speed1,speed1,pwm1,1);
				pwm2 = PID_speed_update(setspeed*setspeed_flag-theta_speed+b_speed+x_speed2,speed2,pwm2,2);
				portDISABLE_INTERRUPTS();
				printf("speed1=%f,speed2=%f",speed1,speed2);						
				portENABLE_INTERRUPTS();
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
			osDelay(1);
  }
  /* USER CODE END PID_update */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

