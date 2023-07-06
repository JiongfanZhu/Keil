#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/timer.h"

#include "pid.h"
#include "my_uart.h"
#include "hand_hand.h"

void Sysint_init(void);
void PWM_init(void);
void QEI_init(void);
void USART_init(void);
void GPIO_init(void);
void TIMER0_IRQHandler(void);
void UART5_Handler(void);
void UART1_Handler(void);
void Wheel_set(float pwm,int num);
void USART_PID_Adjust(void);
float Get_Data(void);
void f_char_printf(float Xangle);
uint8_t Drug_Read(void);

#define SETSPEED 350
#define round_pulse 390 //������ÿȦ������
#define K_round 1000.0 //pwm�任ϵ��

int x_pid_flag = 0; //��ʼ�ر�xpid
int pos_pid_flag = 0;
int setspeed_flag = 0;
uint8_t pid_flag = 0;

float setspeed = SETSPEED;
float x_set1 = 0;
float x_set2 = 0;
float pos = 0;

float pos_speed = 0;

float speed1 = 0;
float speed2 = 0;

uint8_t rx_buf5;
uint8_t rx_buf1;

uint8_t rData5[30];
uint8_t rData1[30];
uint8_t rDataCount5 = 0;
uint8_t rDataCount1 = 0;
int rDataFlag5 = 0;
int rDataFlag1 = 0;

extern uint8_t status_hand;
uint8_t route_flag = 0;     //ҩƷ״̬��־λ
uint8_t test_flag = 1;      //����ģʽ
uint8_t LED_flag = 0;       //0����,1�����,2���̵�
int uart_flag = 0;      //�Ƿ�����Чֹͣ��Ϣ
int x_last_flag = 0;    //xpid��ʼ׼����ʶ
uint8_t data_flag = 0; //�ǶȽؾ���Ϣ��Ч��ʶ
uint8_t pid_reset_flag = 0; //�ٶ�pid���ñ�ʶ
uint8_t time_count = 0;
uint8_t keep = 0; //����ѭ����ʶ

uint8_t question = 0; //��Ŀѡ��,Ĭ��Ϊ��������0,���(1)1,���(2)2
uint32_t x_last1 = 0x7fffffff;
uint32_t x_last2 = 0x7fffffff;

char * endptr;

int main(void)
{
    //����ϵͳʱ�ӣ�ʹ���ⲿ��������PLL����Ƶϵ��2.5��ϵͳʱ��80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    PWM_init();
    QEI_init();
    USART_init();
    PID_init();
    Sysint_init();
    GPIO_init();

    test_flag = 0;
    if(test_flag == 0)StatusReset();
    Wheel_set(0,1);
    Wheel_set(0,2);
    //SysCtlDelay(2*SysCtlClockGet()/3);

    while(1)
    {
        if(test_flag == 1) //���Գ���
        {
            /*λ�û�����*/
            /*pos_pid_flag = 0;
            pid_flag = 0;

            x_set1 = 950;
            x_set2 = -950;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(SysCtlClockGet()/3);
            x_pid_flag = 0;
            pos_pid_flag = 1;
            pid_flag = 1;
            SysCtlDelay(SysCtlClockGet()/3);
            pos_pid_flag = 0;
            pid_flag = 0;

            x_set1 = -1200;
            x_set2 = 1200;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(SysCtlClockGet()/3);
            x_pid_flag = 0;
            pos_pid_flag = 1;
            pid_flag = 1;
            SysCtlDelay(SysCtlClockGet()/3);*/

            /*ֱ���ٶȱջ�����*/
            /*pid_flag = 1;
            setspeed_flag = 1;
            setspeed = 400;
            SysCtlDelay(SysCtlClockGet()/3);
            setspeed = 0;
            SysCtlDelay(SysCtlClockGet()/3);
            setspeed = -400;
            SysCtlDelay(SysCtlClockGet()/3);
            setspeed = 0;
            SysCtlDelay(SysCtlClockGet()/3);*/

            /*����ɲ������*/
            /*x_pid_flag = 0;
            pid_flag = 1;
            setspeed_flag = 1;
            setspeed = 400;
            SysCtlDelay(SysCtlClockGet()/3);
            setspeed_flag = 0;
            x_set1 = 450;
            x_set2 = 450;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(SysCtlClockGet()/3);

            x_pid_flag = 0;
            pid_flag = 1;
            setspeed_flag = 1;
            setspeed = -400;
            SysCtlDelay(SysCtlClockGet()/3);
            setspeed_flag = 0;
            x_set1 = -150;
            x_set2 = -150;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(SysCtlClockGet()/3);
            x_set1 = -500;
            x_set2 = -500;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(SysCtlClockGet()/3);*/

            /*ֱ�߱ջ����Գ���*/
            /*x_set1 = 500;
            x_set2 = 500;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(SysCtlClockGet()/3);
            x_set1 = -500;
            x_set2 = -500;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(SysCtlClockGet()/3);*/
            
            /*����ת����*/
            /*x_set1 = 950;
            x_set2 = -950;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(3*SysCtlClockGet()/3);
            x_set1 = -1200;
            x_set2 = 1200;
            x_pid_flag = 1;
            x_last_flag = 0;
            SysCtlDelay(3*SysCtlClockGet()/3);*/

            /*��ͷ����*/
            /*x_set1 = 1750;
            x_set2 = -1750;
            x_last_flag = 0;
            x_pid_flag = 1;
            SysCtlDelay(3*SysCtlClockGet()/3);*/


        }
        else if(question == 0)
        {
            switch(LED_flag)
            {
                case 0: //Ϩ��
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2, 0);
                    break;
                case 1: //�����
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2, 2);
                    break;
                case 2: //�̵���
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2, 4);
            }
        }
    }

}

void Sysint_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);       //ʹ������
    //���ö�ʱ������Timer0��ֲ�����TIMERAΪ�������¼���ģʽ
    TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    //����Ԥ��Ƶֵ
    TimerPrescaleSet(TIMER0_BASE, TIMER_A,199); //80MHz->400kHz
    //����װ��ֵ
    TimerLoadSet(TIMER0_BASE, TIMER_A,3999); //400kHz->100Hz,����10msһ�ο��Ƶ�������
    //ע���жϷ�����
    TimerIntRegister(TIMER0_BASE,TIMER_A, TIMER0_IRQHandler);
    //������ʱ��A��ʱ�ж�
    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
    //�����ж����ȼ�
    IntPrioritySet(INT_TIMER0A, 0);
    //ʹ���ж�
    IntEnable(INT_TIMER0A);
    IntMasterEnable();
    //ʹ�ܶ�ʱ��
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void PWM_init()
{
    //PF1��Ӧ����ģ��1PWM5,�ɷ�����2����
    //PF2��Ӧ����ģ��1PWM6,�ɷ�����3����
    //PWM���ò���
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);//PWMʱ��80M/64,��ϵͳʱ�ӷ�Ƶ�õ�,PWMʱ������Ϊ0.8us
    //ʹ��ʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);         //ʹ��PWMģ��1ʱ��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);        //ʹ��GPIOFʱ��
    //ʹ�����Ÿ���PWM����
    //PF1,PF1Ϊģ��1������3������ͨ��,��Ҫ����
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_1); //��ӦPWM5
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_2); //��ӦPWM6
    //PWM�źŷ���
    GPIOPinConfigure(GPIO_PF1_M1PWM5);                  //PF1->PWMģ��1�ź�5
    GPIOPinConfigure(GPIO_PF2_M1PWM6);                  //PF2->PWMģ��1�ź�6
    //����PWM������
    //ģ��1,2->������3->���¼�������ͬ��
    PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    //����PWM����
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2,25000-1); //25000*0.8us=20ms
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_3,25000-1); //25000*0.8us=20ms
    //����PWMռ�ձ�
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_2)+1)*0); //��ʼռ�ձ�����Ϊ0
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_6,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)+1)*0);
    //ʹ��PWMģ��1ͨ��5,6���
    PWMOutputState(PWM1_BASE,PWM_OUT_5_BIT,true);
    PWMOutputState(PWM1_BASE,PWM_OUT_6_BIT,true);
    //ʹ��PWM������
    PWMGenEnable(PWM1_BASE,PWM_GEN_2);
    PWMGenEnable(PWM1_BASE,PWM_GEN_3);
}

void QEI_init()
{
    //PD6,PD7Ϊ���1�������ź�
    //PC5,PC6Ϊ���2�������ź�
    //ʹ��QEI��GPIOD,GPIOC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //����PD7����
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    //����GPIO
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_6);
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_7);
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_5);
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PD6_PHA0);//PD6->������A0������
    GPIOPinConfigure(GPIO_PD7_PHB0);//PD7->������B0������
    GPIOPinConfigure(GPIO_PC5_PHA1);//PC5->������A1������
    GPIOPinConfigure(GPIO_PC6_PHB1);//PC6->������B1������
    //����QEI0,QEI1ģ�飬A��B�����źű��ؾ������������������������帴λ��������λģʽ��A��B�����źŲ�������������ֵΪ0xffffffff
    QEIConfigure(QEI0_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP,0xffffffff);
    QEIConfigure(QEI1_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP,0xffffffff);
    //��ֹQEI0,QEI1�������ж�
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    //ʹ��QEI0,QEI1
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);
    //����QEI0,QEI1ģ�鵱ǰλ�ü�����ֵΪ0x7fffffff
    QEIPositionSet(QEI0_BASE,0x7fffffff);
    QEIPositionSet(QEI1_BASE,0x7fffffff);
}

void USART_init()
{
    //ʹ��PE4,PE5����ݮ�ɵĴ����շ�(USART5)
    //ʹ��PB0,PB1���û��Ĵ����շ�(USART1)
    //ʹ������
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    //��PE4,PE5�������óɴ�������
    GPIOPinConfigure(GPIO_PE4_U5RX);//RX=GPIO_PIN_4
    GPIOPinConfigure(GPIO_PE5_U5TX);//TX=GPIO_PIN_5
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    //��PB0,PB1�������óɴ�������
    GPIOPinConfigure(GPIO_PB0_U1RX);//RX=GPIO_PIN_0
    GPIOPinConfigure(GPIO_PB1_U1TX);//TX=GPIO_PIN_1
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //���ô��ڲ���ʱ��Դ������
    UARTClockSourceSet(UART5_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk( UART5_BASE, SysCtlClockGet(),115200,  UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    //UARTConfigSetExpClk( UART1_BASE, SysCtlClockGet(),115200,  UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    UARTStdioConfig(1, 115200, 16000000);//0-115200-80M,���ô���1Ϊ�ض���
    //ʹ��FIFO������FIFO���
    UARTFIFOEnable(UART5_BASE);
    UARTFIFODisable(UART5_BASE);
    UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8); //���շ��;�Ϊ2�ֽ�
    UARTFIFOEnable(UART1_BASE);
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    //���ô����ж�
    UARTIntRegister(UART5_BASE, UART5_Handler);
    UARTIntEnable(UART5_BASE, UART_INT_RX|UART_INT_RT); //�������ڽ����жϺͽ��ճ�ʱ�ж�,������FIFOʱһ���ַ��ж�һ��
    UARTIntClear(UART5_BASE, UART5_BASE);
    UARTIntRegister(UART1_BASE, UART1_Handler);
    UARTIntEnable(UART1_BASE, UART_INT_RX|UART_INT_RT); //�������ڽ����жϺͽ��ճ�ʱ�ж�
    UARTIntClear(UART1_BASE, UART1_BASE);
    //���ô����жϲ������������жϿ�����
    IntEnable(INT_UART5);
    IntEnable(INT_UART1);
    IntMasterEnable();
    //ʹ�ܴ���
    UARTEnable(UART5_BASE);
    UARTEnable(UART1_BASE);
}

void GPIO_init()
{
    //����PF3����Ϊ���1��ӦGPIO��
    //����PE0����Ϊ���2��ӦGPIO��
    //����PE1����Ϊ��ɫLED��ӦGPIO��
    //����PE2����Ϊ��ɫLED��ӦGPIO��
    //����PE3����ΪҩƷ�����ӦGPIO��
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);       //ʹ������
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_3); //�ú���Ĭ����������,��������
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
    //����PE3����Ϊ����,����ģʽ,�ڲ�����
    GPIODirModeSet(GPIO_PORTE_BASE,GPIO_PIN_3, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void Wheel_set(float pwm,int num) //pwm��-1��1
{
    uint32_t pwm_out = 0;
    uint32_t pwm_gen = 0;
    uint8_t dir = 0;
    switch(num)
    {
        case 1:
            pwm_out = PWM_OUT_5;
            pwm_gen = PWM_GEN_2;
            break;
        case 2:
            pwm_out = PWM_OUT_6;
            pwm_gen = PWM_GEN_3;
            break;
    }
    if(pwm > 0.99)
    {
        pwm = 0.99;
    }
    else if(pwm < -0.99)
    {
        pwm = -0.99;
    }

    if(pwm == 0)
    {
        PWMGenDisable(PWM1_BASE,pwm_gen); //pwmΪ0ʱ��������Ϊ0,Ӧ�ر�ͨ��
        switch(num) //��ӦGPIO��д��ʵ������ת
         {
                    case 1:
                        PWMOutputState(PWM1_BASE,PWM_OUT_5_BIT,false);
                        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0*GPIO_PIN_3);
                        break;
                    default:
                        PWMOutputState(PWM1_BASE,PWM_OUT_6_BIT,false);
                        GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,0*GPIO_PIN_0);
                        break;
                }
    }
    else
    {
        dir = pwm>0;
        pwm = (pwm*(1.5-num)<0)?(1-fabs(pwm)):pwm;

        switch(num) //��ӦGPIO��д��ʵ������ת
        {
            case 1:
                GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,(1-dir)*GPIO_PIN_3);
                PWMOutputState(PWM1_BASE,PWM_OUT_5_BIT,true);
                break;
            default:
                GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0,dir*GPIO_PIN_0);
                PWMOutputState(PWM1_BASE,PWM_OUT_6_BIT,true);
                break;
        }
        PWMPulseWidthSet(PWM1_BASE,pwm_out,(PWMGenPeriodGet(PWM1_BASE, pwm_gen)+1)*fabs(pwm));
        PWMGenEnable(PWM1_BASE,pwm_gen);
    }
}

void TIMER0_IRQHandler() //10msһ���ж�
{
    static int i = 0;
    static float x_speed1 = 0;
    static float x_speed2 = 0;
    static float pwm1 = 0;
    static float pwm2 = 0;
    static uint32_t x_next1 = 0x7fffffff;
    static uint32_t x_next2 = 0x7fffffff;
    static int x1 = 0;
    static int x2 = 0;

    //��ȡ�ж�״̬
    uint32_t status=TimerIntStatus(TIMER0_BASE, true);
    //����жϱ�־λ
    TimerIntClear(TIMER0_BASE,status);
        /*setspeed PID processing*/
        /*Delta_speed>0 means clockwise*/
        if(test_flag == 0) //����ģ������Э����
        {
            StatusDeal(0);
        }

        if(time_count > 0)time_count--;

        if(data_flag == 1 && pos_pid_flag==1) // 0.10s theta pid (Using "%" matters!)
        {
            pos_speed = PID_pos_update(pos);
            data_flag = 0;
        }

        if(i%4==0 && x_pid_flag==1) // 0.08s x pid (Using "%" matters!)
        {
            if(x_last_flag == 0) //����xpid
            {
                x_last1 = QEIPositionGet(QEI0_BASE);
                x_last2 = QEIPositionGet(QEI1_BASE);
                x_last_flag = 1;
            }
            else
            {
                x1 = (int)(QEIPositionGet(QEI0_BASE)-x_last1);
                x2 = (int)(x_last2-QEIPositionGet(QEI1_BASE));

                x_speed1 = PID_x_update(x_set1,x1,1); //�õ�ǰ������������λ������
                x_speed2 = PID_x_update(x_set2,x2,2);

                if(test_flag == 1)
                {
                }
            }
        }
        if(pid_reset_flag == 1) //���õ�ǰ������
        {
            QEIPositionSet(QEI0_BASE, 0x7fffffff);
            QEIPositionSet(QEI1_BASE, 0x7fffffff);
            x_next1 = 0x7fffffff;
            x_next2 = 0x7fffffff;
            pid_reset_flag = 0;
        }
        else
        {
            speed1 = (int)(QEIPositionGet(QEI0_BASE)-x_next1)*11; //���ǵ��������ת���,���м�ֵ0x7fffffff��ʼ����
            speed2 = (int)(x_next2-QEIPositionGet(QEI1_BASE))*11;
            x_next1 = QEIPositionGet(QEI0_BASE);
            x_next2 = QEIPositionGet(QEI1_BASE);

            pwm1 += PID_speed_update(setspeed*setspeed_flag+pos_speed*pos_pid_flag+x_pid_flag*x_speed1,speed1,1);
            pwm2 += PID_speed_update(setspeed*setspeed_flag-pos_speed*pos_pid_flag+x_pid_flag*x_speed2,speed2,2);
            Wheel_set(pwm1/K_round,1);
            Wheel_set(pwm2/K_round,2);

            /*f_char_printf(pos);
            UARTprintf(",");
            f_char_printf(pos_speed);
            UARTprintf("\n");*/
 
        }

        if(test_flag == 1)
        {
            //f_char_printf(pwm1);
            //UARTprintf(",");
            //f_char_printf(pwm2);
            //UARTprintf(",");
            f_char_printf(speed1);
            UARTprintf(",");
            f_char_printf(speed2);
            UARTprintf(","); 
            f_char_printf(x_speed1);
            UARTprintf(",");
            f_char_printf(x_speed2);
            UARTprintf(",");
            f_char_printf(x_set1);
            UARTprintf(",");
            f_char_printf(x_set2);
            UARTprintf(",");
            f_char_printf(setspeed);
            UARTprintf(",");
            f_char_printf(x1);
            UARTprintf(",");
            f_char_printf(x2);
            UARTprintf("\n");
        }

        if(i>100) // 0.5s reset i
        {
            /*return pulse count*/
            //printf("%d, %d\r\n",pwm1,pwm2);
            i=0;
        }
        i++;
}

void UART1_Handler() //�û�/˫��ͨ�Ŵ���
{
    //��ȡ�ж�״̬
    uint32_t status=UARTIntStatus(UART1_BASE,true);
    //����жϱ�־λ
    UARTIntClear(UART1_BASE, status);
    //���FIFO���Ƿ����ַ������У���ȡ����������
    while(UARTCharsAvail(UART1_BASE))
    {
        rx_buf1=UARTCharGetNonBlocking(UART1_BASE); //�Ӵ��ڽ���FIFO�ж�ȡһ���ַ�������������
        rDataCount1++;
        rData1[rDataCount1-1]=rx_buf1;
                    if(rx_buf1==0x20)// (ascii)0x14 = " "
                    {
                           if(test_flag == 1)
                           {
                               //UARTprintf("hello\r\n");
                               UARTCharPutNonBlocking(UART5_BASE, rData1[0]); //����Ŀ�����Ϣ
                           }
                           else if(strcmp((char*)rData1,"yes ") == 0)
                           {
                                route_flag = 1;
                           }
                           else if(strcmp((char*)rData1,"no ") == 0)
                           {
                                route_flag = 0;
                           }
                           else if(strcmp((char*)rData1,"reset ") == 0)
                           {
                                StatusReset();
                           }
                           /*else if(strcmp((char*)rData1,"Q1 ") == 0) //��߲��ֵ�1��
                           {
                                question = 1;
                           }
                           else if(strcmp((char*)rData1,"Q2 ") == 0) //��߲��ֵ�2��
                           {
                                question = 2;
                           }
                           else if(strcmp((char*)rData1,"ok ") == 0) //�ӳ��ͷ������ź�
                           {
                                StatusDeal(3);
                           }*/
                           else // para set
                           {
                               USART_PID_Adjust();//���ݽ����Ͳ�����ֵ����
                           }
                           memset(rData1,0,sizeof(rData1)); //��ջ�������
                           rDataCount1=0; //��ս��ճ���
                }
                    rx_buf1=0;
    }
}

void UART5_Handler() //��ݮ�ɴ���
{
    //��ȡ�ж�״̬
    uint32_t status=UARTIntStatus(UART5_BASE,true);
    //����жϱ�־λ
    UARTIntClear(UART5_BASE, status);
    //���FIFO���Ƿ����ַ������У���ȡ����������
    while(UARTCharsAvail(UART5_BASE))
    {
        rx_buf5=UARTCharGetNonBlocking(UART5_BASE); //�Ӵ��ڽ���FIFO�ж�ȡһ���ַ�������������
                rDataCount5++;
                rData5[rDataCount5-1]=rx_buf5;
            if(rx_buf5==0x20)// (ascii)0x14 = " "
            {
                if(rData5[0]>='1' && rData5[0]<='8' && rData5[1] == ' ') //��������,Ŀ�겡����Ϣ
                {
                    if(test_flag == 1)
                    {
                        UARTCharPutNonBlocking(UART5_BASE, 'r');
                    }
                    else
                    {
                        StatusDeal(2);
                    }
                }
                else if(((rData5[0]>='a' && rData5[0]<='z')||rData5[0]=='S') && rData5[1]==' ') //ѭ��ֹͣ����ʶ����Ϣ
                {
                    if(test_flag == 1)
                    {
                        UARTCharPutNonBlocking(UART5_BASE, 'r');
                    }
                    else
                    {
                        UARTprintf("message=%c\r\n",rData5[0]);
                        StatusDeal(1); //����Э��
                    }
                }
                else if(status_hand == 0 || status_hand == 7 || test_flag == 1)   //ѭ����Ϣ,"theta?b "
                {

                    pos = strtod((char*)rData5,&endptr);

                    endptr = (char*)rData5;
                    data_flag = 1;
                    keep = 0;
                    if(status_hand == 7 && uart_flag > 0)uart_flag --;
                }
                memset(rData5,0,sizeof(rData5)); //��ջ�������
                rDataCount5=0; //��ս��ճ���
            }
            rx_buf5=0;
    }
}

void f_char_printf(float Xangle)
{
    float temp = Xangle;

    if(Xangle>=0)
    {
        UARTprintf("%d.%d",(int32_t)temp ,(int32_t)((temp -(int32_t)temp )*1000));
    }
    else
    {
        temp = -temp;
        UARTprintf("-%d.%d",(int32_t)temp ,(int32_t)((temp -(int32_t)temp )*1000));
    }
}

uint8_t Drug_Read(void) //ҩƷ���
{
    if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)!=GPIO_PIN_3) //PF3������,����˵����ҩƷ����
    {
        return 1;
    }
    return 0;
}
