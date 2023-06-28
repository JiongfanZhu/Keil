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

#define Wheel_1 GPIO_PIN_3
#define Wheel_2 GPIO_PIN_0
#define LED_red GPIO_PIN_1
#define LED_green GPIO_PIN_2
#define Drug GPIO_PIN_3
#define round_pulse 390 //������ÿȦ������
#define K_round 300 //pwm�任ϵ��

int x_pid_flag = 0; //��ʼ�ر�xpid
int theta_pid_flag = 1;
int b_pid_flag = 1;
int setspeed_flag = 1;

float b = 200; //��pid��ʼ����Ԥ��bһ��
float setspeed = 160;
float x_set1 = 0;
float x_set2 = 0;
float theta = 0;

float theta_speed = 0;
float b_speed = 0;
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
uint8_t uart_flag = 0;      //Ѳ����Ϣ��ʶλ
uint8_t LED_flag = 0;       //0����,1�����,2���̵�

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

    Wheel_set(0,1);
    Wheel_set(0,2);
    //PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_2)+1)*fabs(0.2));
    //PWMPulseWidthSet(PWM1_BASE,PWM_OUT_6,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)+1)*fabs(0.7));

    while(1)
    {
        /*ҩƷ���*/
        if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)!=GPIO_PIN_3) //PF3������,����˵����ҩƷ����
        {
            route_flag = 1;
        }
        else
        {
            route_flag = 0;
        }

        if(test_flag == 1) //���Գ���
        {
            //UARTprintf("test\r\n"); //�û����ڲ���
            //UARTCharPutNonBlocking(UART5_BASE, 'h'); //��ݮ�ɴ��ڲ���
            //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2); //��Ʋ���
            //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4); //�̵Ʋ���

            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2, 4); //�̵Ʋ���
            if(!(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)==GPIO_PIN_3))
            {
                SysCtlDelay(SysCtlClockGet()*0.05/3);//��ʱ����
                if(!(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)==GPIO_PIN_3))
                {
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2, 2); //��Ʋ���
                }
                //while(!(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)==GPIO_PIN_3)); //�������ͷ�
                //SysCtlDelay(SysCtlClockGet()*0.05/3);//��ʱ����
            }
            //SysCtlDelay(SysCtlClockGet()/3);
        }
        else
        {
            switch(LED_flag)
            {
                case 0: //Ϩ��
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
                    break;
                case 1: //�����
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
                    break;
                case 2: //�̵���
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4);
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
    uint8_t pwm_out = 0;
    uint8_t pwm_gen = 0;
    uint8_t dir = 0;
    switch(num)
    {
        case 1:
            pwm_out = PWM_OUT_5;
            pwm_gen = PWM_GEN_2;
            break;
        default:
            pwm_out = PWM_OUT_6;
            pwm_gen = PWM_GEN_3;
    }
    if(pwm > 1)
    {
        pwm = 1;
    }
    else if(pwm < -1)
    {
        pwm = -1;
    }

    if(pwm == 0)
    {
        PWMGenDisable(PWM1_BASE,pwm_gen); //pwmΪ0ʱ��������Ϊ0,Ӧ�ر�ͨ��
    }
    else
    {
        if(pwm>0)dir = 1; // forward


        switch(num) //��ӦGPIO��д��ʵ������ת
        {
            case 1:
                GPIOPinWrite(GPIO_PORTF_BASE,Wheel_1,dir*8);
                break;
            default:
                GPIOPinWrite(GPIO_PORTF_BASE,Wheel_2,1-dir);
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

    //��ȡ�ж�״̬
    uint32_t status=TimerIntStatus(TIMER0_BASE, true);
    //����жϱ�־λ
    TimerIntClear(TIMER0_BASE,status);
        /*setspeed PID processing*/
        /*Delta_speed>0 means clockwise*/
        //if(uart_flag == 1 && test_flag == 0) //�д������ݸ����Ҳ����ڲ���״̬
        if(test_flag == 0) //�����ڲ���״̬->��֤10ms�Ķ�ʱ��������Э��
        {
            if(status_hand != 0) //����ģ������Э����
            {
                StatusDeal(0);
            }

            if(i%10==0) // 0.10s theta pid (Using "%" matters!)
            {
                if(theta_pid_flag==1)
                {
                    theta_speed = PID_theta_update(theta);
                }

                if(b_pid_flag==1)
                {
                    b_speed = PID_b_update(b);
                }
            }

            if(i%8==0 && x_pid_flag==1) // 0.08s x pid (Using "%" matters!)
            {
                x_speed1 = PID_x_update(x_set1,(QEIPositionGet(QEI0_BASE)-0x7fffffff),1); //�õ�ǰ������������λ������
                x_speed2 = PID_x_update(x_set2,(QEIPositionGet(QEI1_BASE)-0x7fffffff),2);
            }

            if(i%2==0) // 0.02s speed pid
            {
                /*present speed*/
                speed1 = (QEIPositionGet(QEI0_BASE)-0x7fffffff)*60.0/(0.02*round_pulse); //���ǵ��������ת���,���м�ֵ0x7fffffff��ʼ����
                speed2 = (QEIPositionGet(QEI1_BASE)-0x7fffffff)*60.0/(0.02*round_pulse);
                if(x_pid_flag == 0) //��λ�Ʊջ�����ʱ,���Ա�������λ,��speed1,speed2��Ϊ����λ��ʹ��
                {
                    QEIPositionSet(QEI0_BASE, 0x7fffffff);
                    QEIPositionSet(QEI1_BASE, 0x7fffffff);
                }
                pwm1 = PID_speed_update(setspeed*setspeed_flag+theta_pid_flag*theta_speed-b_pid_flag*b_speed+x_pid_flag*x_speed1,speed1,pwm1,1);
                pwm2 = PID_speed_update(setspeed*setspeed_flag-theta_pid_flag*theta_speed+b_pid_flag*b_speed+x_pid_flag*x_speed2,speed2,pwm2,2);
                Wheel_set(pwm1/K_round,1);
                Wheel_set(pwm2/K_round,2);
            }

            if(i>100) // 0.5s reset i
            {
                /*return pulse count*/
                //printf("%d, %d\r\n",pwm1,pwm2);
                i=0;
            }
            i++;
            //uart_flag = 0;
        }
}

void UART1_Handler() //�û�����
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
                        //printf("%s",rData);
                        //UARTprintf("RXLen=%d\r\n",rDataCount0);
                        //for(int i=0;i<rDataCount0;i++) UARTprintf("UART rData0[%d] =%c\r\n",i,rData0[i]);
                           if(test_flag == 1)
                           {
                               UARTprintf("hello\r\n");
                               //UARTCharPutNonBlocking(UART5_BASE, rData1[0]); //����Ŀ�����Ϣ
                               //StatusDeal(2); //����С������
                           }
                           else if(strcmp((char*)rData1,"reset ") == 0) //��λ
                           {
                               StatusReset();
                           }
                           //else if(rData1[0]>='0' && rData1[0]<='9')    //Ŀ���ѡ��
                           //{
                           //   UARTCharPutNonBlocking(UART5_BASE, rData1[0]); //����Ŀ�����Ϣ
                           //    StatusDeal(2); //����С������
                           //}
                           else // para set
                           {
                               USART_PID_Adjust();//���ݽ����Ͳ�����ֵ����
                           }
                           memset(rData1,0,sizeof(rData1)); //��ջ�������
                           rDataCount1=0; //��ս��ճ���
                }
                    rx_buf1=0;
        //UARTCharPutNonBlocking(UART0_BASE, rxbuf);
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
                if(test_flag == 1)  //����ģʽ
                {
                    UARTprintf("berrypie\r\n");
                }
                else if(status_hand == 0)   //ѭ����Ϣ,"theta?b "
                {
                    theta = strtof(rData5,&endptr);
                    endptr++;
                    b = strtod(endptr,NULL);
                    endptr = rData5;
                    uart_flag = 1;
                }
                else if(strcmp((char*)rData5,"s ") == 0|| strcmp((char*)rData5,"X ") == 0 || status_hand != 0) //�������ģ������Э����
                {
                    StatusDeal(1);
                }

                memset(rData5,0,sizeof(rData5)); //��ջ�������
                rDataCount5=0; //��ս��ճ���
            }
            rx_buf5=0;
        //UARTCharPutNonBlocking(UART1_BASE, rxbuf);
    }
}


