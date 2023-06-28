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
#define round_pulse 390 //编码盘每圈脉冲数
#define K_round 300 //pwm变换系数

int x_pid_flag = 0; //初始关闭xpid
int theta_pid_flag = 1;
int b_pid_flag = 1;
int setspeed_flag = 1;

float b = 200; //与pid初始化中预设b一致
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
uint8_t route_flag = 0;     //药品状态标志位
uint8_t test_flag = 1;      //测试模式
uint8_t uart_flag = 0;      //巡线信息标识位
uint8_t LED_flag = 0;       //0不亮,1亮红灯,2亮绿灯

char * endptr;

int main(void)
{
    //配置系统时钟，使用外部晶振驱动PLL，分频系数2.5，系统时钟80Mhz
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
        /*药品检测*/
        if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)!=GPIO_PIN_3) //PF3有上拉,不等说明有药品放置
        {
            route_flag = 1;
        }
        else
        {
            route_flag = 0;
        }

        if(test_flag == 1) //测试程序
        {
            //UARTprintf("test\r\n"); //用户串口测试
            //UARTCharPutNonBlocking(UART5_BASE, 'h'); //树莓派串口测试
            //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2); //红灯测试
            //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4); //绿灯测试

            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2, 4); //绿灯测试
            if(!(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)==GPIO_PIN_3))
            {
                SysCtlDelay(SysCtlClockGet()*0.05/3);//延时消抖
                if(!(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)==GPIO_PIN_3))
                {
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2, 2); //红灯测试
                }
                //while(!(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3)==GPIO_PIN_3)); //待按键释放
                //SysCtlDelay(SysCtlClockGet()*0.05/3);//延时消抖
            }
            //SysCtlDelay(SysCtlClockGet()/3);
        }
        else
        {
            switch(LED_flag)
            {
                case 0: //熄灭
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
                    break;
                case 1: //红灯亮
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 2);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
                    break;
                case 2: //绿灯亮
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
                    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 4);
            }
        }
    }

}

void Sysint_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);       //使能外设
    //配置定时器，将Timer0拆分并配置TIMERA为连续向下计数模式
    TimerConfigure(TIMER0_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    //设置预分频值
    TimerPrescaleSet(TIMER0_BASE, TIMER_A,199); //80MHz->400kHz
    //设置装载值
    TimerLoadSet(TIMER0_BASE, TIMER_A,3999); //400kHz->100Hz,满足10ms一次控制调整需求
    //注册中断服务函数
    TimerIntRegister(TIMER0_BASE,TIMER_A, TIMER0_IRQHandler);
    //开启定时器A超时中断
    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
    //设置中断优先级
    IntPrioritySet(INT_TIMER0A, 0);
    //使能中断
    IntEnable(INT_TIMER0A);
    IntMasterEnable();
    //使能定时器
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void PWM_init()
{
    //PF1对应控制模块1PWM5,由发生器2控制
    //PF2对应控制模块1PWM6,由发生器3控制
    //PWM设置部分
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);//PWM时钟80M/64,从系统时钟分频得到,PWM时钟周期为0.8us
    //使能时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);         //使能PWM模块1时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);        //使能GPIOF时钟
    //使能引脚复用PWM功能
    //PF1,PF1为模块1发生器3的两个通道,都要开启
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_1); //对应PWM5
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_2); //对应PWM6
    //PWM信号分配
    GPIOPinConfigure(GPIO_PF1_M1PWM5);                  //PF1->PWM模块1信号5
    GPIOPinConfigure(GPIO_PF2_M1PWM6);                  //PF2->PWM模块1信号6
    //配置PWM发生器
    //模块1,2->发生器3->向下计数，不同步
    PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE,PWM_GEN_3,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    //配置PWM周期
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2,25000-1); //25000*0.8us=20ms
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_3,25000-1); //25000*0.8us=20ms
    //配置PWM占空比
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_5,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_2)+1)*0); //初始占空比设置为0
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_6,(PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)+1)*0);
    //使能PWM模块1通道5,6输出
    PWMOutputState(PWM1_BASE,PWM_OUT_5_BIT,true);
    PWMOutputState(PWM1_BASE,PWM_OUT_6_BIT,true);
    //使能PWM发生器
    PWMGenEnable(PWM1_BASE,PWM_GEN_2);
    PWMGenEnable(PWM1_BASE,PWM_GEN_3);
}

void QEI_init()
{
    //PD6,PD7为电机1编码器信号
    //PC5,PC6为电机2编码器信号
    //使能QEI与GPIOD,GPIOC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //解锁PD7引脚
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    //复用GPIO
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_6);
    GPIOPinTypeQEI(GPIO_PORTD_BASE,GPIO_PIN_7);
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_5);
    GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PD6_PHA0);//PD6->编码器A0相输入
    GPIOPinConfigure(GPIO_PD7_PHB0);//PD7->编码器B0相输入
    GPIOPinConfigure(GPIO_PC5_PHA1);//PC5->编码器A1相输入
    GPIOPinConfigure(GPIO_PC6_PHB1);//PC6->编码器B1相输入
    //配置QEI0,QEI1模块，A、B两相信号边沿均产生计数，不开启索引脉冲复位，正交相位模式，A、B两相信号不交换，最大计数值为0xffffffff
    QEIConfigure(QEI0_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP,0xffffffff);
    QEIConfigure(QEI1_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP,0xffffffff);
    //禁止QEI0,QEI1的所有中断
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    //使能QEI0,QEI1
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);
    //设置QEI0,QEI1模块当前位置计数器值为0x7fffffff
    QEIPositionSet(QEI0_BASE,0x7fffffff);
    QEIPositionSet(QEI1_BASE,0x7fffffff);
}

void USART_init()
{
    //使用PE4,PE5作树莓派的串口收发(USART5)
    //使用PB0,PB1作用户的串口收发(USART1)
    //使能外设
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    //将PE4,PE5引脚配置成串口引脚
    GPIOPinConfigure(GPIO_PE4_U5RX);//RX=GPIO_PIN_4
    GPIOPinConfigure(GPIO_PE5_U5TX);//TX=GPIO_PIN_5
    GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    //将PB0,PB1引脚配置成串口引脚
    GPIOPinConfigure(GPIO_PB0_U1RX);//RX=GPIO_PIN_0
    GPIOPinConfigure(GPIO_PB1_U1TX);//TX=GPIO_PIN_1
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //配置串口波特时钟源及参数
    UARTClockSourceSet(UART5_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk( UART5_BASE, SysCtlClockGet(),115200,  UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
    //UARTConfigSetExpClk( UART1_BASE, SysCtlClockGet(),115200,  UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
    UARTStdioConfig(1, 115200, 16000000);//0-115200-80M,设置串口1为重定向
    //使能FIFO并设置FIFO深度
    UARTFIFOEnable(UART5_BASE);
    UARTFIFODisable(UART5_BASE);
    UARTFIFOLevelSet(UART5_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8); //接收发送均为2字节
    UARTFIFOEnable(UART1_BASE);
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    //配置串口中断
    UARTIntRegister(UART5_BASE, UART5_Handler);
    UARTIntEnable(UART5_BASE, UART_INT_RX|UART_INT_RT); //开启串口接收中断和接收超时中断,不开启FIFO时一个字符中断一次
    UARTIntClear(UART5_BASE, UART5_BASE);
    UARTIntRegister(UART1_BASE, UART1_Handler);
    UARTIntEnable(UART1_BASE, UART_INT_RX|UART_INT_RT); //开启串口接收中断和接收超时中断
    UARTIntClear(UART1_BASE, UART1_BASE);
    //启用串口中断并开启处理器中断控制器
    IntEnable(INT_UART5);
    IntEnable(INT_UART1);
    IntMasterEnable();
    //使能串口
    UARTEnable(UART5_BASE);
    UARTEnable(UART1_BASE);
}

void GPIO_init()
{
    //配置PF3引脚为电机1对应GPIO口
    //配置PE0引脚为电机2对应GPIO口
    //配置PE1引脚为红色LED对应GPIO口
    //配置PE2引脚为绿色LED对应GPIO口
    //配置PE3引脚为药品输入对应GPIO口
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);       //使能外设
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_3); //该函数默认引脚推挽,无上下拉
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
    //配置PE3引脚为输入,推挽模式,内部上拉
    GPIODirModeSet(GPIO_PORTE_BASE,GPIO_PIN_3, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void Wheel_set(float pwm,int num) //pwm从-1到1
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
        PWMGenDisable(PWM1_BASE,pwm_gen); //pwm为0时不能设置为0,应关闭通道
    }
    else
    {
        if(pwm>0)dir = 1; // forward


        switch(num) //对应GPIO口写入实现正反转
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

void TIMER0_IRQHandler() //10ms一次中断
{
    static int i = 0;
    static float x_speed1 = 0;
    static float x_speed2 = 0;
    static float pwm1 = 0;
    static float pwm2 = 0;

    //获取中断状态
    uint32_t status=TimerIntStatus(TIMER0_BASE, true);
    //清除中断标志位
    TimerIntClear(TIMER0_BASE,status);
        /*setspeed PID processing*/
        /*Delta_speed>0 means clockwise*/
        //if(uart_flag == 1 && test_flag == 0) //有串口数据更新且不处于测试状态
        if(test_flag == 0) //不处于测试状态->保证10ms的定时触发握手协议
        {
            if(status_hand != 0) //处于模拟握手协议中
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
                x_speed1 = PID_x_update(x_set1,(QEIPositionGet(QEI0_BASE)-0x7fffffff),1); //用当前编码盘数据作位移数据
                x_speed2 = PID_x_update(x_set2,(QEIPositionGet(QEI1_BASE)-0x7fffffff),2);
            }

            if(i%2==0) // 0.02s speed pid
            {
                /*present speed*/
                speed1 = (QEIPositionGet(QEI0_BASE)-0x7fffffff)*60.0/(0.02*round_pulse); //考虑到电机正反转情况,从中间值0x7fffffff开始计数
                speed2 = (QEIPositionGet(QEI1_BASE)-0x7fffffff)*60.0/(0.02*round_pulse);
                if(x_pid_flag == 0) //当位移闭环开启时,不对编码器置位,将speed1,speed2作为两轮位移使用
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

void UART1_Handler() //用户串口
{
    //读取中断状态
    uint32_t status=UARTIntStatus(UART1_BASE,true);
    //清除中断标志位
    UARTIntClear(UART1_BASE, status);
    //检查FIFO中是否有字符，若有，读取出来并发送
    while(UARTCharsAvail(UART1_BASE))
    {
        rx_buf1=UARTCharGetNonBlocking(UART1_BASE); //从串口接收FIFO中读取一个字符，非阻塞函数
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
                               //UARTCharPutNonBlocking(UART5_BASE, rData1[0]); //发送目标点信息
                               //StatusDeal(2); //开启小车任务
                           }
                           else if(strcmp((char*)rData1,"reset ") == 0) //复位
                           {
                               StatusReset();
                           }
                           //else if(rData1[0]>='0' && rData1[0]<='9')    //目标点选择
                           //{
                           //   UARTCharPutNonBlocking(UART5_BASE, rData1[0]); //发送目标点信息
                           //    StatusDeal(2); //开启小车任务
                           //}
                           else // para set
                           {
                               USART_PID_Adjust();//数据解析和参数赋值函数
                           }
                           memset(rData1,0,sizeof(rData1)); //清空缓存数组
                           rDataCount1=0; //清空接收长度
                }
                    rx_buf1=0;
        //UARTCharPutNonBlocking(UART0_BASE, rxbuf);
    }
}

void UART5_Handler() //树莓派串口
{
    //读取中断状态
    uint32_t status=UARTIntStatus(UART5_BASE,true);
    //清除中断标志位
    UARTIntClear(UART5_BASE, status);
    //检查FIFO中是否有字符，若有，读取出来并发送
    while(UARTCharsAvail(UART5_BASE))
    {
        rx_buf5=UARTCharGetNonBlocking(UART5_BASE); //从串口接收FIFO中读取一个字符，非阻塞函数
                rDataCount5++;
                rData5[rDataCount5-1]=rx_buf5;
            if(rx_buf5==0x20)// (ascii)0x14 = " "
            {
                if(test_flag == 1)  //测试模式
                {
                    UARTprintf("berrypie\r\n");
                }
                else if(status_hand == 0)   //循迹信息,"theta?b "
                {
                    theta = strtof(rData5,&endptr);
                    endptr++;
                    b = strtod(endptr,NULL);
                    endptr = rData5;
                    uart_flag = 1;
                }
                else if(strcmp((char*)rData5,"s ") == 0|| strcmp((char*)rData5,"X ") == 0 || status_hand != 0) //进入或处于模拟握手协议中
                {
                    StatusDeal(1);
                }

                memset(rData5,0,sizeof(rData5)); //清空缓存数组
                rDataCount5=0; //清空接收长度
            }
            rx_buf5=0;
        //UARTCharPutNonBlocking(UART1_BASE, rxbuf);
    }
}


