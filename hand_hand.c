/*
 * hand_hand.c
 *
 *  Created on: 2023��6��6��
 *      Author: LENOVO
 */

#include "hand_hand.h"

#define TURN_X 2000
#define L_ROUND_X 1250
#define R_ROUND_X 1250
#define STRAIGHT_X1 1250
#define STRAIGHT_X2 1300
#define STOP 15
#define SPEED_B 50
#define UART_CNT 10

uint8_t status_hand = 4;    //��Ƭ����ǰ״̬
uint8_t route[10] = {0};    //С��·�ھ��߼�¼(0ֱ��,1��ת,2��ת)
int route_len = 0;      //С��·�ھ�������
extern uint8_t route_flag;     //ҩƷ״̬��־λ
extern int uart_flag;

uint8_t recognize_flag = 0; //ʶ�������ʶ
uint8_t turn_route_flag = 0; //�ջ�ָʾ(0ֱ��,1��ת,2��ת,3��ͷ)
uint8_t x_task_flag = 0; //ֱ�߱ջ�����ָʾ
uint8_t turn_task_flag = 0; //ת��ջ�����ָʾ
uint8_t task_flag = 0; //�ջ����ָʾ
uint8_t target = 0; //Ŀ����ʶ
uint8_t next_q = 0; //���ӳ���Ϣ��������һ״̬
int stop_count = 1; //���뱻�ӳ�������·����
uint8_t turn_flag = 0; //�Ƿ���ɵ�һ��ת��
uint8_t cross_flag = 0; //�Ƿ�Ϊ��һ��·��
int stop_judge = STOP; //ֹͣ�ж�����

void StatusReset(void)
{
    //status_hand�ļ���״̬
    //0:��������״̬/ָ������״̬
    //1:����ֹͣ״̬
    //2:ֹͣ״̬/��ݮ�ɵȴ�״̬
    //3:�ջ�����״̬
    //4:�û�ָ��ȴ�״̬/��ʼ״̬
    //5:�ӳ�����״̬
    status_hand = 4;
    //PID_reset();
    route_len = 0; //��վ�����
    memset(route,0,sizeof(route)); //��վ��߼�¼
    route_flag = 0; //��ҩƷ����
    recognize_flag = 0; //��ʶ�����󷢳�
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //Ϩ��
    target = 0;
    next_q = 0;
    stop_count = 1;
    turn_flag = 0;
    cross_flag = 0;
    uart_flag = UART_CNT;
    stop_judge = STOP;
    UARTCharPutNonBlocking(UART5_BASE, 'R'); //����ݮ�ɷ��͸�λ��Ϣ
}

/*
��ݮ�ɷ���ָ�:
    "s ":С��ֹͣ;
    "l ":С����ת;
    "r ":С����ת;
    "S ":С��ֱ��;
    "b ":С����ͷ(����ҩ��);
    "X ":��ݮ���Ѽ�¼Ŀ���(X��1~8������);

��ݮ�ɽ���ָ�:
    'd':��ݮ�ɽ���ʶ��;
    'r':��ݮ�ɽ���Ѳ��;
    'R':��ݮ�ɸ�λ;

message����:
    0:��ʱ�жϽ���;
    1:��ݮ�ɴ�����Ϣ;
    2:Ŀ�겡����Ϣ;
    3:�ӳ��ͷ������ź�;
    
    4:Ѳ�߲���;


˫��ͨ��ָ�:
    "X ":ĸ�����жҩ,�ӳ�ȡҩָ��;
    "1 ":���ӳ�����ĸ��������Ϣ;
    "Q1 ":��ĸ�����Ͷ�Ӧ��Ŀ��Ϣ;
    "ok ":��ĸ��/�ӳ��������ͷ��ź�;

PS:��ݮ����С�������źź�,���ǻص�ָ��ȴ�״̬
   ����ʼ��ʱ��ݮ�ɵ�״̬����Ҫ��ȶ(?)
*/

void StatusDeal(uint8_t message) //message=0��ʾ�޴�����Ϣ,�����д�����Ϣ
{
    switch(status_hand)
    {
        case 0:     //��������״̬
        UARTprintf("case 0\r\n");
            if(message == 1 && strcmp((char*)rData5,"s ") == 0) //ʶ��ֹͣ��ʶ,��ݮ�ɷ���ָֹͣ����Զ�����
            {
                //UARTprintf("stop\r\n");
                status_hand = 1; //�޸�Ϊ����ֹͣ״̬1
                //status_hand = 6;
                /*�ر�Ѳ�����pid*/
                //PID_reset();
                b_pid_flag = 0;
                setspeed_flag = 0;
                theta_pid_flag = 0;
                Wheel_set(0,1);
                Wheel_set(0,2);
            }
            break;
        case 1:     //����ֹͣ״̬,��Ҫ����Ƿ��Ѿ�ͣ��,ֹͣ��ʹ�ñջ�
            UARTprintf("case 1\r\n");
            if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B) //��ֹͣ
            {
                stop_judge--;
            }
            else
            {
                stop_judge = STOP;
            }
            if(stop_judge <=0)
            {
                stop_judge = STOP;
                //PID_reset();
                status_hand = 2; //�޸�Ϊֹͣ״̬2
                recognize_flag = 0; //ʶ������λ
                if(route_flag == 1 && cross_flag == 1) //��ҩƷ,����ҩ����,�Ҳ�Ϊ��һ��ֹͣ
                {
                    recognize_flag = 1; //ʶ�������ͱ�ʶ��λ
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //����ʶ������
                    UARTprintf("d send\r\n");
                }
                else if(cross_flag == 0) //��û��ֹͣ��
                {
                    //UARTprintf("first turn\r\n");
                    switch(target)
                    {
                        case 1:
                            turn_route_flag = 1; //Ŀ�겡��Ϊ1,Ӧ�����
                            break;
                        case 2:
                            turn_route_flag = 2; //Ŀ�겡��Ϊ2,Ӧ���Ҳ�
                            break;
                        default:
                            turn_route_flag = 0; //���ǽ���ҩ��,ֱ��
                    }
                    status_hand = 3; //ֱ�ӽ���ջ�
                    cross_flag = 1; //��ʶλ��λ
                }
                else if(question==2 || question==1) //������߽�������
                {
                    stop_count--; //ͬʱҲ���ڶ��ӳ��ͷ�����
                    if(stop_count==0 && question==2) //���ڵڶ��ʶ�ĸ������
                    {
                        next_q = status_hand;
                        status_hand = 5; //����
                    }
                }
            }
            else
            {
                //UARTprintf("not stop yet\r\n");
            }
            break;
        case 2:     //ֹͣ״̬/��ݮ�ɵȴ�״̬
            UARTprintf("case 2\r\n");
            if(message != 1 && recognize_flag == 1)
            {
                UARTCharPutNonBlocking(UART5_BASE, 'd'); //����ʶ������
                UARTprintf("d send again\r\n");
            }
            if(message == 1 && recognize_flag == 1) //�д�����Ϣ�ҷ�����ʶ������->��ҩ����
            {
                if(strcmp((char*)rData5,"l ")==0) //��ת
                {
                    if(turn_flag == 0) //��һ��ת��
                    {
                        UARTprintf("l ");
                        turn_flag = 1;
                    }
                    route[route_len] = 1;
                    route_len++;
                    turn_route_flag = 1;
                }
                else if(strcmp((char*)rData5,"r ")==0) //��ת
                {
                    if(turn_flag == 0) //��һ��ת��
                    {
                        UARTprintf("r ");
                        turn_flag = 1;
                    }
                    route[route_len] = 2;
                    route_len++;
                    turn_route_flag = 2;
                }
                else if(strcmp((char*)rData5,"S ")==0) //ֱ��
                {
                    route[route_len] = 0;
                    route_len++;
                    turn_route_flag = 0;
                }
                else if(strcmp((char*)rData5,"b ")==0) //��ͷ,����Ŀ��ҩ��
                {
                    route_len--; //ɾȥһ������,��ǰroute_len��Ӧ���һ�������Ϣ,׼����ջ
                    turn_route_flag = 3;
                    LED_flag = 1; //�����(��������)
                }
                status_hand = 3; //�޸�Ϊ�ջ�����״̬3
            }
            else if(route_len == -2) //�������м�¼·�ڲ�ֹͣ->�������
            {
                status_hand = 4; //�ȴ�,Ҫ��С������ҩ��
                LED_flag = 2; //�̵���
            }
            else if(message == 0 && recognize_flag == 0) //�޴�����Ϣ��δ����ʶ������->�ؼҹ�����
            {
                switch(route[route_len]) //��ȡ��·��ת����Ϣ
                {
                    case 0: //ֱ��
                        turn_route_flag = 0;
                        break;
                    case 1: //��ת,����ʱ��ת
                        turn_route_flag = 2;
                        break;
                    case 2: //��ת,����ʱ��ת
                        turn_route_flag = 1;
                        break;
                }
                route_len--; //ɾȥһ��·��
                status_hand = 3; //�޸�Ϊ�ջ�����״̬3
            }
            x_task_flag = 0; //���ñջ��йر�ʶ
            turn_task_flag = 0;
            task_flag = 0;
            break;
        case 3:     //�ջ�����״̬
        UARTprintf("case 3\r\n");
            if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B) //ֹͣ˵���������һ�ջ�������״̬2ת����3
            {
                stop_judge--;
            }
            else
            {
                stop_judge = STOP;
            }
            if(stop_judge <= 0)
            {
                stop_judge = STOP;
                //PID_reset();
                //UARTprintf("case 3\r\n");
                switch(turn_route_flag) //��ȡ�ջ�ָʾ
                {
                    case 0: //ֱ��
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            //UARTprintf("step1\r\n");

                            //PID_reset();
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else
                        {
                            //PID_reset();
                            //UARTprintf("step2\r\n");
                            task_flag = 1;
                        }
                        break;
                    case 1: //��ת
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            //PID_reset();
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            //PID_reset();
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = L_ROUND_X;
                            x_set2 = -L_ROUND_X;
                            turn_task_flag = 1;
                        }
                        else //�ջ����
                        {
                            task_flag = 1;
                        }
                        break;
                    case 2: //��ת
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = -R_ROUND_X;
                            x_set2 = R_ROUND_X;
                            turn_task_flag = 1;
                        }
                        else //�ջ����
                        {
                            task_flag = 1;
                        }
                        break;
                    case 3: //��ͷ,���ڵ���ҩ��ʱִ����һ��֧
                        if(turn_task_flag == 0 && route_flag == 0) //δ����ת��ջ�,��ҩƷ��ж��
                        {
                            if(question != 0)
                            {
                                UARTprintf("X "); //�ӳ�ȡҩָ��
                            }
                            LED_flag = 0; //Ϩ��
                            x_last_flag = 0;
                            x_pid_flag = 1;

                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1 && route_flag == 0) //ת��ջ����,��ҩƷ��ж��
                        {
                            task_flag = 1; //�ջ����
                        }
                        break;
                }
                if(task_flag == 1) //��ɱջ�����
                {
                    //UARTprintf("task done\r\n");
                    UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
                    if((stop_count==-1 && question==2)||(stop_count==0 && question==1))UARTprintf("ok "); //�ڱ�������ĵ�һ��·����ɶ�����,�ͷ��ӳ������ź�
                    status_hand = 7; //�޸�Ϊ��������״̬
                    /*������Ҫ��pid*/
                    //PID_reset();
                    x_pid_flag = 0;
                    uart_flag = UART_CNT;

                    //b_pid_flag = 1;
                    //theta_pid_flag = 1;
                    //setspeed_flag = 1;
                }
            }
            break;

        case 4:     //ָ��ȴ�״̬/��ʼ״̬
        UARTprintf("case 4\r\n");
            if(message == 2) //��ݮ�����ʶ��
            {
                target = rData5[0]-'0';
                UARTprintf("%d ",target); //����ĸ��������Ϣ
            }
            if(target != 0 && route_flag == 1) //ҩƷ���װ������ݮ�������ʶ��
            {
                status_hand = 7; //�޸�Ϊ��������״̬
                /*����Ѳ�߶�Ӧpid*/
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
                UARTprintf("start\r\n");
                uart_flag = UART_CNT;
            }
            break;
        case 5:     //�ӳ�����״̬
            if(message == 3) //�����źű��ͷ�
            {
                status_hand = next_q;
                next_q = 0;
            }
            break;
        case 6:     //����״̬
            //PID_reset();
            if(message == 4)
            {
                status_hand = 7;
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
                uart_flag = UART_CNT;
            }
            break;
        case 7:     //ѭ������״̬
            UARTprintf("case 7\r\n");
            if(message == 1)
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
                uart_flag = UART_CNT;
            }
            if(uart_flag <= 0)
            {
                status_hand = 0;
                x_pid_flag = 0;
                //PID_reset();
                setspeed_flag = 1;
                b_pid_flag = 1;
                theta_pid_flag = 1;
            }
            else
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
            }
            break;
    }
}
