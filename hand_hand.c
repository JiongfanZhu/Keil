/*
 * hand_hand.c
 *
 *  Created on: 2023��6��6��
 *      Author: LENOVO
 */

#include "hand_hand.h"

#define TURN_X 2000
#define L_ROUND_X 950
#define R_ROUND_X 1050
#define STRAIGHT_X1 1800
#define STRAIGHT_X2 1800
#define STOP 10
#define SPEED_B 50
#define UART_CNT 10
#define TIME 50
#define TIME_STOP 50

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
uint8_t cross = 0; //·����
uint8_t cross_flag = 0; //�Ƿ񾭹�����·��
int stop_judge = STOP; //ֹͣ�ж�����
uint8_t home_flag = 0;

void StatusReset(void)
{
    //status_hand�ļ���״̬
    //0:��������״̬/ָ������״̬
    //1:����ֹͣ״̬
    //2:ֹͣ״̬/��ݮ�ɵȴ�״̬
    //3:�ջ�����״̬
    //4:�û�ָ��ȴ�״̬/��ʼ״̬
    //5:�ӳ�����״̬
    //6:ֹͣ����״̬
    //7:ѭ������״̬

    status_hand = 4;
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
    cross = 0;
    cross_flag = 0;
    uart_flag = UART_CNT;
    stop_judge = STOP;
    pid_flag = 0;
    pid_reset_flag = 0;
    home_flag = 0;
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
            //UARTprintf("case 0\r\n");
            if(message == 1 && time_count == 0) //ʶ��ֹͣ��ʶ,��ݮ�ɷ���ָֹͣ����Զ�����
            {
                    UARTprintf("stop\r\n");
                    /*�ر�Ѳ�����pid*/
                    //pid_flag = 0;
                    time_count = TIME_STOP;

                    if(route_len == 0 && route_flag == 0 && target > 2)
                    {
                        cross = 1;
                    }

                    if(cross == 1) //��һ��·��
                    {
                        //status_hand = 0;
                        keep = 1;
                        cross = 0;
                        if(route_flag == 1)
                        {
                            route[route_len] = 0;
                            route_len++;
                        }
                    }
                    else
                    {
                        /*��pid*/
                        pos_pid_flag = 0;
                        setspeed_flag = 0;

                        status_hand = 1;
                    }
            }
            else if(keep == 1)//��ǰΪѲ�߱���״̬
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
            }
            break;
        case 1:     //����ֹͣ״̬,��Ҫ����Ƿ��Ѿ�ͣ��,ֹͣ��ʹ�ñջ�
            //UARTprintf("case 1\r\n");
            if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B && time_count == 0) //��ֹͣ
            //if(time_count == 0)
            {
                stop_judge = STOP;
                UARTprintf("already stop\r\n");
                status_hand = 2; //�޸�Ϊֹͣ״̬2
                recognize_flag = 0; //ʶ������λ
                if(target <= 2 && route_flag == 1 && cross_flag == 0)
                {
                    route[route_len] = target;
                    route_len++;
                    turn_route_flag = target;

                    cross_flag = 1; //����Ϊת���ʶ
                    status_hand = 3;
                    x_task_flag = 0; //���ñջ��йر�ʶ
                    turn_task_flag = 0;
                    task_flag = 0;
                    time_count = TIME;
                }
                else if(route_flag == 1) //��ҩƷ,����ҩ����,�Ҳ�Ϊ��һ��ֹͣ
                {
                    recognize_flag = 1; //ʶ�������ͱ�ʶ��λ
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //����ʶ������
                    UARTprintf("d send\r\n");
                }
                /*else if(cross_flag == 0) //��û��ֹͣ��
                {
                    UARTprintf("1 or 2 room\r\n");
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
                    route[route_len] = turn_route_flag;
                    route_len++;
                    
                    status_hand = 3; //ֱ�ӽ���ջ�
                    cross_flag = 1; //��ʶλ��λ
                    x_pid_flag = 1;
                    x_last_flag = 0;
                    task_flag = 0;
                    x_set1 = 0;
                    x_set2 = 0;
                }*/
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
            break;
        case 2:     //ֹͣ״̬/��ݮ�ɵȴ�״̬
            //UARTprintf("case 2\r\n");
            /*if(message != 1 && recognize_flag == 1)
            {
                UARTCharPutNonBlocking(UART5_BASE, 'd'); //����ʶ������
                UARTprintf("d send again\r\n");
            }*/
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
                    
                }
                UARTprintf("decision:%d\r\n",turn_route_flag);
                status_hand = 3; //�޸�Ϊ�ջ�����״̬3
            }
            else if(route_len == -1) //�������м�¼·�ڲ�ֹͣ->�������
            {
                status_hand = 3; //�ȴ�,Ҫ��С������ҩ��
                turn_route_flag = 0;
                home_flag = 1;
                //if(question == 0)LED_flag = 2; //�̵���
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
        //UARTprintf("case 3\r\n");
            if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B && time_count == 0)
            {
                stop_judge--;
            }
            else
            {
                stop_judge = STOP;
            }
            if(stop_judge==0)
            {
                //UARTprintf("mission start\r\n");
                stop_judge = STOP;
                switch(turn_route_flag) //��ȡ�ջ�ָʾ
                {
                    case 0: //ֱ��
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else
                        {
                            task_flag = 1;
                        }
                        break;
                    case 1: //��ת
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;

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
                            time_count = TIME;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;

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
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0 && route_flag == 1)
                        {
                            LED_flag = 1;
                        }
                        else if(turn_task_flag == 0 && route_flag == 0) //δ����ת��ջ�,��ҩƷ��ж��
                        {
                            route_flag = 0;
                            if(question != 0)
                            {
                                UARTprintf("X "); //�ӳ�ȡҩָ��
                            }
                            LED_flag = 0; //Ϩ��
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;

                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1) //ת��ջ����,��ҩƷ��ж��
                        {
                            task_flag = 1; //�ջ����
                        }
                        break;
                }
                if(task_flag == 1) //��ɱջ�����
                {
                    UARTprintf("task done\r\n");
                    if(home_flag == 1)
                    {
                        status_hand = 4;
                        LED_flag = 2;
                    }
                    else
                    {
                    UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
                    //if((stop_count==-1 && question==2)||(stop_count==0 && question==1))UARTprintf("ok "); //�ڱ�������ĵ�һ��·����ɶ�����,�ͷ��ӳ������ź�
                    status_hand = 7; //�޸�Ϊ��������״̬
                    /*������Ҫ��pid*/
                    //x_pid_flag = 0;
                    uart_flag = UART_CNT;
                    x_task_flag = 0; //���ñջ��йر�ʶ
                    turn_task_flag = 0;
                    task_flag = 0;
                    pos = 0;
                    pos_speed = 0;

                    //pos_pid_flag = 1;
                    //setspeed_flag = 1;
                    }
                }
            }
            break;

        case 4:     //ָ��ȴ�״̬/��ʼ״̬
        //UARTprintf("case 4\r\n");
            if(message == 2) //��ݮ�����ʶ��
            {
                target = rData5[0]-'0';
                if(target>2)
                {
                    cross = 1; //��Ҫ����
                }
                else
                {
                    cross = 0;
                }
                //UARTprintf("%d ",target); //���ӳ�����ĸ��������Ϣ
                UARTprintf("room:%d\r\n",target);
            }
            if(target != 0 && Drug_Read()==1) //ҩƷ���װ������ݮ�������ʶ��
            {
                route_flag = 1;
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
        case 7:     //ѭ������״̬
            //UARTprintf("case 7\r\n");
            if(message == 1) //����֡���ֹͣ
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
                UARTprintf("error stop\r\n");
                uart_flag = UART_CNT;
            }
            else if(uart_flag == 0) //��ǰѭ��֡�ȶ�
            {
                uart_flag = -1;
                status_hand = 0;
                x_pid_flag = 0;
                data_flag = 0;
                pid_reset_flag = 1;
                pid_flag = 1;
                pos = 0;
                pos_speed = 0;

                setspeed_flag = 1;
                pos_pid_flag = 1;
            }
            else
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
            }
            break;
    }
}
