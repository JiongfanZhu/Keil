/*
 * hand_hand.c
 *
 *  Created on: 2023��6��6��
 *      Author: LENOVO
 */

#include "hand_hand.h"

uint8_t status_hand = 0;    //��Ƭ����ǰ״̬
extern uint8_t route_flag;     //ҩƷ״̬��־λ 

uint8_t recognize_flag = 0; //ʶ�������ʶ
uint8_t turn_route_flag = 0; //�ջ�ָʾ(0ֱ��,1��ת,2��ת,3��ͷ)
uint8_t x_task_flag = 0; //ֱ�߱ջ�����ָʾ
uint8_t turn_task_flag = 0; //ת��ջ�����ָʾ
uint8_t task_flag = 0; //�ջ����ָʾ
uint8_t target_flag = 0; //Ŀ����ʶ

uint8_t uart_flag = 0; //˫��ͨ�ű�ʶ,���ĸ���Ƿ��Ѿ�����·��
uint8_t car_flag = 0; //ĸ��ת���ʶ(0��ת,1��ת)
uint8_t question = 0; //��Ŀ��ʶ
uint8_t mom_car = 0; //ĸ��Ŀ����¼
uint8_t mom_drug = 0; //ĸ��жҩ��ʶ
uint8_t mom_decision = 0; //ĸ����һ��ת�����(0��ת,1��ת)
uint8_t decision_flag = 0; //������ǰת������Ƿ���Ч
uint8_t back_flag = 0; //�ӳ��Ƿ���Ҫ������·��ʶ
uint8_t next_q = 0; //�������Ĵ�̬
uint8_t auto_count = 0; //�Զ�Ѳ������

#define TURN_X 400
#define ROUND_X 100
#define STRAIGHT_X 100

void StatusReset(void)
{
    //status_hand�ļ���״̬
    //0:��������״̬/ָ������״̬
    //1:����ֹͣ״̬
    //2:ֹͣ״̬/��ݮ�ɵȴ�״̬
    //3:�ջ�����״̬
    //4:�û�ָ��ȴ�״̬/��ʼ״̬
    //5:����״̬
    status_hand = 5;
    x_pid_flag = 0;
    route_len = 0; //��վ�����
    memset(route,0,sizeof(route)); //��վ��߼�¼
    route_flag = 0; //��ҩƷ����
    recognize_flag = 0; //��ʶ�����󷢳�
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //Ϩ��
    target_flag = 0;
    uart_flag = 0;
    car_flag = 0;
    question = 0;
    mom_car = 0;
    mom_drug = 0;
    back_flag = 0;
    next_q = 0;
    UARTCharPutNonBlocking(UART5_BASE, 'R'); //����ݮ�ɷ��͸�λ��Ϣ
}

/*
��ݮ�ɷ���ָ�:
    "s ":С��ֹͣ;
    "l ":С����ת;
    "r ":С����ת;
    "S ":С��ֱ��;
    "b ":С����ͷ(����ҩ��);
    "X ":��ݮ���Ѽ�¼Ŀ�겡��(XΪһ����);

��ݮ�ɽ���ָ�:
    'd':��ݮ�ɽ���ʶ��;
    'r':��ݮ�ɽ���Ѳ��;
    'R':��ݮ�ɽ��и�λ;

message����:
    0:��ʱ�жϽ���;
    1:��ݮ�ɴ�����Ϣ����;
    2:ĸ��������Ϣ(��Ŀѡ��);
    3:ĸ��жҩ���;
    4:ĸ�������ź��ͷ�;

˫��ͨ��ָ�:
    "X ":ĸ�����жҩ,�ӳ�ȡҩָ��;
    "1 ":���ӳ�����ĸ��������Ϣ;
    "Q1 "/"Q2 ":��ĸ�����Ͷ�Ӧ��Ŀ��Ϣ;
    "ok ":��ĸ��/�ӳ��������ͷ��ź�;

PS:��ݮ����С�������źź�,���ǻص�ָ��ȴ�״̬
   ����ʼ��ʱ��ݮ�ɵ�״̬����Ҫ��ȶ(?)
*/

void StatusDeal(uint8_t message) //message=0��ʾ�޴�����Ϣ,�����д�����Ϣ
{
    switch(status_hand)
    {
        case 0:     //��������״̬
            if(message == 1 && strcmp((char*)rData5,"s ") == 0) //ʶ��ֹͣ��ʶ,��ݮ�ɷ���ָֹͣ����Զ�����
            {
                status_hand = 1; //�޸�Ϊ����ֹͣ״̬1
                /*�ر�Ѳ�����pid*/
                b_pid_flag = 0;
                theta_pid_flag = 0;
                setspeed_flag = 0;
            }
            break;
        case 1:     //����ֹͣ״̬,��Ҫ����Ƿ��Ѿ�ͣ��,ֹͣ��ʹ�ñջ�
            if(speed1 == 0 && speed2 ==0) //��ֹͣ
            {
                status_hand = 2; //�޸�Ϊֹͣ״̬2
                recognize_flag = 1; //ʶ�������ͱ�ʶ��λ
                if(back_flag == 1) //��·��λ,��Ҫ����,�ȴ�ĸ���ͷ�
                {
                    next_q = 3;
                    status_hand = 5;
                    turn_route_flag = 3; //�´��ͷ�������ֱ�ӵ�ͷ
                }
                else //һ���Ե�ѭ��
                {
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //����ʶ������
                }
            }
            break;
        case 2:     //ֹͣ״̬/��ݮ�ɵȴ�״̬
            if(message == 1 && recognize_flag == 1) //�д�����Ϣ�ҷ�����ʶ������->��ҩ����
            {
                if(strcmp((char*)rData5,"l ")==0) //��ת
                {
                    if(question == 2) //�ڶ���
                    {
                        if(decision_flag == 1) //ĸ��ת���ʶ��Ч
                        {
                            if(mom_decision == 0) //ĸ��Ҳ��ת,·�߳�ͻ
                            {
                                turn_route_flag = 2; //�ӳ�����ת
                                back_flag = 1; //��·��ʶ��λ 
                            }
                            else //ĸ����ת,·�߲���ͻ
                            {
                                turn_route_flag = 1;
                                back_flag = 0;
                            }
                            decision_flag = 0; //ִ��һ�κ���Чĸ����ת������
                        }
                        else
                        {
                            turn_route_flag = 1;
                        }  
                    }
                    else if(auto_count > 0) //��һ���Զ�Ѳ��
                    {
                        auto_count--; //Ѳ��·��-1
                        turn_route_flag = 0; //����ת��,ֱ��
                    }
                    else if(auto_count == 0) //������ѡ��
                    {
                        turn_route_flag = 3; //��ͷ
                    }
                }
                else if(strcmp((char*)rData5,"r ")==0) //��ת
                {
                    if(question == 2) //�ڶ���
                    {
                        if(decision_flag == 1) //ĸ��ת���ʶ��Ч
                        {
                            if(mom_decision == 0) //ĸ����ת,·�߲���ͻ
                            {
                                turn_route_flag = 2;
                                back_flag = 0;
                            }
                            else //ĸ��Ҳ��ת,·�߳�ͻ
                            {
                                turn_route_flag = 1; //�ӳ�����ת
                                back_flag = 1; //��·��ʶ��λ 
                            }
                            decision_flag = 0; //ִ��һ�κ���Чĸ����ת������
                        }
                        else
                        {
                            turn_route_flag = 2;
                        }
                    }
                    else if(auto_count > 0) //��һ���Զ�Ѳ��
                    {
                        auto_count--; //Ѳ��·��-1
                        turn_route_flag = 0; //����ת��,ֱ��
                    }
                    else if(auto_count == 0) //������ѡ��
                    {
                        turn_route_flag = 3; //��ͷ
                    }
                }
                else if(strcmp((char*)rData5,"S ")==0) //ֱ��
                {
                    if(question == 2)
                    {
                        turn_route_flag = 0;
                    }
                    else if(auto_count > 0) //��һ���Զ�Ѳ��
                    {
                        auto_count--; //Ѳ��·��-1
                        turn_route_flag = 0; //����ת��,ֱ��
                    }
                    else if(auto_count == 0) //������ѡ��
                    {
                        turn_route_flag = 3; //��ͷ
                    }
                }
                status_hand = 3; //�޸�Ϊ�ջ�����״̬3
            }
            x_task_flag = 0; //���ñջ��йر�ʶ
            turn_task_flag = 0;
            task_flag = 0;
            break;
        case 3:     //�ջ�����״̬
            if(speed1 == 0 && speed2 == 0) //ֹͣ˵���������һ�ջ�������״̬2ת����3
            {
                switch(turn_route_flag) //��ȡ�ջ�ָʾ
                {
                    case 0: //ֱ��
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
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
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_set1 += -ROUND_X;
                            x_set2 += ROUND_X;
                            turn_task_flag = 1;
                            //x_task_flag = 0; //����Ҫִ��һ��ֱ�߱ջ�
                        }
                        else //�ջ����
                        {
                            task_flag = 1;
                        }
                        break;

                    case 2: //��ת
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_set1 += ROUND_X;
                            x_set2 += -ROUND_X;
                            turn_task_flag = 1;
                            //x_task_flag = 0; //����Ҫִ��һ��ֱ�߱ջ�
                        }
                        else //�ջ����
                        {
                            task_flag = 1;
                        }
                        break;

                    case 3: //��ͷ
                        if(turn_task_flag == 0) //δ����ת��ջ�,��ҩƷ��ж��
                        {
                            LED_flag = 0; //Ϩ��
                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1) //ת��ջ����,��ҩƷ��ж��
                        {
                            task_flag = 1; //�ջ����
                        }
                        break;
                }
                if(task_flag == 1)
                {
                    UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
                    status_hand = 0; //�޸�Ϊ��������״̬
                    if(auto_count == 0 && question == 1) // ��ѡ���ͷ���(��һ��)
                    {
                        next_q = 0;
                        status_hand = 5;
                        LED_flag = 2; //�����Ƶ�
                    }
                    else
                    {
                        /*������Ҫ��pid*/
                        x_pid_flag = 0;
                        b_pid_flag = 1;
                        theta_pid_flag = 1;
                        setspeed_flag = 1;
                    }
                }
            }

        case 4:     //ָ��ȴ�״̬/��ʼ״̬
            if(message == 1 && rData5[0]>='1' && rData5[0]<='8' && rData5[1] == ' ') //��ݮ�����ʶ��
            {
                target_flag = rData5[0] - '0'; // ��֪Ŀ�겡��,��һ���ȴ�ҩƷװ��
                if(mom_car == target_flag) // ͬһĿ�겡��
                {
                    question = 1;
                    UARTprintf("Q1 ");
                }
                else
                {
                    question = 2;
                    UARTprintf("Q2 ");
                }
            }
            else if(message == 2)
            {
                mom_car = rData1[0];
            }

            /*ҩƷ���װ������֪Ŀ�겡��(Q1)��ĸ�����жҩ(Q2)*/
            if((route_flag == 1 && question == 1)||(mom_drug == 1 && question == 2))
            {
                status_hand = 0; //�޸�Ϊ��������״̬
                if(question == 1)auto_count = 2; //����ֹͣ�������
                /*����Ѳ�߶�Ӧpid*/
                b_pid_flag = 1;
                theta_pid_flag = 1;
                setspeed_flag = 1;
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
            }
            break;
        case 5: //����״̬
            if(message == 4) // �����ͷ�
            {
                status_hand = next_q;
                next_q = -1;
            }
            break;
    }
}



