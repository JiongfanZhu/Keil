/*
 * hand_hand.c
 *
 *  Created on: 2023��6��6��
 *      Author: LENOVO
 */

#include "hand_hand.h"

uint8_t status_hand = 0;    //��Ƭ����ǰ״̬
extern uint8_t route_flag;     //ҩƷ״̬��־λ

uint8_t turn_route_flag = 0; //�ջ�ָʾ(0ֱ��,1��ת,2��ת,3��ͷ)
uint8_t x_task_flag = 0; //ֱ�߱ջ�����ָʾ
uint8_t turn_task_flag = 0; //ת��ջ�����ָʾ
uint8_t task_flag = 0; //�ջ����ָʾ
uint8_t target_flag = 0; //Ŀ���ȷ����ʶ

uint8_t question = 0; //��Ŀѡ��
uint8_t mom_car = 0; //ĸ��������Ϣ
uint8_t mom_drug = 0; //ĸ��жҩ��ʶ
uint8_t mom_decision = 0; //ĸ����һ��ת����Ϣ(0��ת,1��ת)
uint8_t decision_flag = 0; //�Ƿ����ĸ��ת��
uint8_t back_flag = 0; //��·��ʶ
uint8_t next_q = 0; //����������һ״̬
uint8_t auto_count = 0; //�Զ�Ѳ��·����

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
    //5:�ӳ�����״̬
    status_hand = 5;
    x_pid_flag = 0;
    route_flag = 0; //��ҩƷ����
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //Ϩ��
    target_flag = 0;
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
            if(message == 1 && strcmp((char*)rData5,"s ") == 0) //ʶ��ֹͣ��ʶ,��ݮ�ɷ���ָֹͣ����Զ�����
            {
                status_hand = 1; //�л�Ϊ����ֹͣ״̬
                /*������Ҫ��pid*/
                b_pid_flag = 0;
                theta_pid_flag = 0;
                setspeed_flag = 0;
            }
            break;
        case 1:     //����ֹͣ״̬
            if(speed1 == 0 && speed2 ==0) //�Ѿ�ֹͣ
            {
                status_hand = 2; //�л�Ϊֹͣ״̬2
                if(back_flag == 1) //��·��ʶ��λ
                {
                    next_q = 3;
                    status_hand = 5;
                    turn_route_flag = 3; //�����������ȵ�ͷ
                    back_flag = 0;
                }
                else //һ����ѭ��
                {
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //����ʶ������
                }
            }
            break;
        case 2:     //ֹͣ״̬
            if(message == 1) //�д�����Ϣ
            {
                status_hand = 3; //�ջ�״̬3
                if(decision_flag == 1 && question == 1) //ĸ��ת����Ϣ��Ч(��һ��)
                {
                    turn_route_flag = 2 - mom_decision; //��ĸ��ת���෴
                    decision_flag = 0;
                }
                else if(strcmp((char*)rData5,"l ")==0) //��ת
                {
                    if(question == 2) //�ڶ���
                    {
                        if(decision_flag == 1) //ĸ��ת���ʶ�Ƿ���Ч
                        {
                            if(mom_decision == 0) //ĸ��Ҳ��ת,��ͻ
                            {
                                turn_route_flag = 2; //�ӳ�����ת
                                back_flag = 1; //��·��ʶ��λ
                            }
                            else //ĸ����ת,����ͻ
                            {
                                turn_route_flag = 1;
                                back_flag = 0;
                            }
                            decision_flag = 0; //ĸ��ת����Чһ�κ���Ч
                        }
                        else //����ת��
                        {
                            turn_route_flag = 1;
                        }  
                    }
                    else if(auto_count > 0) //�Զ�Ѳ������
                    {
                        auto_count--; //Ѳ��·��-1
                        turn_route_flag = 0; //����ֱ��
                    }
                    else if(auto_count == 0) //Ѳ������
                    {
                        auto_count--; //Ѳ��·��-1
                        turn_route_flag = 3; //��ͷ
                    }
                }
                else if(strcmp((char*)rData5,"r ")==0) //��ת
                {
                    if(question == 2)
                    {
                        if(decision_flag == 1)
                        {
                            if(mom_decision == 0)
                            {
                                turn_route_flag = 2;
                                back_flag = 0;
                            }
                            else
                            {
                                turn_route_flag = 1;
                                back_flag = 1;
                            }
                            decision_flag = 0;
                        }
                        else
                        {
                            turn_route_flag = 2;
                        }
                    }
                    else if(auto_count > 0)
                    {
                        auto_count--;
                        turn_route_flag = 0;
                    }
                    else if(auto_count == 0)
                    {
                        turn_route_flag = 3;
                    }
                }
                else if(strcmp((char*)rData5,"S ")==0) //ֱ��
                {
                    if(question == 2)
                    {
                        turn_route_flag = 0;
                    }
                    else if(auto_count > 0)
                    {
                        auto_count--;
                        turn_route_flag = 0;
                    }
                    else if(auto_count == 0)
                    {
                        turn_route_flag = 3;
                    }
                }
                else if(strcmp((char*)rData5,"b ")==0) //��ͷ,��ҩ��
                {
                    if(question == 2)LED_flag = 1; //�������
                    status_hand = 5; //��������
                }
            }
            x_task_flag = 0; //��رջ���ʶ����
            turn_task_flag = 0;
            task_flag = 0;
            break;
        case 3:     //�ջ�״̬
            if(speed1 == 0 && speed2 == 0) //�������һ�ջ�����
            {
                switch(turn_route_flag) //�ջ���ʶ
                {
                    case 0: //ֱ��
                        if(x_task_flag == 0) //δ���ֱ�ж���
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
                        if(x_task_flag == 0) //δ���ֱ�ж���
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ���ת����
                        {
                            x_set1 += -ROUND_X;
                            x_set2 += ROUND_X;
                            turn_task_flag = 1;
                        }
                        else
                        {
                            task_flag = 1;
                        }
                        break;

                    case 2: //��ת
                        if(x_task_flag == 0)
                        {
                            x_set1 = STRAIGHT_X;
                            x_set2 = STRAIGHT_X;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0)
                        {
                            x_set1 += ROUND_X;
                            x_set2 += -ROUND_X;
                            turn_task_flag = 1;
                        }
                        else
                        {
                            task_flag = 1;
                        }
                        break;

                    case 3: //��ͷ
                        if(turn_task_flag == 0)
                        {
                            LED_flag = 0; //Ϩ��
                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1)
                        {
                            task_flag = 1;
                        }
                        break;
                }
                if(task_flag == 1)
                {
                    if(auto_count == 0 && question == 1) //�Զ�Ѳ������(��һ��),���������
                    {
                        next_q = 0;
                        status_hand = 5;
                        LED_flag = 2; //���Ƶ�
                        auto_count--; //Ѳ��·��-1
                    }
                    else
                    {
                        UARTCharPutNonBlocking(UART5_BASE, 'r'); //����ѭ������
                        status_hand = 0; //�л�Ϊ״̬0
                        /*������Ҫ��pid*/
                        x_pid_flag = 0;
                        b_pid_flag = 1;
                        theta_pid_flag = 1;
                        setspeed_flag = 1;
                    }
                }
            }

        case 4:     //ָ��ȴ�ģʽ
            if(message == 1 && rData5[0]>='1' && rData5[0]<='8' && rData5[1] == ' ') //�ӳ�������Ϣ
            {
                target_flag = rData5[0] - '0'; //��ȡ��Ϣ
                if(mom_car == target_flag) //ͬһ����
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
                mom_car = rData1[0]; //ĸ����Ϣ
            }

            /*������߲��ֵķ�������*/
            if((route_flag == 1 && question == 1)||(mom_drug == 1 && question == 2))
            {
                status_hand = 0; //�л�Ϊ״̬0
                if(question == 1)auto_count = 2; //��һ�������Զ�Ѳ��
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ��ָ��
                /*������Ҫ��pid*/
                b_pid_flag = 1;
                theta_pid_flag = 1;
                setspeed_flag = 1;
            }
            break;
        case 5: //����
            if(message == 4) // ĸ���ͷ������ź�
            {
                status_hand = next_q;
                next_q = 0;
            }
            break;
    }
}



