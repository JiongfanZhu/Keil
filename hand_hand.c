/*
 * hand_hand.c
 *
 *  Created on: 2023��6��6��
 *      Author: LENOVO
 */

#include "hand_hand.h"

uint8_t status_hand = 0;    //��Ƭ����ǰ״̬
uint8_t route[10] = {0};    //С��·�ھ��߼�¼(0ֱ��,1��ת,2��ת)
int route_len = 0;      //С��·�ھ�������
extern uint8_t route_flag;     //ҩƷ״̬��־λ

uint8_t recognize_flag = 0; //ʶ�������ʶ
uint8_t turn_route_flag = 0; //�ջ�ָʾ(0ֱ��,1��ת,2��ת,3��ͷ)
uint8_t x_task_flag = 0; //ֱ�߱ջ�����ָʾ
uint8_t turn_task_flag = 0; //ת��ջ�����ָʾ
uint8_t task_flag = 0; //�ջ����ָʾ
uint8_t target_flag = 0; //Ŀ���ȷ��

void StatusReset(void)
{
    //status_hand�ļ���״̬
    //0:��������״̬/ָ������״̬
    //1:����ֹͣ״̬
    //2:ֹͣ״̬/��ݮ�ɵȴ�״̬
    //3:�ջ�����״̬
    //4:�û�ָ��ȴ�״̬/��ʼ״̬
    status_hand = 4;
    x_pid_flag = 0;
    route_len = 0; //��վ�����
    memset(route,0,sizeof(route)); //��վ��߼�¼
    route_flag = 0; //��ҩƷ����
    recognize_flag = 0; //��ʶ�����󷢳�
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //Ϩ��

}

void StatusDeal(uint8_t message) //message=0��ʾ�޴�����Ϣ,�����д�����Ϣ
{
    switch(status_hand)
    {
        case 0:     //��������״̬
            if(message == 1 && strcmp((char*)rData5,"s ") == 0) //ʶ��ֹͣ��ʶ
            {
                status_hand = 1; //�޸�Ϊ����ֹͣ״̬1
                b_pid_flag = 0; //�ر�����pid(xpid������ʱӦ�ǹر�״̬)
                theta_pid_flag = 0;
                setspeed_flag = 0;
            }
            break;
        case 1:     //����ֹͣ״̬,��Ҫ����Ƿ��Ѿ�ͣ��,ֹͣ��ʹ�ñջ�
            if(speed1 == 0 && speed2 ==0) //��ֹͣ
            {
                //x_pid_flag = 0;
                status_hand = 2; //�޸�Ϊֹͣ״̬2
                recognize_flag = 0; //ʶ���������
                if(route_flag == 1) //��ҩƷ,����ҩ����
                {
                    recognize_flag = 1; //ʶ��������λ
                    UARTCharPutNonBlocking(UART5_BASE, 'd'); //����ʶ������
                }
            }
            break;
        case 2:     //ֹͣ״̬/��ݮ�ɵȴ�״̬
            if(message == 1 && recognize_flag == 1) //�д�����Ϣ�ҷ�����ʶ������->��ҩ����
            {
                if(strcmp((char*)rData5,"l ")==0) //��ת
                {
                    route[route_len] = 1;
                    route_len++;
                    turn_route_flag = 1;
                }
                else if(strcmp((char*)rData5,"r ")==0) //��ת
                {
                    route[route_len] = 2;
                    route_len++;
                    turn_route_flag = 2;
                }
                else if(strcmp((char*)rData5,"s ")==0) //ֱ��
                {
                    route[route_len] = 0;
                    route_len++;
                    turn_route_flag = 0;
                }
                else if(strcmp((char*)rData5,"b ")==0) //��ͷ,Ŀ��ҩ��
                {
                    route_len--; //ɾȥһ������,��ǰroute_len��Ӧ���һ�������Ϣ
                    turn_route_flag = 3;
                    LED_flag = 1; //�����
                }
                status_hand = 3; //�޸�Ϊ�ջ�����״̬3
            }
            else if(route_len == -1) //�������м�¼·�ڲ�ֹͣ->�������
            {
                status_hand = 4; //�ȴ�
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
            if(speed1 == 0 && speed2 == 0) //ֹͣ˵���������һ�ջ�������״̬2ת����3
            {
                switch(turn_route_flag) //��ȡ�ջ�ָʾ
                {
                    case 0: //ֱ��
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_set1 = 100;
                            x_set2 = 100;
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
                            x_set1 = 100;
                            x_set2 = 100;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_set1 = -100;
                            x_set2 = 100;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                            //x_task_flag = 0; //����Ҫִ��һ��ֱ�߱ջ�
                        }
                        else //���αջ����
                        {
                            task_flag = 1;
                        }
                        break;

                    case 2: //��ת
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_set1 = 100;
                            x_set2 = 100;
                            x_pid_flag = 1;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_set1 = 100;
                            x_set2 = -100;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                            //x_task_flag = 0; //����Ҫִ��һ��ֱ�߱ջ�
                        }
                        else //�ջ����
                        {
                            task_flag = 1;
                        }
                        break;

                    case 3: //��ͷ
                        if(turn_task_flag == 0 && route_flag == 0) //δ����ת��ջ�,��ҩƷ��ж��
                        {
                            LED_flag = 0; //Ϩ��
                            x_set1 = 400;
                            x_set2 = -400;
                            x_pid_flag = 1;
                            turn_task_flag = 1;
                        }
                        else if(turn_task_flag == 1 && route_flag == 0) //ת��ջ����,��ҩƷ��ж��
                        {
                            task_flag = 1; //�ջ����
                        }
                        break;
                }
                if(task_flag == 1)
                {
                    UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
                    status_hand = 0; //�޸�Ϊ��������״̬
                    x_pid_flag = 0;
                    b_pid_flag = 1;
                    theta_pid_flag = 1;
                    setspeed_flag = 1;
                }
            }

        case 4:     //ָ��ȴ�״̬/��ʼ״̬
            if(message == 1 && strcmp((char*)rData5,"X ") == 0) //��ݮ�����ʶ��
            {
                status_hand = 0; //�޸�Ϊ��������״̬
                b_pid_flag = 1;
                theta_pid_flag = 1;
                setspeed_flag = 1;
            }
            break;
    }
}


