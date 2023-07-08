/*
 * hand_hand.c
 *
 *  Created on: 2023��6��6��
 *      Author: LENOVO
 */

#include "hand_hand.h"

#define TURN_X 1650
#define L_ROUND_X 950
#define R_ROUND_X 950
#define STRAIGHT_X1 1100
#define STRAIGHT_X2 1100
#define STOP 5
#define SPEED_B 70
#define UART_CNT 10
#define TIME 50
#define TIME_STOP 50

uint8_t status_hand = 4;    //��Ƭ����ǰ״̬
uint8_t route[10] = {0};    //С��·�ھ��߼�¼(0ֱ��,1��ת,2��ת)
int route_len = 0;      //С��·�ھ�������
extern uint8_t route_flag;     //ҩƷ״̬��־λ
extern int uart_flag;
extern uint8_t question;
extern uint8_t conflict_flag;

uint8_t turn_route_flag = 0; //�ջ�ָʾ(0ֱ��,1��ת,2��ת,3��ͷ)
uint8_t x_task_flag = 0; //ֱ�߱ջ�����ָʾ
uint8_t turn_task_flag = 0; //ת��ջ�����ָʾ
uint8_t task_flag = 0; //�ջ����ָʾ
uint8_t target = 0; //Ŀ����ʶ
uint8_t next_q = 0; //���ӳ���Ϣ��������һ״̬
int stop_count = 1; //���뱻�ӳ�������·����
uint8_t turn_flag = 0; //�Ƿ���ɵ�һ��ת��
uint8_t cross = 0; //����·����
uint8_t cross_flag = 0; //�Ƿ񾭹�����·��
int stop_judge = STOP; //ֹͣ�ж�����
uint8_t yellow_light = 0; //��ʶ�Ƿ񵽴����ѡ��
uint8_t back_flag = 0; //��һ���Ƿ��Ѿ�ת���
uint8_t block_flag = 0; //�����źű�ʶ
uint8_t back_count = 0;

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
    x_task_flag = 0;
    turn_task_flag = 0;
    LED_flag = 0; //Ϩ��
    target = 0;
    next_q = 0;
    stop_count = 1;
    turn_flag = 0;
    cross = 0;
    uart_flag = UART_CNT;
    stop_judge = STOP;
    pid_flag = 0;
    keep = 0;
    pid_reset_flag = 0;
    back_flag = 0;
    block_flag = 0;
    question = 0;
    back_count = 0;
    conflict_flag = 0;
    //UARTCharPutNonBlocking(UART5_BASE, 'R'); //����ݮ�ɷ��͸�λ��Ϣ
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
    
    4:X;
    5:ok;


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
            if(keep > 0) //���ι�������е�������(����ʱ�жϴ���)
            {
                if(message == 1)keep = 5;
                //UARTprintf("keep=%d\r\n",keep);
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ��
            }
            else if((message==1 && time_count==0 && question==2)||(message==1 && time_count==0 && question==1 && strcmp((char*)rData5,"s ")==0)) //ʶ��ֹͣ��ʶ,��ݮ�ɷ���ָֹͣ����Զ�����
            {

                if(cross>0) //����·��
                {
                    //UARTprintf("cross\r\n");
                    cross--;
                    keep = 5; //����
                    UARTCharPutNonBlocking(UART5_BASE, 'r');
                }
                else
                {
                    //UARTprintf("crossover\r\n");
                    stop_judge = STOP;
                    time_count = TIME_STOP;
                    pos_pid_flag = 0;
                    setspeed_flag = 0;
                    status_hand = 1;
                }
            }

            else if(question == 1 && message == 1 && time_count==0 && strcmp((char*)rData5,"b ") == 0)
            {
                //UARTprintf("back\r\n");
                stop_judge = STOP;
                time_count = TIME_STOP;
                
                pos_pid_flag = 0;
                setspeed_flag = 0;
                status_hand = 3;
                turn_route_flag = 3;

                x_task_flag = 0; //���ñջ��йر�ʶ
                turn_task_flag = 0;
                task_flag = 0;
            }
            break;
        case 1:     //����ֹͣ״̬,��Ҫ����Ƿ��Ѿ�ͣ��,ֹͣ��ʹ�ñջ�
            //if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B && time_count == -1) //��ֹͣ
            if(time_count == 0) //��ֹͣ
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
                //UARTprintf("already stop\r\n");
                //status_hand = 2; //�޸�Ϊֹͣ״̬2
                if(question==1)
                {
                    if(route_flag != 0 && yellow_light == 0) //ĸ����ת����Ϣ,����ѡ��δ���
                    {
                        turn_route_flag = 3 - route_flag; //�ӳ�����ת
                        if(route_flag == 0)turn_route_flag = 1; //����ϢĬ����ת
                    }
                    else if(yellow_light == 1) //��ѡ�����,��ĸ�����ͷ�������Ϣ
                    {   
                        turn_route_flag = 0;
                    }
                    status_hand = 3;
                    time_count = TIME;
                    stop_judge = STOP;
                    x_task_flag = 0; //���ñջ��йر�ʶ
                    turn_task_flag = 0;
                    task_flag = 0;
                }
                else if(question == 2)
                {
                    switch(route_flag)
                    {
                        case 1:
                            route_len = 0;
                            route[0] = 1;
                            route[1] = 1;
                            route_flag = 0; //����
                            break;
                        case 2:
                            route_len = 0;
                            route[0] = 1;
                            route[1] = 2;
                            route_flag = 0; //����
                            break;
                        case 3:
                            route_len = 0;
                            route[0] = 2;
                            route[1] = 1;
                            route_flag = 0; //����
                            break;
                        case 4:
                            route_len = 0;
                            route[0] = 2;
                            route[1] = 2;
                            route_flag = 0; //����
                            break;
                        default:
                            break;
                    }

                    if(route_flag == 0)
                    {
                        if(back_flag == 0) //����ͻ
                        {
                            turn_route_flag = route[route_len];
                        }
                        else //��ͻ
                        {
                            turn_route_flag = 3 - route[route_len]; //���෴����
                        }
                        route_len++;
                        status_hand = 3;
                        time_count = TIME;
                        stop_judge = STOP;
                        x_task_flag = 0; //���ñջ��йر�ʶ
                        turn_task_flag = 0;
                        task_flag = 0;
                    }
                }
            }
            break;
        /*case 2:     //ֹͣ״̬/��ݮ�ɵȴ�״̬
            //UARTprintf("case 2\r\n");
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
                UARTprintf("decision:%d\r\n",turn_route_flag);
                status_hand = 3; //�޸�Ϊ�ջ�����״̬3
            }
            else if(route_len == -1) //�������м�¼·�ڲ�ֹͣ->�������
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
            break;*/
        case 3:     //�ջ�����״̬
        //UARTprintf("case 3\r\n");
            if(fabs(speed1) <= SPEED_B && fabs(speed2) <= SPEED_B && time_count==0) //ֹͣ˵���������һ�ջ�������״̬2ת����3
            {
                stop_judge--;
            }
            else
            {
                stop_judge = STOP;
            }
            if(stop_judge<=0 && time_count==0)
            {
                //stop_judge = STOP;
                //UARTprintf("mission start\r\n");
                switch(turn_route_flag) //��ȡ�ջ�ָʾ
                {
                    case 0: //ֱ��
                        if(x_task_flag == 0) //δ����ֱ�߱ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

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
                            stop_judge = STOP;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

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
                            stop_judge = STOP;

                            x_set1 = STRAIGHT_X1;
                            x_set2 = STRAIGHT_X2;
                            x_task_flag = 1;
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�
                        {
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

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
                            stop_judge = STOP;
                            if(question == 2)
                            {
                                x_set1 = 4000;
                                x_set2 = 3900;
                            }
                            else
                            {
                                x_set1 = STRAIGHT_X1;
                                x_set2 = STRAIGHT_X2;
                            }
                            x_task_flag = 1;

                            if(question == 1 && yellow_light == 1) //��һ�������ѡ��
                            {
                                turn_task_flag = 1; //����ת��
                                task_flag = 1;
                            }
                        }
                        else if(turn_task_flag == 0) //δ����ת��ջ�,��ҩƷ��ж��
                        {
                            route_flag = 0;
                            x_last_flag = 0;
                            x_pid_flag = 1;
                            time_count = TIME;
                            stop_judge = STOP;

                            x_set1 = TURN_X;
                            x_set2 = -TURN_X;
                            turn_task_flag = 1;

                        }
                        else //ת��ջ����,��ҩƷ��ж��
                        {
                            if(question == 1) //��һ��δ�����ѡ��
                            {
                                if(yellow_light == 0)
                                {
                                    LED_flag = 2;
                                    //UARTprintf("light on\r\n");
                                }

                                if(message == 4) //ĸ�����жҩ
                                {
                                    //UARTprintf("light off\r\n");
                                    LED_flag = 0; //���
                                    yellow_light = 1; //��ѡ���������
                                }
                                else if(message == 5) //ĸ���ͷ�����
                                {
                                    //UARTprintf("car release\r\n");

                                    cross = 1;
                                    task_flag = 1;
                                }
                            }
                            else if(question == 2)
                            {
                                task_flag = 1; //�ջ����
                            }
                        }
                        break;
                }
                if(task_flag == 1) //��ɱջ�����
                {
                    //UARTprintf("task done\r\n");
                    /*������Ҫ��pid*/
                    //x_pid_flag = 0;
                    uart_flag = UART_CNT;
                    x_task_flag = 0; //���ñջ��йر�ʶ
                    turn_task_flag = 0;
                    task_flag = 0;
                    pos = 0;
                    pos_speed = 0;
                    if(question == 1)
                    {
                        if(back_flag == 0 && yellow_light == 1) //�������ѡ��
                        {
                            //UARTprintf("back set\r\n");
                            status_hand = 7; //�޸�Ϊ��������״̬
                            back_flag = 1;
                            UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
                        }
                        else if(back_flag == 1) //�����ѡ��
                        {
                            status_hand = 8;
                        }
                        else
                        {
                            status_hand = 7; //�޸�Ϊ��������״̬
                            UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������                       
                        }
                    }
                    else if(question == 2)
                    {
                        if(block_flag == 1) //����
                        {
                            //UARTprintf("block\r\n");
                            block_flag = 0;
                            back_flag = 0;
                            status_hand = 5;
                            next_q = 7;
                            cross = 1; //��������Ҫ����һ��·��
                            keep = 0;
                        }
                        else if(back_flag == 0) //����ͻ���ͻ���� 
                        {
                            //UARTprintf("no conflict\r\n");
                            if(conflict_flag == 0 && question==2)
                            {
                                status_hand = 7;
                                UARTprintf("ok ");
                                conflict_flag = 1;
                            }
                            else
                            {
                                if(turn_route_flag == 0) //û����Ч��ת����Ϣ��,���յ�
                                {
                                    status_hand = 8;
                                }
                                else
                                {
                                    status_hand = 7;
                                }
                            }

                        }
                        else
                        {
                            block_flag = 1; //�����һ�������������״̬
                            UARTprintf("ok "); //��ĸ���ͷ������ź�
                            /*����ת��*/
                            x_task_flag = 0;
                            turn_task_flag = 0;
                            task_flag = 0;

                            turn_route_flag = 3;
                            time_count = TIME;
                            stop_judge = STOP;
                        }
                    }
                }
            }
            break;

        case 4:     //ָ��ȴ�״̬/��ʼ״̬
            if((question == 1 && Drug_Read() == 1)||(question == 2 && message == 4)) //��һ�� �� �ӳ������ź�
            {
                //UARTprintf("go\r\n");
                cross = question; //����·����
                keep = 0;
                status_hand = 7;
                block_flag = 0;
                if(question == 1)
                {
                    yellow_light = 0;
                    back_flag = 0;
                }
                /*����Ѳ�߶�Ӧpid*/
                UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
                uart_flag = UART_CNT;
            }
            else //ĸ���д���ת������
            {
                //UARTprintf("wait!\r\n");
            }
            break;
        case 5:     //ĸ������״̬
            if(message == 5) //�����źű��ͷ�
            {
                status_hand = next_q;
                next_q = 0;
                if(status_hand == 7)UARTCharPutNonBlocking(UART5_BASE, 'r'); //����Ѳ������
            }
            break;
        case 7:     //ѭ������״̬
            //UARTprintf("case 7\r\n");
            if(message == 1 && (strcmp((char*)rData5,"s ") == 0 || strcmp((char*)rData5,"b ") == 0)) //����֡���ֹͣ
            {
                UARTCharPutNonBlocking(UART5_BASE, 'r');
                //UARTprintf("error stop\r\n");
                uart_flag = UART_CNT;
            }
            else if(uart_flag == 0) //��ǰѭ��֡�ȶ�
            {
                //UARTprintf("goto case 0\r\n");
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
                //UARTprintf("r printf\r\n");
                UARTCharPutNonBlocking(UART5_BASE, 'r');
            }
            break;
        case 8: //ɶ������
            if(question == 2)LED_flag = 1;
            //UARTprintf("do nothing\r\n");
            break;
    }
}
