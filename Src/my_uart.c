/*
 * my_uart.c
 *
 *  Created on: 2023��6��6��
 *      Author: LENOVO
 */

#include "my_uart.h"

float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // ��¼����λ��ʼ�ĵط�
    uint8_t data_End_Num = 0; // ��¼����λ�����ĵط�
    uint8_t data_Num = 0; // ��¼����λ��
    uint8_t minus_Flag = 0; // �ж��ǲ��Ǹ���
    float data_return = 0; // �����õ�������
    for(int i=0;i<200;i++) // ���ҵȺź͸�̾�ŵ�λ��
    {
        if(rData3[i] == '=') data_Start_Num = i + 1; // +1��ֱ�Ӷ�λ��������ʼλ
        if(rData3[i] == ' ')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(rData3[data_Start_Num] == '-') // ����Ǹ���
    {
        data_Start_Num += 1; // ����һλ������λ
        minus_Flag = 1; // ����flag
    }
        data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // ���ݹ�4λ
    {
        data_return = (rData3[data_Start_Num]-48) +
        (rData3[data_Start_Num+2]-48)*0.1f +
        (rData3[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // ���ݹ�5λ
    {
        data_return = (rData3[data_Start_Num]-48) +
        (rData3[data_Start_Num+2]-48)*0.1f +
        (rData3[data_Start_Num+3]-48)*0.01f +
        (rData3[data_Start_Num+4]-48)*0.001f;
    }
     else if(data_Num == 6) // ���ݹ�6λ
    {
        data_return = (rData3[data_Start_Num]-48) +
        (rData3[data_Start_Num+2]-48)*0.1f +
        (rData3[data_Start_Num+3]-48)*0.01f +
        (rData3[data_Start_Num+4]-48)*0.001f +
        (rData3[data_Start_Num+5]-48)*0.0001f;
    }
    if(minus_Flag == 1) data_return = -data_return;
    // printf("data=%.2f\r\n",data_return);
    return data_return;
}


void USART_PID_Adjust(void)
{
    float data_Get = Get_Data(); // ��Ž��յ�������
    // printf("data=%.2f\r\n",data_Get);
        if(rData3[0]=='P' && rData3[1]=='t') // �Ƕ�P
            PID_para(3,1,data_Get);
        else if(rData3[0]=='I' && rData3[1]=='t') // �Ƕ�I
            PID_para(3,2,data_Get);
        else if(rData3[0]=='D' && rData3[1]=='t') // �Ƕ�D
            PID_para(3,3,data_Get);
        else if(rData3[0]=='P' && rData3[1]=='v' && rData3[2]=='1') // �ٶȻ�P
            PID_para(1,1,data_Get);
        else if(rData3[0]=='I' && rData3[1]=='v' && rData3[2]=='1') // �ٶȻ�I
            PID_para(1,2,data_Get);
        else if(rData3[0]=='D' && rData3[1]=='v' && rData3[2]=='1') // �ٶȻ�D
            PID_para(1,3,data_Get);
//      else if(rData3[0]=='S' && rData3[1]=='p' && rData3[2]=='e') //Ŀ���ٶ�
//          PID_para(1,4,data_Get);
//      else if(rData3[0]=='P' && rData3[1]=='o' && rData3[2]=='s') //Ŀ��λ��
////            PID_para(3,4,data_Get);
//          set_x1 = data_Get;


        else if(rData3[0]=='P' && rData3[1]=='b') // �ؾ�P
            PID_para(4,1,data_Get);
        else if(rData3[0]=='I' && rData3[1]=='b') // �ؾ�I
            PID_para(4,2,data_Get);
        else if(rData3[0]=='D' && rData3[1]=='b') // �ؾ�D
            PID_para(4,3,data_Get);
        else if(rData3[0]=='P' && rData3[1]=='v' && rData3[2]=='2') // �ٶȻ�P
            PID_para(2,1,data_Get);
        else if(rData3[0]=='I' && rData3[1]=='v' && rData3[2]=='2') // �ٶȻ�I
            PID_para(2,2,data_Get);
        else if(rData3[0]=='D' && rData3[1]=='v' && rData3[2]=='2') // �ٶȻ�D
            PID_para(2,3,data_Get);
//      else if((rData3[0]=='S' && rData3[1]=='p') && rData3[2]=='e') //Ŀ���ٶ�
//          PID_para(2,4,data_Get);
//      else if((rData3[0]=='P' && rData3[1]=='o') && rData3[2]=='s') //Ŀ��λ��
////            PID_para(4,4,data_Get);
//          set_x2 = data_Get;
}

