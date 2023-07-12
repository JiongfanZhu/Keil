/*
 * my_uart.c
 *
 *  Created on: 2023年6月6日
 *      Author: LENOVO
 */

#include "my_uart.h"

float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
    for(int i=0;i<200;i++) // 查找等号和感叹号的位置
    {
        if(rData3[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(rData3[i] == ' ')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(rData3[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
        data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 数据共4位
    {
        data_return = (rData3[data_Start_Num]-48) +
        (rData3[data_Start_Num+2]-48)*0.1f +
        (rData3[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (rData3[data_Start_Num]-48) +
        (rData3[data_Start_Num+2]-48)*0.1f +
        (rData3[data_Start_Num+3]-48)*0.01f +
        (rData3[data_Start_Num+4]-48)*0.001f;
    }
     else if(data_Num == 6) // 数据共6位
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
    float data_Get = Get_Data(); // 存放接收到的数据
    // printf("data=%.2f\r\n",data_Get);
        if(rData3[0]=='P' && rData3[1]=='t') // 角度P
            PID_para(3,1,data_Get);
        else if(rData3[0]=='I' && rData3[1]=='t') // 角度I
            PID_para(3,2,data_Get);
        else if(rData3[0]=='D' && rData3[1]=='t') // 角度D
            PID_para(3,3,data_Get);
        else if(rData3[0]=='P' && rData3[1]=='v' && rData3[2]=='1') // 速度环P
            PID_para(1,1,data_Get);
        else if(rData3[0]=='I' && rData3[1]=='v' && rData3[2]=='1') // 速度环I
            PID_para(1,2,data_Get);
        else if(rData3[0]=='D' && rData3[1]=='v' && rData3[2]=='1') // 速度环D
            PID_para(1,3,data_Get);
//      else if(rData3[0]=='S' && rData3[1]=='p' && rData3[2]=='e') //目标速度
//          PID_para(1,4,data_Get);
//      else if(rData3[0]=='P' && rData3[1]=='o' && rData3[2]=='s') //目标位置
////            PID_para(3,4,data_Get);
//          set_x1 = data_Get;


        else if(rData3[0]=='P' && rData3[1]=='b') // 截距P
            PID_para(4,1,data_Get);
        else if(rData3[0]=='I' && rData3[1]=='b') // 截距I
            PID_para(4,2,data_Get);
        else if(rData3[0]=='D' && rData3[1]=='b') // 截距D
            PID_para(4,3,data_Get);
        else if(rData3[0]=='P' && rData3[1]=='v' && rData3[2]=='2') // 速度环P
            PID_para(2,1,data_Get);
        else if(rData3[0]=='I' && rData3[1]=='v' && rData3[2]=='2') // 速度环I
            PID_para(2,2,data_Get);
        else if(rData3[0]=='D' && rData3[1]=='v' && rData3[2]=='2') // 速度环D
            PID_para(2,3,data_Get);
//      else if((rData3[0]=='S' && rData3[1]=='p') && rData3[2]=='e') //目标速度
//          PID_para(2,4,data_Get);
//      else if((rData3[0]=='P' && rData3[1]=='o') && rData3[2]=='s') //目标位置
////            PID_para(4,4,data_Get);
//          set_x2 = data_Get;
}

