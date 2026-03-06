/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-10 22:09:30
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-05 20:53:23
 * @FilePath: \RM_Template\modules\CH104_IMU_USART.C
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "CH104_IMU_USART.H"
#include "usart.h"
#include <string.h>
#include "bsp_can.h"
#include <stdio.h>
#include "vofa.h"
CH104_IMU_USART_Data_Union CH104_IMU_USART_Buffer;
CH104_IMU_USART_Struct CH104_IMU_USART;
void CH104_IMU_USART_Send_Cmd(char *Cmd)
{
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Cmd, strlen(Cmd));
}

void CH104_IMU_USART_Set_BAUD(uint32_t Baud)
{
    char cmd[50];
    sprintf(cmd, "AT+BAUD=%lu\r\n", Baud);
    CH104_IMU_USART_Send_Cmd(cmd);
}

void CH104_IMU_USART_Set_Rat(uint16_t rat)
{
    char cmd[30];
    sprintf(cmd, "AT+ODR=%u\r\n", rat);
    CH104_IMU_USART_Send_Cmd(cmd);
}

void CH104_IMU_USART_RST(void)
{
    CH104_IMU_USART_Send_Cmd((char*)"AT+RST\r\n");
}
void CH104_IMU_USART_Init(void)
{
//    CH104_IMU_USART_Set_BAUD(921600);
    CH104_IMU_USART_Set_Rat(400);
//    CH104_IMU_USART_RST();

    //   HAL_UART_Transmit_DMA(&huart6, (uint8_t *)(&(Vofa_Data.TX_JustFloat)), sizeof(TX_Struct));
    // 串口空闲中断接收DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)(CH104_IMU_USART_Buffer.Buffer), CH104_IMU_USART_BUFFER_LENGTH);
    // 关闭DMA接收完一半中断
    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

void CH104_IMU_USART_Timing_Handle(void)
{
    if (CH104_IMU_USART.Count > 0)
    {
        CH104_IMU_USART.Count--;
    }
    if (CH104_IMU_USART.RX_Count > 0)
    {
        CH104_IMU_USART.RX_Count--;
    }
}

static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j = 0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currect_crc = crc;
}

uint8_t *CH104_IMU_USART_Get_Buffer(void)
{
    return (CH104_IMU_USART_Buffer.Buffer);
}

float CH104_IMU_USART_Get_Angle_Sum(CH104_IMU_USART_Angle_Sum_Struct *Angle_Sum_Process, float Angle_Value, float Max, float Min)
{

    float Jump_Value = Max - Min;

    Angle_Sum_Process->Angle_Last = Angle_Sum_Process->Angle_Now;
    Angle_Sum_Process->Angle_Now = Angle_Value;
    if ((Angle_Sum_Process->Angle_Last - Angle_Sum_Process->Angle_Now) > (Jump_Value - 0.2f))
    {
        (Angle_Sum_Process->Angle_Turn) += Jump_Value;
    }
    else if ((Angle_Sum_Process->Angle_Last - Angle_Sum_Process->Angle_Now) < -(Jump_Value - 0.2f))
    {
        (Angle_Sum_Process->Angle_Turn) -= Jump_Value;
    }
    Angle_Sum_Process->Angle_Sum_Value = (Angle_Sum_Process->Angle_Turn) + (Angle_Sum_Process->Angle_Now);
    return Angle_Sum_Process->Angle_Sum_Value;
}

void CH104_IMU_USART_Get_Data(uint8_t *Buffer, uint16_t size)
{
    uint16_t CRC_Data = 0;
    uint16_t payload_len = 0;
    payload_len = Buffer[2] + (Buffer[3] << 8);

    crc16_update(&CRC_Data, Buffer, 4);
    // CRC_Data1 = CRC_Data;

    crc16_update(&CRC_Data, Buffer + 6, payload_len);
    // CRC_Data2 = CRC_Data;
    if (CRC_Data != CH104_IMU_USART_Buffer.Data.CRC_Value)
    {
        return;
    }

    memcpy(CH104_IMU_USART.Ret_Data.Buffer, Buffer, sizeof(CH104_IMU_USART_Data_Union));
    CH104_IMU_USART_Get_Angle_Sum(&(CH104_IMU_USART.Euler_Angle_Sum.Pitch), CH104_IMU_USART.Ret_Data.Data.Euler_Angles.Pitch / 180.0f, 0.5f, -0.5f);
    CH104_IMU_USART_Get_Angle_Sum(&(CH104_IMU_USART.Euler_Angle_Sum.Yaw), CH104_IMU_USART.Ret_Data.Data.Euler_Angles.Yaw / 360.0f, 0.5f, -0.5f);
    CH104_IMU_USART_Get_Angle_Sum(&(CH104_IMU_USART.Euler_Angle_Sum.Roll), CH104_IMU_USART.Ret_Data.Data.Euler_Angles.Roll / 360.0f, 0.5f, -0.5f);
    CH104_IMU_USART.RX_Count = 100;
}

void CH104_IMU_USART_RX_Handle(uint8_t *Buffer, uint8_t size)
{
    CH104_IMU_USART_Get_Data(Buffer, size);
    CH104_IMU_USART_Init();
}

void CH104_IMU_USART_Handle(void)
{
    if (CH104_IMU_USART.Count > 0)
    {
        return;
    }

    switch (CH104_IMU_USART.Flag)
    {
    case 0:
        CH104_IMU_USART.Flag = 1;
        CH104_IMU_USART.Count = 100;
        break;

    case 1:
        if (CH104_IMU_USART.RX_Count > 0)
        {

            CH104_IMU_USART.State = CH104_IMU_USART_CONNECT;
        }
        else if (CH104_IMU_USART.RX_Count == 0)
        {
            CH104_IMU_USART.State = CH104_IMU_USART_DICONNECT;
            CH104_IMU_USART.Flag = 2;
        }

        CH104_IMU_USART.Count = 100;

        break;

    case 2:
        CH104_IMU_USART_Init();
        CH104_IMU_USART.Flag = 1;

        break;
    }

    //    Vofa_print("State:%d,RX_Count:%u\r\n", CH104_IMU_USART.State, CH104_IMU_USART.RX_Count);
}
