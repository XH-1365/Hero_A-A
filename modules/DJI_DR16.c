/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-14 15:28:32
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-09-07 21:13:08
 * @FilePath: \RM_Template\modules\DJI_DR16.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "DJI_DR16.h"
#include "bsp_can.h"
#include "dma.h"
#include "usart.h"

uint8_t DJI_DR16_Buffer[36];
DJI_DR16_Struct DJI_DR16_Data = {0};
void DJI_DR16_Init(void)
{
    // HAL_UART_Receive_DMA(&huart1,DJI_DR16_Buffer,18);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, DJI_DR16_Buffer, sizeof(DJI_DR16_Buffer)); // 串口空闲中断接收DMA
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);                                // 关闭DMA接收一半中断
}

uint8_t *DJI_DR16_Get_Buffer(void)
{
    return DJI_DR16_Buffer;
}

void DJI_DR16_Data_Process(uint8_t *Buffer, uint8_t size)
{
    if (Buffer == NULL)
    {
        return;
    }

    if (size != DJI_DR16_DATA_LENGHT)
    {
        return;
    }

    DJI_DR16_Data.RC.CH0 = ((int16_t)Buffer[0] | ((int16_t)Buffer[1] << 8)) & 0x07FF;
    DJI_DR16_Data.RC.CH1 = (((int16_t)Buffer[1] >> 3) | ((int16_t)Buffer[2] << 5)) & 0x07FF;
    DJI_DR16_Data.RC.CH2 = (((int16_t)Buffer[2] >> 6) | ((int16_t)Buffer[3] << 2) |
                            ((int16_t)Buffer[4] << 10)) &
                           0x07FF;
    DJI_DR16_Data.RC.CH3 = (((int16_t)Buffer[4] >> 1) | ((int16_t)Buffer[5] << 7)) & 0x07FF;
    DJI_DR16_Data.RC.Wheel = (((int16_t)Buffer[16]) | ((int16_t)Buffer[17] << 8));
    DJI_DR16_Data.RC.S1 = ((Buffer[5] >> 4) & 0x000C) >> 2;
    DJI_DR16_Data.RC.S2 = ((Buffer[5] >> 4) & 0x0003);

    DJI_DR16_Data.RC_Value.CH0 = NORMALIZE(DJI_DR16_Data.RC.CH0, 364, 1684) ; // 数据归一化为（-1~1）
    DJI_DR16_Data.RC_Value.CH1 = NORMALIZE(DJI_DR16_Data.RC.CH1, 364, 1684);             // 数据归一化为（-1~1）
    DJI_DR16_Data.RC_Value.CH2 = NORMALIZE(DJI_DR16_Data.RC.CH2, 364, 1684);             // 数据归一化为（-1~1）
    DJI_DR16_Data.RC_Value.CH3 = NORMALIZE(DJI_DR16_Data.RC.CH3, 364, 1684);             // 数据归一化为（-1~1）
    DJI_DR16_Data.RC_Value.Wheel = NORMALIZE(DJI_DR16_Data.RC.Wheel, 364, 1684);         // 数据归一化为（-1~1）
    DJI_DR16_Data.RC_Value.S1 = DJI_DR16_Data.RC.S1;
    DJI_DR16_Data.RC_Value.S2 = DJI_DR16_Data.RC.S2;

    DJI_DR16_Data.Mouse.X = ((int16_t)Buffer[6]) | ((int16_t)Buffer[7] << 8);
    DJI_DR16_Data.Mouse.Y = ((int16_t)Buffer[8]) | ((int16_t)Buffer[9] << 8);
    DJI_DR16_Data.Mouse.Z = ((int16_t)Buffer[10]) | ((int16_t)Buffer[11] << 8);

    DJI_DR16_Data.Mouse.Key_L = Buffer[12];
    DJI_DR16_Data.Mouse.Key_R = Buffer[13];

    DJI_DR16_Data.Keyboard.Data = ((int16_t)Buffer[14]) | ((int16_t)Buffer[15] << 8);
}

/// @brief 处理DR16延时相关操作需放到定时器1ms中断里
/// @param
void DR16_Timing_Handle(void)
{

    if (DJI_DR16_Data.Count > 0)
    {
        DJI_DR16_Data.Count--;
    }


}



/// @brief 处理接收相关操作需放到串口接收中断里
/// @param
void DR16_RX_Handle(uint8_t *Buffer, uint8_t size)
{



    DJI_DR16_Data_Process(Buffer, size);
    DJI_DR16_Init();
    DJI_DR16_Data.Count = 50;
}
uint8_t DR16_Get_Count(void)
{
	
	
	return DJI_DR16_Data.Count;
}

