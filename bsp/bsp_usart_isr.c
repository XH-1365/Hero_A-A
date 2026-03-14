/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-08-14 15:54:24
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-14 14:35:45
 * @FilePath: \RM_Template\bsp\bsp_usart_isr.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp_usart.h"
#include "DJI_DR16.h"
#include "vofa.h"
#include "CH104_IMU_USART.H"
#include "rm_referee.h"
/* 串口空闲中断回调函数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart8) //  将上位机发来的数据完整的返回回去
  {
    Vofa_Receive_Enabled(Size);
  }
  else if (huart == &huart6) //  将上位机发来的数据完整的返回 回去
  {
    CH104_IMU_USART_RX_Handle(CH104_IMU_USART_Get_Buffer(), Size);
  }
  else if (huart == &huart1) //  将上位机发来的数据完整的返回 回去
  {
    DR16_RX_Handle(DJI_DR16_Get_Buffer(), Size);
    //    sprintf(str, "CH0:%u,CH1:%u,CH2:%u,CH3:%u,Wheel:%u,S1:%u,S2:%u\r\n",
    //            DJI_DR16_Data.RC.CH0,
    //            DJI_DR16_Data.RC.CH1,
    //            DJI_DR16_Data.RC.CH2,
    //            DJI_DR16_Data.RC.CH3,
    //            DJI_DR16_Data.RC.Wheel,
    //            DJI_DR16_Data.RC.S1,
    //            DJI_DR16_Data.RC.S2);

    // HAL_UART_Transmit_DMA(&huart6,DJI_DR16_Get_Buffer(),Size);
    //  HAL_UART_Transmit_DMA(&huart6,(uint8_t*)str, strlen(str));
  }
  else if (huart == &huart3) //  接收裁判系统发送的数据
  {
    Referee_RX_Handle(Referee_Get_Buffer(), Size);
  }
}
