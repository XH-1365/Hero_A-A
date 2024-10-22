/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-14 15:54:09
 * @LastEditors: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @LastEditTime: 2024-08-14 16:06:50
 * @FilePath: \RM_Template\bsp\bsp_usart.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp_usart.h"

uint8_t USART6_RX_Buffer[100];
uint8_t USART6_TX_Buffer[16]="Hello World!!\r\n";

void BSP_USART_Init(void)
{
  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)USART6_TX_Buffer, sizeof(USART6_TX_Buffer)-1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, USART6_RX_Buffer, sizeof(USART6_RX_Buffer)); // 串口空闲中断接收DMA
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);               //关闭DMA接收一半中断
}
