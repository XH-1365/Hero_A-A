#ifndef _BSP_USART_H__
#define _BSP_USART_H__
#include "usart.h"

#define huart8_control 0 //0 mini ,1 vofa


extern uint8_t USART6_RX_Buffer[100];

void BSP_USART_Init(void);
void BSP_USART6_Config(void);
#endif
