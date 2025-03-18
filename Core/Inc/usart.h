/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2024-10-20 16:56:40
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-14 14:57:27
 * @FilePath: \RM_Hero_Down_Board\Core\Inc\usart.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.h
 * @brief   This file contains all the function prototypes for
 *          the usart.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  extern UART_HandleTypeDef huart8;

  extern UART_HandleTypeDef huart1;

  extern UART_HandleTypeDef huart3;

  extern UART_HandleTypeDef huart6;

  /* USER CODE BEGIN Private defines */
  extern DMA_HandleTypeDef hdma_usart1_rx;

  extern DMA_HandleTypeDef hdma_usart6_rx;
  extern DMA_HandleTypeDef hdma_usart6_tx;

  extern DMA_HandleTypeDef hdma_uart8_rx;
  extern DMA_HandleTypeDef hdma_uart8_tx;
  
  extern DMA_HandleTypeDef hdma_usart3_rx;
  extern DMA_HandleTypeDef hdma_usart3_tx;

  /* USER CODE END Private defines */

  void MX_UART8_Init(void);
  void MX_USART1_UART_Init(void);
  void MX_USART3_UART_Init(void);
  void MX_USART6_UART_Init(void);

  /* USER CODE BEGIN Prototypes */

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
