#include "vofa.h"
#include "usart.h"
#include "gpio.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "bsp_usart.h"
#define VOFA_BUFFER_SIZE 100
const uint8_t TX_JustFloat_Tail[4] = {0x00, 0x00, 0x80, 0x7f};

Vofa_Struct Vofa_Data = {0};
uint8_t Vofa_Send_Buffer[VOFA_BUFFER_SIZE];
void Vofa_Init(void)
{
  Vofa_Data.RX_FireWater.Flag = 0;
  Vofa_Data.RX_FireWater.Status = 0;
  Vofa_Data.RX_FireWater.Data[VOFA_RX_CURRENT_KP].Head = "current_kp:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_CURRENT_KI].Head = "current_ki:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_CURRENT_KD].Head = "current_kd:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_CURRENT_SW].Head = "current_sw:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_CURRENT_TAR].Head = "current_tar:";

  Vofa_Data.RX_FireWater.Data[VOFA_RX_SPEED_KP].Head = "speed_kp:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_SPEED_KI].Head = "speed_ki:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_SPEED_KD].Head = "speed_kd:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_SPEED_SW].Head = "speed_sw:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_SPEED_TAR].Head = "speed_tar:";

  Vofa_Data.RX_FireWater.Data[VOFA_RX_ANGLE_KP].Head = "angle_kp:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_ANGLE_KI].Head = "angle_ki:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_ANGLE_KD].Head = "angle_kd:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_ANGLE_KF].Head = "angle_kf:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_ANGLE_SW].Head = "angle_sw:";
  Vofa_Data.RX_FireWater.Data[VOFA_RX_ANGLE_TAR].Head = "angle_tar:";

  memcpy((uint8_t *)(&(Vofa_Data.TX_JustFloat.Tail)), TX_JustFloat_Tail, sizeof(TX_JustFloat_Tail));
  HAL_UART_Transmit_DMA(&huart8, (uint8_t *)(&(Vofa_Data.TX_JustFloat)), sizeof(TX_Struct));
  // 串口空闲中断接收DMA
  HAL_UARTEx_ReceiveToIdle_DMA(&huart8, (uint8_t *)(Vofa_Data.RX_FireWater.Buffer), VOFA_BUFFER_LENGTH);
  // 关闭DMA接收完一半中断
  __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
}

void Vofa_Set_TX_Value(Vofa_TX_CH CHANNEL, float FData)
{
  Vofa_Data.TX_JustFloat.Value[CHANNEL] = FData;
}

float Vofa_Get_RX_Value(VOFA_RX_CH CHANNEL)
{
  if (Vofa_Data.RX_FireWater.Data[CHANNEL].Head != NULL)
  {
    return Vofa_Data.RX_FireWater.Data[CHANNEL].Value;
  }
  return 0;
}

/// @brief 此函数要放在串口中断内
/// @param size  需将接收到的字节数传入
void Vofa_Receive_Enabled(uint16_t Size)
{
  Vofa_Data.RX_FireWater.Status = 1;
  Vofa_Data.RX_FireWater.Buffer[Size + 1] = '\0';
  Vofa_Data.RX_FireWater.Buffer_Size = Size;
}

char *Find_Para_Str(const char *Haystack, const char *Needle, uint16_t Offset)
{
  char *Buffer = 0;
  if (Needle == NULL)
  {
    Buffer = Offset + (char *)Haystack;
    return Buffer;
  }
  Buffer = strstr(Haystack, Needle);
  if (Buffer != NULL)
  {
    Buffer += Offset + strlen(Needle);
    return Buffer;
  }

  // 未找到匹配
  return NULL;
}

char *Find_Para_Number(const char *Haystack, const char *Needle, uint16_t Offset, float *Num)
{
  char *Buffer = 0;
  char Str[15];
  uint8_t I = 0;

  Str[14] = '\0';
  Buffer = Find_Para_Str(Haystack, Needle, Offset);
  if (Buffer == NULL)
  {
    return NULL;
  }
  for (I = 0; I < 5; I++)
  {
    if ((Buffer[I] >= '0') && (Buffer[I] <= '9') || Buffer[I] == '.' || Buffer[I] == '-')
    {
      Str[I] = Buffer[I];
    }
    else
    {
      break;
    }
  }
  Str[I] = '\0';
  *Num = atof(Str);
  return Buffer + I;
}

static char *Vofa_Set_RX_Value(void)
{
  char *Buffer = NULL;
  char *Ret_Buffer = NULL;
  uint8_t I = 0;

  for (I = 0; I < VOFA_RX_CH_SUM; I++)
  {
    Buffer = Find_Para_Number(Vofa_Data.RX_FireWater.Buffer,
                              Vofa_Data.RX_FireWater.Data[I].Head,
                              0,
                              &(Vofa_Data.RX_FireWater.Data[I].Value));
    if (Buffer != NULL)
    {
      Ret_Buffer = Buffer;
    }
  }
  return Ret_Buffer;
}

void Vofa_Timing_Handle(void)
{
  if (Vofa_Data.RX_FireWater.Count > 0)
  {
    Vofa_Data.RX_FireWater.Count--;
  }
}

void Vofa_Receive_Handle(void)
{
  char *Buffer = 0;

  if (Vofa_Data.RX_FireWater.Count != 0)
  {
    return;
  }
  Vofa_Data.RX_FireWater.Count = 10;
  switch (Vofa_Data.RX_FireWater.Flag)
  {
  case 0:
    if (Vofa_Data.RX_FireWater.Status == 1)
    {
      Buffer = Vofa_Set_RX_Value();
      if (Buffer != NULL)
      {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3,
                          Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? GPIO_PIN_RESET : GPIO_PIN_SET);

        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2,
                          Vofa_Get_RX_Value(VOFA_RX_ANGLE_SW) ? GPIO_PIN_RESET : GPIO_PIN_SET);
      }

      Vofa_Data.RX_FireWater.Status = 0;

      Vofa_Data.RX_FireWater.Flag = 1;
    }
    break;
  case 1:
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, (uint8_t *)(Vofa_Data.RX_FireWater.Buffer), VOFA_BUFFER_LENGTH);
    // 串口空闲中断接收DMA
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
    Vofa_Data.RX_FireWater.Flag = 0;
    break;
  }
}

void Vofa_Transmit(void)
{
    #if huart8_control
    HAL_UART_Transmit_DMA(&huart8, (uint8_t *)(&(Vofa_Data.TX_JustFloat)), sizeof(TX_Struct));
    #endif
}

void Vofa_print(char *fmt, ...)
{
  va_list args;                                                                                    // 记录输入的参数
  uint16_t length;                                                                                 // 用于记录字符串长度
  va_start(args, fmt);                                                                             // 对字符串进行转换
  length = (uint16_t)vsnprintf((char *)Vofa_Send_Buffer, sizeof(Vofa_Send_Buffer) - 1, fmt, args); // 开始对DMA_Send_Buffer赋值，并返回字符串长度
  if (length > VOFA_BUFFER_SIZE - 1)
  {
    length = VOFA_BUFFER_SIZE - 1; // 防止超出数组界限
  }

  HAL_UART_Transmit_DMA(&huart8, Vofa_Send_Buffer, length);
}
