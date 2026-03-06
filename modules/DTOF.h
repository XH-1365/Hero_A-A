#ifndef _DTOF_H__
#define _DTOF_H__
#include "RM_typedef.h"
#include "daemon.h"
// DTOF IIC 设备地址
#define DTOF_IIC_DEVICE_ADDRESS 0x51
// DTOF IIC 写地址
#define DTOF_IIC_WRITE_ADDRESS 0xA2
// DTOF IIC 读地址
#define DTOF_IIC_READ_ADDRESS 0xA3
// DTOF IIC 内部寄存器地址
typedef enum __DTOF_REG_enum
{
  DTOF_DISTANCE_8BIT_H = 0x00, // 测量距离高8位(只读) 距离使用2byte表示（单位mm）
  DTOF_DISTANCE_8BIT_L,        // 测量距离低8位(只读) 距离使用2byte表示（单位mm）
  DTOF_ENABLE_FLAG,            // 使能测距标志位(读写) 写1开始测量，激光开启，距离数据开始刷新，写0结束测量激光关闭。
  DTOF_REG_TEST,               // 测试寄存器 (只读)默认值0x3B
} DTOF_REG_enum;

typedef struct __DTOF_Struct
{
  uint8_t Flag;
  uint16_t Count;
  Daemon_Struct *Daemon;

  uint8_t TX_Buffer[2];
  uint8_t RX_Buffer[2];
  uint16_t Distance;
  uint8_t Enable_Flag;
} DTOF_Struct;
extern DTOF_Struct DTOF;
void DTOF_Init(void);
void DTOF_Test_Handle(void);
void DTOF_IIC_RX_Handle(void);
void DTOF_IIC_Handle(void);
void DTOF_IIC_Timing_Handle(void);
#endif // _TOF_H__
