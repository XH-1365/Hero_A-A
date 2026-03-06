#ifndef _VOFA_H__
#define _VOFA_H__
#include "stm32f4xx_hal.h"
#define VOFA_TX_TAIL_NUM 4
#define VOFA_BUFFER_LENGTH 30

typedef enum Vofa_TX_CH_enum
{
    VOFA_TX_CURRENT,
    VOFA_TX_CURRENT_TAR,
    VOFA_TX_SPEED,
    VOFA_TX_SPEED_TAR,
    VOFA_TX_ANGLE,
    VOFA_TX_ANGLE_TAR,

    VOFA_TX_CH_SUM
} Vofa_TX_CH;

typedef enum Vofa_RX_CH_enum
{
    VOFA_RX_CURRENT_KP,
    VOFA_RX_CURRENT_KI,
    VOFA_RX_CURRENT_KD,
    VOFA_RX_CURRENT_SW,
    VOFA_RX_CURRENT_TAR,

    VOFA_RX_SPEED_KP,
    VOFA_RX_SPEED_KI,
    VOFA_RX_SPEED_KD,
    VOFA_RX_SPEED_SW,
    VOFA_RX_SPEED_TAR,

    VOFA_RX_ANGLE_KP,
    VOFA_RX_ANGLE_KI,
    VOFA_RX_ANGLE_KD,
    VOFA_RX_ANGLE_KF,
    VOFA_RX_ANGLE_SW,
    VOFA_RX_ANGLE_TAR,

    VOFA_RX_CH_SUM
} VOFA_RX_CH;

#pragma pack(1) // 指定结构体按照1字节对齐

typedef struct
{
    float Value[VOFA_TX_CH_SUM];
    unsigned char Tail[VOFA_TX_TAIL_NUM];
} TX_Struct;

typedef struct
{
    char *Head;  // 匹配回传的对应的字符串头
    float Value; // 字符串后跟着的数值转换成的数字
} RX_Data;

typedef struct
{
    uint8_t Flag;   // vofa接收状态机标志位
    uint8_t Count;  // vofa接收状态机标志位
    uint8_t Status; // 接收状态，表示接收到了一段数据
    RX_Data Data[VOFA_RX_CH_SUM];
    char Buffer[VOFA_BUFFER_LENGTH];
    uint16_t Buffer_Size; // 接收缓存接收到的字节数
} RX_Struct;

typedef struct
{
    TX_Struct TX_JustFloat;
    RX_Struct RX_FireWater;
} Vofa_Struct;

#pragma pack() // 取消结构体对齐

extern Vofa_Struct Vofa_Data;

void Vofa_Init(void);
void Vofa_Set_TX_Value(Vofa_TX_CH CHANNEL, float Fdata);
float Vofa_Get_RX_Value(VOFA_RX_CH CHANNEL);
void Vofa_Transmit(void);
void Vofa_Receive_Enabled(uint16_t Size);
void Vofa_Receive_Handle(void);
void Vofa_Timing_Handle(void);
void Vofa_print(char *fmt, ...);
#endif
