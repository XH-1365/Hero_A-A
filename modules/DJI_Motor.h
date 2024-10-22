/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-06 14:10:56
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-22 15:19:13
 * @FilePath: \DJI_Motor\Inc\DJI_Motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _DJI_Motor_H__
#define _DJI_Motor_H__
#include "RM_typedef.h"

typedef enum __DJI_Motor_TX_ID_enum
{
    DJI_Motor_TX_1_4,
    DJI_Motor_TX_5_8,
    DJI_Motor_TX_SUM
} DJI_Motor_TX_ID_enum;
// 电机组1接收ID标识
typedef enum __DJI_Motor_Group1_RX_ID_enum
{
    DJI_MGRP1_M3508_RX_1,
    DJI_MGRP1_M3508_RX_2,
    DJI_MGRP1_M3508_RX_3,
    DJI_MGRP1_M3508_RX_4,
    DJI_MGRP1_RX_SUM
} DJI_Motor_Group1_RX_ID_enum;

// 电机组2接收ID标识
typedef enum __DJI_Motor_Group2_RX_ID_enum
{
    DJI_MGRP2_GM6020_RX_5, // 航向轴电机,Yaw轴
    DJI_MGRP2_M2006_RX_6, // 俯仰轴电机,Pich轴
    DJI_MGRP2_RX_SUM
} DJI_Motor_Group2_RX_ID_enum;


typedef enum __DJI_Motor_Group_enum
{
    DJI_MGRP1, // 底盘电机组
    DJI_MGRP2, // 云台电机组
    DJI_MGRP_SUM
} DJI_Motor_Group_enum;
#pragma pack(1) // 指定结构体按照1字节对齐

typedef struct __DJI_Motor_Angle_Sum_Struct
{
    float Angle_Sum_Value;
    float Angle_Turn;
    float Angle_Now;
    float Angle_Last;
} DJI_Motor_Angle_Sum_Struct;

typedef struct __DJI_Motor_RetData_Struct
{
    float Angle;            // 转子机械角度值范围: 0~8191 (对应转子机械角度为 0~360°）归1化后为0~1
    DJI_Motor_Angle_Sum_Struct Angle_Sum_Process;
    int16_t Speed;          // 转子转速值的单位为：RPM
    float Current;          // 控制电流范围—16384~0~+16384 归1化后为-1~1
    uint8_t Temperature;    // 电机温度的单位为：℃
    uint8_t RX_Flag;        // 电机温度的单位为：℃
} DJI_Motor_RetData_Struct; // 电机的返回数据


typedef struct __DJI_Motor_TarData_Struct
{
    float Angle; // 转子机械角度值范围: 0~8191 (对应转子机械角度为 0~360°）归1化后为0~1
   
    int16_t Speed;          // 转子转速值的单位为：RPM
    float Current;          // 控制电流范围—16384~0~+16384 归1化后为-1~1
    uint8_t Temperature;    // 电机温度的单位为：℃
} DJI_Motor_TarData_Struct; // 电机的目标数据

typedef struct __DJI_Motor_Data_Struct
{

    DJI_Motor_RetData_Struct Ret; // 电机返回的数据
    // DJI_Motor_TarData_Struct Tar; // 电机的目标数据

} DJI_Motor_Data_Struct; // 电机的数据 包含电机目标数据和返回数据

typedef struct __DJI_Motor_Group_Struct
{
    uint16_t Flag;
    uint16_t Count;
    CAN_HandleTypeDef *hcan;             // 使用的CAN口，hcan1或者hcan2
    int16_t TX_STD_ID[DJI_Motor_TX_SUM]; // CAN发送数据帧ID
    int16_t RX_STD_ID;                   // CAN接收数据帧ID偏移，比如电机1的反馈报文为RX_STD_ID+1
    DJI_Motor_Data_Struct *Data;         // 电机的数据 包含电机目标数据和返回数据
    const uint8_t Data_Length;           // 电机数据长度
} DJI_Motor_Group_Struct;                // DJI电机组数据
#pragma pack()                           // 取消结构体对齐

extern DJI_Motor_Group_Struct Motor_Group[DJI_MGRP_SUM];

void DJI_Motor_Init(void);
void DJI_Motor_Set_Current(DJI_Motor_Group_enum DJI_MGRPx, DJI_Motor_TX_ID_enum TX_ID, int motor1, int motor2, int motor3, int motor4);
void DJI_Motor_Set_Current_Value(DJI_Motor_Group_enum DJI_MGRPx, DJI_Motor_TX_ID_enum TX_ID, float Motor1, float Motor2, float Motor3, float Motor4);
void DJI_Motor_Data_Init(void);
char DJI_Motor_Get_Data(DJI_Motor_Group_enum DJI_MGRPx, CAN_RxHeaderTypeDef *RxHeader, uint8_t *Motor_RX_Buffer);
void DJI_Motor_Timing_Handle(void);
void DJI_Motor_Handle(void);
#endif
