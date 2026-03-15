/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2025-03-08 19:59:12
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-08 20:04:24
 * @FilePath: \RM_Hero_Down_Board\modules\super_cap.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __SUPER_CAP_H__
#define __SUPER_CAP_H__
#include "RM_typedef.h"
// 超级电容接收ID
#define SUPER_CAP_RX_STD_ID 0x211

// 超级电容发送ID
#define SUPER_CAP_TX_STD_ID 0x210

#pragma pack(1) // 指定结构体按照1字节对齐

typedef union
{
    struct __Super_Cap_Struct_Data
    {
        int16_t Input_Voltage; // 输入电压
        int16_t Cap_Voltage;   // 电容电压
        int16_t Input_Current; // 输入电流
        int16_t Target_Power;  // 目标功率 底盘的限定最大功率
    } Data;
    uint8_t Buffer[8];
} Super_Cap_Ret_Data_Union; // 电机的返回数据

typedef struct __Super_Cap_Struct
{
        uint16_t Time_out;
    
    CAN_HandleTypeDef *hcan; // 使用的CAN口，hcan1或者hcan2
    int16_t TX_STD_ID;
    int16_t RX_STD_ID;

    Super_Cap_Ret_Data_Union Ret_Data; // 接收到的超电数据
} Super_Cap_Struct;

#pragma pack() // 取消结构体对齐
extern Super_Cap_Struct Super_Cap;

char Super_Cap_Get_Data(Super_Cap_Struct *Super_Cap_x, CAN_RxHeaderTypeDef *RxHeader, uint8_t *CAN_RX_Buffer);

void Super_Cap_init(void);
void Super_Cap_Timing_Handle();
void Super_Cap_Set_Power(Super_Cap_Struct *Super_Cap_x, uint16_t Target_Power);

#endif
