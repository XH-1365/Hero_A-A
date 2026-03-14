/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-04 20:19:53
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-09-26 15:47:52
 * @FilePath: \MDK-ARMf:\RM_Program\RM_Code_Program\External_Module\DJI_DR16\Inc\DJI_DR16.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _DJI_DR16_H__
#define _DJI_DR16_H__
#include "RM_typedef.h"

#define DJI_DR16_DATA_LENGHT 18

#pragma pack(1) // 指定结构体按照1字节对齐
typedef struct __DJI_DR16_Struct
{
    struct __RC_Struct
    {
        uint16_t CH0;
        uint16_t CH1;
        uint16_t CH2;
        uint16_t CH3;
        uint16_t Wheel;
        uint8_t S1;
        uint8_t S2;
    } RC;

    struct __RC_Value_Struct
    {
        float CH0;
        float CH1;
        float CH2; // 左摇杆X轴
        float CH3;
        float Wheel;
        uint8_t S1;
        uint8_t S2;
    } RC_Value;

    struct __Mouse_Struct
    {
        int16_t X;
        int16_t Y;
        int16_t Z;
        uint8_t Key_L;
        uint8_t Key_R;
    } Mouse;

    union __Keyboard_Union
    {
        struct __Keyboard_Struct
        {
            uint8_t W : 1;
            uint8_t S : 1;
            uint8_t A : 1;
            uint8_t D : 1;
            uint8_t Q : 1;
            uint8_t E : 1;
            uint8_t Shift : 1;
            uint8_t Ctrl : 1;
            uint8_t None;
        } Key;

        uint16_t Data;

    } Keyboard;

    uint8_t None;
    uint8_t Flag;
    uint16_t Count;
} DJI_DR16_Struct;
#pragma pack() // 取消结构体对齐

extern DJI_DR16_Struct DJI_DR16_Data;
void DJI_DR16_Init(void);
uint8_t *DJI_DR16_Get_Buffer(void);
void DJI_DR16_Data_Process(uint8_t *Buffer, uint8_t size);
void DR16_Timing_Handle(void);
void DR16_RX_Handle(uint8_t *Buffer, uint8_t size);
uint8_t DR16_Get_Count(void);
#endif
