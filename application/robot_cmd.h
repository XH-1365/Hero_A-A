/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 14:33:08
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-03-22 14:36:55
 * @FilePath: \RM_Template\application\robot_cmd.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _ROBOT_CMD_H__
#define _ROBOT_CMD_H__
#include "RM_typedef.h"
#pragma pack(1) // 指定结构体按照1字节对齐
typedef struct __Chassis_Control_Struct
{
    int16_t X_Speed;
    int16_t Y_Speed;
    uint8_t Mode;
    uint16_t DTOF_Distance; // 激光传感器距离 单位：mm
    uint8_t Fire_Flag : 4;
    uint8_t Power_Mode : 4;//为0为普通模式，为1为超级模式（在正常底盘功率下增加20w）
} Chassis_Control_Struct; //
#pragma pack()            // 指定结构体按照1字节对齐

void Robot_Cmd_Init(void);
void Robot_Cmd_Timing_Handle(void);
void Robot_Cmd_Task(void);

#endif
