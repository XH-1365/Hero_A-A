/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 14:33:08
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-22 14:52:44
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
    uint8_t None1;
    uint8_t None2;
    uint8_t None3;
} Chassis_Control_Struct; //
#pragma pack()            // 指定结构体按照1字节对齐

void Robot_Cmd_Init(void);
void Robot_Cmd_Timing_Handle(void);
void Robot_Cmd_Task(void);

#endif
