/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-08-15 13:59:02
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-09-25 16:57:26
 * @FilePath: \RM_Template\application\chassis.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CHASSIS_H__
#define _CHASSIS_H__
#include "RM_typedef.h"

typedef enum __Chassis_Mode_enum
{
  CHASSIS_FOLLOW,//跟随模式
  CHASSIS_PEG_TOP//小陀螺模式
} Chassis_Mode_enum;

void Chassis_Init(void);

void Chassis_Set_Mode(Chassis_Mode_enum Mode);
void Chassis_Control(float X_Speed,float Y_Speed);


void Chassis_Timing_Handle(void);             
void Chassis_Task(void);
#endif
