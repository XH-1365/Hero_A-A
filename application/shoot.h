/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 13:32:52
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-16 15:05:08
 * @FilePath: \RM_Template\application\shoot.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _SHOOT_H__
#define _SHOOT_H__
#include "RM_typedef.h"

typedef enum __Shoot_Mode_enum
{
  SHOOT_STOP_FIRE,//停止模式
  SHOOT_SINGE_FIRE,//单发模式
  SHOOT_BURST_FIRE,//连发模式
} Shoot_Mode_enum;

void Shoot_Init(void);
void Shoot_Timing_Handle(void);
void Shoot_Task(void);
void Shoot_Set_Mode(Shoot_Mode_enum Mode);
void Shoot_Set_Fire_Add_Speed(float Firing_Add_Speed);
#endif
