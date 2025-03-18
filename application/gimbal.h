/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-08-22 21:01:10
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-23 20:32:53
 * @FilePath: \RM_Template\application\gimbal.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _GIMBAL_H__
#define _GIMBAL_H__
#include "RM_typedef.h"
float Screw_Pitch_Length_To_Angle_Calculate(float Pitch_Angle);
void Gimbal_Init(void);

void Gimbal_Set_Mode(uint8_t Mode);
// void Gimbal_Control(float Yaw_Angle,float Pitch_Angle);

/// @brief 设置增量角度，输入都是增量的角度，每运行一次增加一次，角度可以是负数
void Gimbal_Add_Angle(float Yaw_Add_Angle, float Pitch_Add_Angle);



void Gimbal_Timing_Handle(void);
void Gimbal_Task(void);
#endif
