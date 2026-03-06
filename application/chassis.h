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

// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)（前后两个轮中心的距离）
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)（左右两个轮中心的距离）
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60             // 轮子半径


typedef enum __Chassis_Mode_enum
{
  CHASSIS_FOLLOW=2,//跟随模式
  CHASSIS_PEG_TOP//小陀螺模式
} Chassis_Mode_enum;

void Chassis_Init(void);

void Chassis_Set_Mode(Chassis_Mode_enum Mode);
void Chassis_Control(float X_Speed,float Y_Speed);


void Chassis_Timing_Handle(void);             
void Chassis_Task(void);
#endif
