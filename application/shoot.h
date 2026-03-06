/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 13:32:52
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-25 17:02:39
 * @FilePath: \RM_Template\application\shoot.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _SHOOT_H__
#define _SHOOT_H__
#include "RM_typedef.h"
#include "pid.h"

#define DIAL GEAR_TEETH_NUM 68
#define DIAL_MOTOR_GEAR_TEETH_NUM 30
#define SHOOT_OPEN_LASER() HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET)
#define SHOOT_CLOSE_LASER() HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET)

typedef enum __Shoot_Mode_enum
{
  SHOOT_STOP_FIRE,  // 停止模式
   SHOOT_BURST_FIRE, // 连发模式
  SHOOT_SINGE_FIRE, // 单发模式
 
} Shoot_Mode_enum;

/// @brief 摩擦轮结构体声明
typedef struct __Shoot_Friction_Wheel_Struct
{
  float Speed_Tar;
  pid_controler Pid_Speed;
} Shoot_Friction_Wheel_Struct;

/// @brief 供弹电机结构体声明
typedef struct __Shoot_Feeding_Motor_Struct
{
  float Speed_Tar;
  pid_controler Pid_Speed;

  uint8_t Angle_Count; // 角度环周期
  float Angle_Tar;     // 航向轴角度目标值
  pid_controler Pid_Angle;

  float Speed_Limit;  // 速度限定值
  float Tprque_Limit; // 力矩限定值

} Shoot_Feeding_Motor_Struct;

/// @brief 热量控制结构体声明
typedef struct __Shoot_Heat_Control_Struct
{
  float Heat;
  float Heat_Tar;
  pid_controler Pid_Heat;

} Shoot_Heat_Control_Struct;

/// @brief 发射机结构体声明
typedef struct __Shoot_Struct
{
  uint8_t Flag;
  uint16_t Count;

  uint8_t Control_Flag;
  uint16_t Control_Count;

  uint8_t Heat_Ctrl_Flag;
  uint8_t Heat_Ctrl_Count;


  uint8_t DM_Control_Flag;
  uint8_t DM_Enable_Flag;
  uint16_t DM_Control_Count;

  uint16_t Vofa_Count;

  Shoot_Mode_enum Mode;

  Shoot_Feeding_Motor_Struct Feeding_Motor; // 弹丸拨盘电机
  Shoot_Friction_Wheel_Struct Right_Wheel;  // 头正对自己右电机摩擦轮
  Shoot_Friction_Wheel_Struct Left_Wheel;   // 头正对自己左电机摩擦轮
  Shoot_Heat_Control_Struct Heat_Control;
} Shoot_Struct;

void Shoot_Enable_DM(void);

void Shoot_Init(void);
void Shoot_Timing_Handle(void);
void Shoot_Task(void);
void Shoot_Set_Mode(Shoot_Mode_enum Mode);
void Shoot_Set_Fire_Add_Speed(float Firing_Add_Speed);
uint8_t Shoot_Get_Mode(void);
#endif
