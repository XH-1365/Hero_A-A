/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-08-15 13:59:02
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-12 21:36:33
 * @FilePath: \RM_Template\application\chassis.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CHASSIS_H__
#define _CHASSIS_H__
#include "RM_typedef.h"
#include "pid.h"
#include "filter.h"
#include "Speed_Ramp.h"
// 机器人底盘修改的参数,单位为mm(毫米)

// 纵向轴距(前进后退方向)（前后两个轮中心的距离）
#define WHEEL_BASE 370.0f
// 横向轮距(左右平移方向)（左右两个轮中心的距离）
#define TRACK_WIDTH 330.0f
// 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_X 0.0f
// 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0.0f
// 轮子半径
#define RADIUS_WHEEL 80.0f

// 电机减速比
#define MOTOR_GEAR_RATIO 3591.0f / 187.0f

// 底盘功率上限 单位： w
#define POWER_MX_LIMIT 120.0f

// 电机最大电流单位：A
#define MOTOR_CURRENT_MAX 20.0f

// 电机最大转速单位：RPM
#define MOTOR_SPEED_RPM_MAX 9000.0f

// 电机最大转速单位：rad/s
#define MOTOR_SPEED_RADIAN_MAX 942.47779607693793f

// RPM转速制电机转速归一化
#define SPEED_RPM_NORMALIZE 1 / 9000.0f
// 弧度制转速归一化
#define SPEED_RAD_NORMALIZE 1 / (9000.0f * RPM_TO_RADIAN)

#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

// 以下为各个电机距离中心的直线长度
#define M1_L1 (HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y)
#define M2_L2 (HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y)
#define M3_L3 (HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y)
#define M4_L4 (HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y)

typedef enum __Chassis_Mode_enum
{
    CHASSIS_reset = 1,       // 云台复位
  CHASSIS_FOLLOW = 2,      // 跟随模式
  CHASSIS_PEG_TOP = 3,    // 小陀螺模式

} Chassis_Mode_enum;

typedef enum __Chassis_Power_Mode_enum
{
  POWER_NORMAL = 0,      // 正常功率模式
  POWER_SUPER = 1,    // 超级功率模式在当前等级上加20w

} Chassis_Power_Mode_enum;


typedef struct __Chassis_Struct
{
  uint16_t Flag;
  uint16_t Count;

  float M1_Speed;
  pid_controler M1_Pid_Speed;

  float M2_Speed;
  pid_controler M2_Pid_Speed;

  float M3_Speed;
  pid_controler M3_Pid_Speed;

  float M4_Speed;
  pid_controler M4_Pid_Speed;

  float Gimbal_Yaw_Angle_Tar;
  pid_controler Gimbal_Yaw_Pid_Angle;

  float Angle_Value;
  float Angle_Turn;
  float Angle_Now;
  float Angle_Last;

  float X_Speed_Tar;
  SpeedRampController X_Speed_Ramp;

  float Y_Speed_Tar;
  SpeedRampController Y_Speed_Ramp;

  float OMEGA_Speed_Tar;
  SpeedRampController OMEGA_Speed_Ramp;

  float X_Speed;
  float Y_Speed;
  float OMEGA_Speed;

  Chassis_Mode_enum Mode; // 底盘模式
  uint8_t Mode_Flag;      // 底盘模式切换标志位

  float Motor_Power_Now[4]; // 四个电机的当前功率值

  float Power_Sum_Now; // 当前底盘总功率
  float Power_Zeta;    // 功率衰减系数，系数同时乘在电流和速度上或者直接乘在速度上就行了？待测试

  Low_Pass_Filter_Struct Power_Zeta_Filter;

  Chassis_Power_Mode_enum Power_Mode; // 底盘功率模式模式 POWER_NORMAL正常模式 POWER_SUPER超级模式，功率在上限处增加20w

  float Power_Mx_Limit;            // 功率上限
  float Power_Mx_Add;              // 功率上限增量超级模式使用在功率上限的基础上增加
  float Power_Buffer;              // 当前功率缓冲能量值 单位： J
  float Power_Buffer_Tar;          // 当前功率缓冲能量目标值 单位： J  一般设为50J
  pid_controler Power_Control_Pid; // 功率控制PID

  uint8_t Level;                // 英雄底盘等级
  uint8_t Level_Last;           // 英雄底盘上一个等级
  float Referee_Power_Mx_Limit; // 裁判系统限制功率
  float Power_Mx_Limit_Sum;     // 裁判系统反馈的功率上限加上功率限制PID的输出值的功率值

} Chassis_Struct;
extern Chassis_Struct Chassis;

void Chassis_Init(void);

void Chassis_Set_Mode(Chassis_Mode_enum Mode);
void Chassis_Control(float X_Speed, float Y_Speed);
void Chassis_Set_Power_Mode(Chassis_Power_Mode_enum Mode);
void Chassis_Timing_Handle(void);
void Chassis_Task(void);
#endif
