/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 14:33:08
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-03-22 14:18:56
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
    uint8_t Fire_Flag:4;//发射标志位 为0则停止 为1则发射
    uint8_t Super_Mode:4;//超级底盘模式 增加20w的功率上限
} Chassis_Control_Struct; // 接收来自上板的结构体

typedef struct __Gimbal_Tx_Struct
{
    float Chassis_OMEGA_Speed; // 底盘的旋转角速度单位  0.01 rad/s
    uint16_t Shoot_Heat;       // 发生器热量
    uint8_t Robot_Level;       // 机器人等级 1~10
} Gimbal_Tx_Struct;            // 给云台上板发送的结构体数据

#pragma pack() // 指定结构体按照1字节对齐
extern Chassis_Control_Struct Chassis_Control_Data;
void Robot_Cmd_Init(void);
void Robot_Cmd_Timing_Handle(void);
void Robot_Cmd_Task(void);

#endif
