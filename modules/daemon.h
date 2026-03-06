/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2025-02-28 11:07:51
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-07 22:43:46
 * @FilePath: \RM_Hero_UP_Board\modules\daemon.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _DAEMON_H__
#define _DAEMON_H__
#include "RM_typedef.h"
typedef enum __Daemon_ID_enum
{
    DAE_CHASSIS_M3508_1 = 0,
    DAE_CHASSIS_M3508_2,
    DAE_CHASSIS_M3508_3,
    DAE_CHASSIS_M3508_4,
    DAE_GIMBAL_GM6020_YAW,
    DAE_GIMBAL_M2006_PITCH,
    DAE_SHOOT_M3508_1,// 头正对自己右电机摩擦轮
    DAE_SHOOT_M3508_2,// 头正对自己左电机摩擦轮
    DAE_DTOF,//激光测距仪
      DAE_DM_4310,//达妙电机
    DAEMON_SUM
} Daemon_ID_enum;

#pragma pack(1) // 指定结构体按照1字节对齐

typedef struct __Daemon_Struct
{
    uint16_t Count;                 // 软件看门狗计时变量 在1ms定时器里大于0就--，单位：1ms
    uint16_t Overload_Count;        // 最长不喂狗时间 单位：1ms
    Fn_State_t Enable;              // 使能开关
    uint8_t Online_Flag;            // 在线标志位
    uint8_t Offline_Event;          // 掉线事件
    void (*Offline_CallBack)(void); // 掉线回调函数
} Daemon_Struct;

typedef struct __Daemon_Control_Struct
{
    uint8_t Count;
    uint8_t Flag;
} Daemon_Control_Struct;

#pragma pack() // 取消结构体对齐
/**
 * @brief Daemon回调处理函数（放入APP循环里）
 *
 * 该函数用于处理守护进程的回调事件。当Daemon_Control.Count大于0时，函数直接返回。
 * 否则，将Daemon_Control.Count设置为1（表示1ms周期）。
 *
 * 函数遍历所有守护进程（DAEMON_SUM），如果某个守护进程的Offline_CallBack不为空且Offline_Event为1，
 * 则将Offline_Event置为0，并调用Offline_CallBack函数。
 *
 * @note 该函数假设Daemon_Control和Daemon数组已经在其他地方定义和初始化。
 */
void Daemon_Callback_Handle(void);
void Daemon_Timing_Handle(void);
void Daemon_Reload(Daemon_Struct *Instance);
void Daemon_Set_Enable(Daemon_Struct *Instance, Fn_State_t Enable);

/// @brief 设置掉线回调函数
/// @param Instance 掉线检测实例
/// @param Offline_CallBack   回调函数指针
void Daemon_Set_Offline_CallBack(Daemon_Struct *Instance, void (*Offline_CallBack)(void));

void Daemon_Config(Daemon_Struct *Instance, uint16_t Overload_Count, void (*Offline_CallBack)(void), Fn_State_t Enable);
Daemon_Struct *Daemon_Register(Daemon_ID_enum Daemon_ID, uint16_t Overload_Count, void (*Offline_CallBack)(void), Fn_State_t Enable);
#endif
