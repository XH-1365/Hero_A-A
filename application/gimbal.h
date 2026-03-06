/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-08-22 21:01:10
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-04 23:08:21
 * @FilePath: \RM_Template\application\gimbal.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _GIMBAL_H__
#define _GIMBAL_H__
#include "pid.h"
#include "filter.h"

    typedef enum __Gimbal_Motor_enum {
        GIMBAL_MOTOR_YAW = 0,
        GIMBAL_MOTOR_PITCH
    } Gimbal_Motor_enum;

    typedef enum __Gimbal_Value_Ptr_enum {
        GIMBAL_SPEED_FF = 0,
    } Gimbal_Value_Ptr_enum;

typedef struct __Gimbal_Yaw_Data_Struct
{
    float Current_Tar;
    pid_controler Pid_Current;
    Low_Pass_Filter_Struct Filter_Current;


    float Speed_Tar;
    pid_controler Pid_Speed;

    //  float * Speed_Feedforward_Ptr;//速度前馈指针
     float Speed_Feedforward_Value;//速度前馈值
     float Speed_FF_Kf;//速度前馈系数
     Low_Pass_Filter_Struct Filter_Speed_FF;

    float Angle_Tar; // 航向轴角度目标值映射
    pid_controler Pid_Angle;
    uint8_t Angle_Count; // 角度环周期

    float Angle_Value;
    float Angle_mini_pc;             // 视觉航向轴角度值映射
    float (*Get_Angel_Add_Fn)(void); // 获取角度增量函数的函数指针
} Gimbal_Yaw_Data_Struct;

typedef struct __Gimbal_Pitch_Data_Struct
{
    float Current_Tar;
    pid_controler Pid_Current;

    float Speed_Tar;
    pid_controler Pid_Speed;
    
    uint8_t Inited_Flag;//初始化完成标志位
    float Angle_Tar;       // 俯仰轴角度目标值

    // float *Angle_Feedback; // 角度反馈值
    pid_controler Pid_Angle;
    uint8_t Angle_Count; // 角度环周期

    float Angle_Value;               // 俯仰轴角度值映射
    float Angle_mini_pc;             // 视觉俯仰轴角度值映射
    float Mech_Angle_Max_Limit;      // 俯仰轴机械角度值上限
    float Mech_Angle_Min_Limit;      // 俯仰轴机械角度值下限
    float Motor_Angle;
    float Motor_Angle_Offset;        // 电机角度偏移

    float Mech_Angle;//丝杆云台解算后的角度归一化为0~1 对应0~360度或0~2PI
    uint8_t Overrun_Flag; // 俯仰轴超限标志位0表示未超限，1表示超上限，2表示超下限
    uint8_t Close_Loop_Mode;//闭环模式1为机械角闭环，0为陀螺仪闭环
    float IMU_Angle_Offset;          // 陀螺仪角度偏移
    float IMU_Angle_Overrun_Value;   // 陀螺仪超限那一刻的值
    float (*Get_Angel_Add_Fn)(void); // 获取角度增量函数的函数指针
} Gimbal_Pitch_Data_Struct;

typedef struct __Gimbal_Struct
{
    uint16_t Flag;
    uint16_t Count;

    uint8_t Control_Flag;
    uint16_t Control_Count;

   uint8_t Power_Down_Flag;
     uint16_t Power_Down_Count;

    uint16_t Vofa_Count;

    /*
    云台航向轴电机ID:5
    */
    Gimbal_Yaw_Data_Struct Yaw;

    /*
    云台俯仰轴电机ID:6
    */
    Gimbal_Pitch_Data_Struct Pitch;

} Gimbal_Struct;


extern Gimbal_Struct Gimbal ;
extern float Yaw_Motor_Gyro_S_Diff; // Yaw轴电机转成Yaw轴转速后与Y轴陀螺仪转速差

float Screw_Pitch_Length_To_Angle_Calculate(float Pitch_Angle);

void Gimbal_Init(void);

void Gimbal_Set_Mode(uint8_t Mode);

void Gimbal_Timing_Handle(void);

void Gimbal_Task(void);

void Gimbal_Set_Add_Angel_Fn_Ptr(Gimbal_Motor_enum Gimbal_Motor, float (*Get_Angel_Add_Fn)(void));
void Gimbal_Set_Value_Ptr(Gimbal_Value_Ptr_enum Value_Ptr, float *Value);
#endif
