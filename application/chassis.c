#include "chassis.h"
#include "DJI_Motor.h"
#include "pid.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "DJI_DR16.h"
#include "vofa.h"
#include "math.h"
/*
M2        M1

     o

M3        M4
*/
#define M1_SPEED (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_1].Ret.Speed / 9000
#define M2_SPEED (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_2].Ret.Speed / 9000
#define M3_SPEED (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_3].Ret.Speed / 9000
#define M4_SPEED (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_4].Ret.Speed / 9000

// #define GIMBAL_YAW_ANGLE Chassis_Get_Angle(&Chassis, Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Angle, 1.0, 0.0)
#define GIMBAL_YAW_ANGLE Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Angle

#define X_VALUE Chassis.X_Speed
#define Y_VALUE Chassis.Y_Speed
// #define X_VALUE DJI_DR16_Data.RC_Value.CH0 / 2
// #define Y_VALUE DJI_DR16_Data.RC_Value.CH1 / -2
// #define OMEGA_VALUE -DJI_DR16_Data.RC_Value.Wheel / 2
// #define OMEGA_VALUE Chassis.Gimbal_Yaw_Pid_Angle.output / 100

// #define COS_BETA() 0.7071f
// #define COS_ALPHA() 0.7071f
#define COS_BETA() cos((PI / 4) + ((GIMBAL_YAW_ANGLE - Gimbal_Yaw_Angle_Offset) * 2 * PI))
#define COS_ALPHA() cos((PI / 4) - ((GIMBAL_YAW_ANGLE - Gimbal_Yaw_Angle_Offset) * 2 * PI))

// 以下为各个电机距离中心的长度
#define M1_L1 0.5f
#define M2_L2 0.5f
#define M3_L3 0.5f
#define M4_L4 0.5f
#define CHASSIS_SET_MOTOR_CURRENT(M1, M2, M3, M4) DJI_Motor_Set_Current_Value(DJI_MGRP1, DJI_Motor_TX_1_4, M1, M2, M3, M4)

#define CHASSIS_VOFA_DEBUG 0
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

    float Gimbal_Yaw_Angle;
    pid_controler Gimbal_Yaw_Pid_Angle;

    float Angle_Value;
    float Angle_Turn;
    float Angle_Now;
    float Angle_Last;

    float X_Speed;
    float Y_Speed;

    Chassis_Mode_enum Mode; // 底盘模式
    uint8_t Mode_Flag;      // 底盘模式切换标志位

} Chassis_Struct;

Chassis_Struct Chassis = {0};
char Chassis_Str[100];
float OMEGA_VALUE = 0;
float Gimbal_Yaw_Angle_Offset = 0;
float Gimbal_Yaw_Relative_Angle = 0;
void Chassis_Init(void)
{
    Chassis.Flag = 0;
    Chassis.Count = 0;
    Chassis.Mode_Flag = 1;
    Chassis.Mode=CHASSIS_FOLLOW;

    Chassis.M1_Speed = 0;
    PID_Init(&Chassis.M1_Pid_Speed);
    PID_Set(&Chassis.M1_Pid_Speed, 2.25, 0.06, 0.4, 0.35, 0.001, 0.9);

    Chassis.M2_Speed = 0;
    PID_Init(&Chassis.M2_Pid_Speed);
    PID_Set(&Chassis.M2_Pid_Speed, 2.25, 0.06, 0.4, 0.35, 0.001, 0.9);

    Chassis.M3_Speed = 0;
    PID_Init(&Chassis.M3_Pid_Speed);
    PID_Set(&Chassis.M3_Pid_Speed, 2.25, 0.06, 0.4, 0.35, 0.001, 0.9);

    Chassis.M4_Speed = 0;
    PID_Init(&Chassis.M4_Pid_Speed);
    PID_Set(&Chassis.M4_Pid_Speed, 2.25, 0.06, 0.4, 0.35, 0.001, 0.9);

    Chassis.Gimbal_Yaw_Angle = 0;
    PID_Init(&Chassis.Gimbal_Yaw_Pid_Angle);
    PID_Set(&Chassis.Gimbal_Yaw_Pid_Angle, 3.5, 0, 0, 0.03, 0.001, 1.0);
}

void Chassis_Set_Mode(Chassis_Mode_enum Mode)
{
    Chassis.Mode = Mode;
}

void Chassis_Control(float X_Speed, float Y_Speed)
{
    Chassis.X_Speed = X_Speed;
    Chassis.Y_Speed = Y_Speed;
}

void Chassis_Timing_Handle(void)
{
    if (Chassis.Count > 0)
    {
        Chassis.Count--;
    }
}
void Chassis_Clean_Angle_Turn(Chassis_Struct *Ca_motorx)
{
    Ca_motorx->Angle_Turn = 0;
}

float Chassis_Get_Angle(Chassis_Struct *Ca_motorx, float Angle_Value, float Max, float Min)
{
    float Jump_Value = Max - Min;

    Ca_motorx->Angle_Last = Ca_motorx->Angle_Now;
    Ca_motorx->Angle_Now = Angle_Value;
    if ((Ca_motorx->Angle_Last - Ca_motorx->Angle_Now) > (Jump_Value - 0.05f))
    {
        (Ca_motorx->Angle_Turn) += Jump_Value;
    }
    else if ((Ca_motorx->Angle_Last - Ca_motorx->Angle_Now) < -(Jump_Value - 0.05f))
    {
        (Ca_motorx->Angle_Turn) -= Jump_Value;
    }
    Ca_motorx->Angle_Value = (Ca_motorx->Angle_Turn) + (Ca_motorx->Angle_Now);
    return Ca_motorx->Angle_Value;
}

void Chassis_Task(void)
{

    if (Chassis.Count != 0)
    {
        return;
    }
    // if (Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_1].Ret.RX_Flag != 1
    // || Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_2].Ret.RX_Flag != 1
    // || Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_3].Ret.RX_Flag != 1
    // || Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_4].Ret.RX_Flag != 1
    // )
    // {
    //     return;
    // }
    switch (Chassis.Flag)
    {
    case 0:
        Chassis.Count = 501;
        Chassis.Flag = 1;

#if CHASSIS_VOFA_DEBUG
#else
        Chassis.M1_Pid_Speed.startfalg = PID_ENABLE;
        Chassis.M2_Pid_Speed.startfalg = PID_ENABLE;
        Chassis.M3_Pid_Speed.startfalg = PID_ENABLE;
        Chassis.M4_Pid_Speed.startfalg = PID_ENABLE;
        Chassis.Gimbal_Yaw_Pid_Angle.startfalg = PID_ENABLE;

#endif
        break;

    case 1:
        Chassis.Count = 500;
        Gimbal_Yaw_Angle_Offset = GIMBAL_YAW_ANGLE;
        Chassis.Gimbal_Yaw_Angle = GIMBAL_YAW_ANGLE;
        Chassis.Flag = 2;
        break;

    case 2:
#if CHASSIS_VOFA_DEBUG
        Chassis.M1_Pid_Speed.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        Chassis.M1_Pid_Speed.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        Chassis.M1_Pid_Speed.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        Chassis.M1_Speed = Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
        Chassis.M1_Pid_Speed.output = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? Chassis.M1_Pid_Speed.output : 0;
#else

        Chassis.Gimbal_Yaw_Pid_Angle.err = Chassis.Gimbal_Yaw_Angle - GIMBAL_YAW_ANGLE;
        Chassis.Gimbal_Yaw_Pid_Angle.output = pid_error_input(&Chassis.Gimbal_Yaw_Pid_Angle, Chassis.Gimbal_Yaw_Pid_Angle.err);
        switch (Chassis.Mode_Flag)
        {
        case 1:
            if (Chassis.Mode == CHASSIS_FOLLOW)
            {
                Chassis.Mode_Flag = 2;
            }
            else
            {
                OMEGA_VALUE = Chassis.Gimbal_Yaw_Pid_Angle.output;
            }

            break;
        case 2:
            if (Chassis.Mode == CHASSIS_PEG_TOP)
            {
                Chassis_Clean_Angle_Turn(&Chassis);
                Chassis.Mode_Flag = 3;
            }
            else
            {
                OMEGA_VALUE = 0.5;
            }

            break;
        case 3:
            OMEGA_VALUE = 0;
            // PID_Set(&Chassis.Gimbal_Yaw_Pid_Angle, 3.5, 0, 0, 0.03, 0.001, 1.0);
            PID_Set(&Chassis.Gimbal_Yaw_Pid_Angle, 3.5, 0, 0, 0.03, 0.001, 0.5);
            Chassis.Mode_Flag = 4;
            break;
        case 4:
            OMEGA_VALUE = Chassis.Gimbal_Yaw_Pid_Angle.output;
            if ((Chassis.Gimbal_Yaw_Angle - GIMBAL_YAW_ANGLE) < 0.01f && (Chassis.Gimbal_Yaw_Angle - GIMBAL_YAW_ANGLE) > -0.01f)
            {
                PID_Set(&Chassis.Gimbal_Yaw_Pid_Angle, 3.5, 0, 0, 0.03, 0.001, 1.0);
                Chassis.Mode_Flag = 1;
            }

            break;

        default:
            break;
        }
        // OMEGA_VALUE = -DJI_DR16_Data.RC_Value.Wheel / 2;
        Chassis.M1_Speed = X_VALUE * COS_BETA() + Y_VALUE * COS_ALPHA() + OMEGA_VALUE * M1_L1;
        Chassis.M2_Speed = X_VALUE * COS_ALPHA() - Y_VALUE * COS_BETA() + OMEGA_VALUE * M2_L2;
        Chassis.M3_Speed = -X_VALUE * COS_BETA() - Y_VALUE * COS_ALPHA() + OMEGA_VALUE * M3_L3;
        Chassis.M4_Speed = -X_VALUE * COS_ALPHA() + Y_VALUE * COS_BETA() + OMEGA_VALUE * M4_L4;
        // 当 CHASSIS_VOFA_DEBUG 为 0 时，这里可以放置替代代码或留空
#endif

        Chassis.M1_Pid_Speed.err = Chassis.M1_Speed - M1_SPEED;
        Chassis.M1_Pid_Speed.output = pid_error_input(&Chassis.M1_Pid_Speed, Chassis.M1_Pid_Speed.err);

        Chassis.M2_Pid_Speed.err = Chassis.M2_Speed - M2_SPEED;
        Chassis.M2_Pid_Speed.output = pid_error_input(&Chassis.M2_Pid_Speed, Chassis.M2_Pid_Speed.err);

        Chassis.M3_Pid_Speed.err = Chassis.M3_Speed - M3_SPEED;
        Chassis.M3_Pid_Speed.output = pid_error_input(&Chassis.M3_Pid_Speed, Chassis.M3_Pid_Speed.err);

        Chassis.M4_Pid_Speed.err = Chassis.M4_Speed - M4_SPEED;
        Chassis.M4_Pid_Speed.output = pid_error_input(&Chassis.M4_Pid_Speed, Chassis.M4_Pid_Speed.err);

        CHASSIS_SET_MOTOR_CURRENT(
            Chassis.M1_Pid_Speed.output,
            Chassis.M2_Pid_Speed.output,
            Chassis.M3_Pid_Speed.output,
            Chassis.M4_Pid_Speed.output);

#if CHASSIS_VOFA_DEBUG
        Vofa_Set_TX_Value(VOFA_TX_SPEED, Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_GM6020_RX_1].Ret.Speed);
        Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Chassis.M1_Speed);
        Vofa_Transmit();
#else
        // sprintf((char *)Chassis_Str, "M1{Speed:%.2f,Angle:%.2f,Current:%.2f,Temperature:%u}\r\n",
        //         (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_1].Ret.Speed / 9000,
        //         Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_1].Ret.Angle,
        //         Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_1].Ret.Current,
        //         Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_1].Ret.Temperature);
        // HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Chassis_Str, strlen(Chassis_Str)); // 串口回传1号电机返回的数据
//        Gimbal_Yaw_Relative_Angle = (GIMBAL_YAW_ANGLE - Gimbal_Yaw_Angle_Offset) * 2 * PI;
//        Vofa_Set_TX_Value(VOFA_TX_SPEED, (GIMBAL_YAW_ANGLE - Gimbal_Yaw_Angle_Offset) * 360.0f);
// Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, cos((PI/4)-Gimbal_Yaw_Relative_Angle));
// Vofa_Set_TX_Value(VOFA_TX_ANGLE, cos((PI/4)+Gimbal_Yaw_Relative_Angle));

//        Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, COS_BETA());
//        Vofa_Set_TX_Value(VOFA_TX_ANGLE, COS_ALPHA());

//        Vofa_Transmit();
#endif

        Chassis.Count = 6;
        break;

    default:
        break;
    }
}
