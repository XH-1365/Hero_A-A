#include "gimbal.h"
#include "DJI_Motor.h"
#include "DJI_DR16.h"
#include "CH104_IMU_CAN.h"
#include "CH104_IMU_USART.H"
#include "vofa.h"
#include "gimbal_solver.h"
#include <math.h>
#include "can_comm.h"
#include "bsp_usart.h"
#define DR16_YAW_ANGLE -(DJI_DR16_Data.RC_Value.CH2)
#define DR16_PITCH_ANGLE DJI_DR16_Data.RC_Value.CH3
// Pitch 轴上限位开关
#define PITCH_UP_LIMIT_SW HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)

#define YAW_CURRENT Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Current
#define YAW_SPEED NORMALIZE((float)Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Speed, -320.0, 320.0)
// YAW轴陀螺仪角速度
#define YAW_GYRO_SPEED NORMALIZE(-CH104_IMU_USART.Ret_Data.Data.Angular_Velocity.Gyro_Z / 180.0f * 60.0f, -320.0f, 320.0f)
// #define YAW_ANGLE Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Angle_Sum_Process.Angle_Sum_Value
// #define YAW_ANGLE -Gimbal_Get_Angle(&(Gimbal.Yaw), (float)CH104_IMU_USART.Ret_Data.Data.Euler_Angles.Yaw / 360, 0.5, -0.5)
#define YAW_ANGLE (-CH104_IMU_USART.Euler_Angle_Sum.Yaw.Angle_Sum_Value)

#define CHASSIS_OMEGA_SPEED UP_Board_RX_Data.Data.Chassis_OMEGA_Speed

#define PITCH_CURRENT Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.Current
#define PITCH_SPEED (float)Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.Speed / 18000
#define PITCH_ANGLE (-CH104_IMU_USART.Euler_Angle_Sum.Pitch.Angle_Sum_Value)
#define PITCH_MOTOR_ANGLE -Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.Angle_Sum_Process.Angle_Sum_Value
#define PITCH_GYRO_SPEED NORMALIZE(CH104_IMU_USART.Ret_Data.Data.Angular_Velocity.Gyro_Y / 180.0f * 60.0f, -320.0f, 320.0f)
// #define PITCH_ANGLE Gimbal_Get_Angle(&(Gimbal.Pitch), -(float)CH104_IMU_USART.Ret_Data.Data.Euler_Angles.Pitch / 360, 0.5, -0.5)
// #define PITCH_ANGLE Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.Angle_Sum_Process.Angle_Sum_Value

// 由于GM6020是发送电压值，而M3508和M2006是发送电流值为了方便控制统一使用Value来代替
#define GIMBAL_SET_MOTOR_VALUE(M5, M6, M7, M8) DJI_Motor_Set_Current_Value(DJI_MGRP2, DJI_Motor_TX_1_4, M5, M6, M7, M8)
#define GIMBAL_VOFA_DEBUG 0

Gimbal_Struct Gimbal = {0};
float current_value = 0;
float current_tar_value = 0;
float Yaw_Angle_Value = -0.2475f;
float Yaw_Angle_Set = 0;
float Pitch_Angle_Value = -0.0055f;
float Pitch_Screw_Angle = 0;
float Pitch_Screw_Angle_Offset = 0;
float Pitch_Motor_Angle_Offset = 0;
float Pitch_Motor_Angle = 0;
float Gyro_X_Sum = 0;
float Yaw_Motor_Gyro_S_Diff = 0; // Yaw轴电机转成Yaw轴转速后与Y轴陀螺仪转速差

float Yaw_Angle_Kf = 0.0f;    // YAW轴角度环前馈系数
float Yaw_Angle_Last_Tar = 0; // YAW轴角度环上一次目标值
float Yaw_Angle_F_Err = 0;    // YAW轴角度环前馈误差
float Yaw_Angle_F_Out = 0;    // YAW轴角度环前馈输出值
float Yaw_Angle_F_Enable = 0; // Yaw前馈使能

float yaw_sum;
float pitch_sum;

float Speed_Feedforward_Value_Temp;

/**
 * @brief 设置云台电机角度增加函数指针
 *
 * 该函数根据传入的云台电机类型，设置对应电机的角度增加函数指针。
 *
 * @param Gimbal_Motor 云台电机类型，取值为GIMBAL_MOTOR_YAW或GIMBAL_MOTOR_PITCH
 * @param Get_Angel_Add_Fn 指向获取角度增加值的 函数指针
 */
void Gimbal_Set_Add_Angel_Fn_Ptr(Gimbal_Motor_enum Gimbal_Motor, float (*Get_Angel_Add_Fn)(void))
{
    switch (Gimbal_Motor)
    {
    case GIMBAL_MOTOR_YAW:
        Gimbal.Yaw.Get_Angel_Add_Fn = Get_Angel_Add_Fn;
        break;

    case GIMBAL_MOTOR_PITCH:
        Gimbal.Pitch.Get_Angel_Add_Fn = Get_Angel_Add_Fn;
        break;

    default:
        break;
    }
}

// 获取Pitch轴机械角度
float Gimbal_Get_Mech_Angle(void)
{
    Gimbal.Pitch.Motor_Angle = PITCH_MOTOR_ANGLE + Gimbal.Pitch.Motor_Angle_Offset;
    return Gimbal.Pitch.Mech_Angle = Gimbal_Screw_Pitch_Get_Angle((Gimbal.Pitch.Motor_Angle) * PIX2) / PI;
}

// 将角度增量累加并赋值给云台目标值变量
void Gimbal_Calculate_Tar_Angel(Gimbal_Motor_enum Gimbal_Motor)
{
    float Add_Angel = 0;
    switch (Gimbal_Motor)
    {
    case GIMBAL_MOTOR_YAW:
        if (Gimbal.Yaw.Get_Angel_Add_Fn != NULL)
        {
            Gimbal.Yaw.Angle_Value += Gimbal.Yaw.Get_Angel_Add_Fn();
           Gimbal.Yaw.Angle_Value += Gimbal.Yaw.Angle_mini_pc ;
        }

        Gimbal.Yaw.Angle_Tar = Gimbal.Yaw.Angle_Value;
        break;

    case GIMBAL_MOTOR_PITCH:

        if ((Gimbal.Pitch.Get_Angel_Add_Fn != NULL) && (Gimbal.Pitch.Inited_Flag == 1))
        {
            Add_Angel = Gimbal.Pitch.Get_Angel_Add_Fn();
            if ((Gimbal.Pitch.Overrun_Flag == 1) && (Add_Angel < 0))
            {
                Gimbal.Pitch.Angle_Value += Add_Angel;
            }
            else if ((Gimbal.Pitch.Overrun_Flag == 2) && (Add_Angel > 0))
            {
                Gimbal.Pitch.Angle_Value += Add_Angel;
            }
            else if ((Gimbal.Pitch.Overrun_Flag == 0))
            {
                Gimbal.Pitch.Angle_Value += Add_Angel;
            }
            Gimbal.Pitch.Angle_Value+= Gimbal.Pitch.Angle_mini_pc;
        }

        Gimbal.Pitch.Angle_Tar = Gimbal.Pitch.Angle_Value;
        break;

    default:
        break;
    }
}

/// @brief 云台初始化
/// @param
void Gimbal_Init(void)
{
    Gimbal.Flag = 0;
    Gimbal.Count = 0;

    Gimbal.Control_Flag = 0;
    Gimbal.Control_Count = 0;

    Gimbal.Yaw.Current_Tar = 0;
    Low_Pass_Filter_Init(&Gimbal.Yaw.Filter_Current, 0.38f, YAW_CURRENT);
    PID_Init(&Gimbal.Yaw.Pid_Current);
    PID_Set(&Gimbal.Yaw.Pid_Current, 2.0, 0.2, 0, 5.0, 0.000006, 1.5);

    Gimbal.Yaw.Speed_Tar = 0;
    PID_Init(&Gimbal.Yaw.Pid_Speed);
    // PID_Set(&Gimbal.Yaw.Pid_Speed, 14, 0.01, 0, 0.05, 0.0001, 0.99);
    PID_Set(&Gimbal.Yaw.Pid_Speed, 7.5f, 0, 0.1, 1, 0.000005, 0.99);

    Gimbal.Yaw.Angle_Tar = 0;
    PID_Init(&Gimbal.Yaw.Pid_Angle);
    PID_Set(&Gimbal.Yaw.Pid_Angle, 3.6, 0.00, 0, 0.1, 0.00001, 1.0);

    Gimbal.Pitch.Current_Tar = 0;
    PID_Init(&Gimbal.Pitch.Pid_Current);
    PID_Set(&Gimbal.Pitch.Pid_Current, 0.1, 0.01, 0, 12, 0.00006, 1.5);

    Gimbal.Pitch.Speed_Tar = 0;
    PID_Init(&Gimbal.Pitch.Pid_Speed);
    // PID_Set(&Gimbal.Pitch.Pid_Speed, 3, 0.02, 0.0, 0.35, 0.0001, 0.9);
    PID_Set(&Gimbal.Pitch.Pid_Speed, 4, 0.01, 0, 2.5, 0.0001, 0.98);
    Gimbal.Pitch.Angle_Tar = 0;
    PID_Init(&Gimbal.Pitch.Pid_Angle);
    PID_Set(&Gimbal.Pitch.Pid_Angle, 9, 0, 0, 0.03, 0.0001, 0.98);

    Gimbal.Pitch.Mech_Angle_Max_Limit = (0.6897 - 0.38) / PI;
    Gimbal.Pitch.Mech_Angle_Min_Limit = (-0.4453 + 0.1) / PI;

    Gimbal.Yaw.Speed_FF_Kf = -0.0007f;
    Low_Pass_Filter_Init(&Gimbal.Yaw.Filter_Speed_FF, 0.2f, 0.0f);
    
    // PID_Set(&Gimbal.Pitch.Pid_Angle, 0.1, 0, 0, 0.03, 0.0001, 0.9);
}

void Gimbal_Timing_Handle(void)
{
    if (Gimbal.Count > 0)
    {
        Gimbal.Count--;
    }
    if (Gimbal.Power_Down_Count > 0)
    {
        Gimbal.Power_Down_Count--;
    }
}

void Gimbal_Power_Down_Handle()
{
    if (Gimbal.Power_Down_Count > 0)
    {
        return;
    }

    switch (Gimbal.Power_Down_Flag)
    {
    case 0:
        if (Gimbal.Control_Flag == 2)
        {
            Gimbal.Power_Down_Flag = 1;
        }

        break;
    case 1:

        if ((Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Daemon->Online_Flag == 0) ||
            (Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Daemon->Online_Flag == 0))
        {
            Gimbal.Power_Down_Flag = 2;
        }
        break;
    case 2:

        if ((Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Daemon->Online_Flag == 1) &&
            (Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Daemon->Online_Flag == 1))
        {
            Gimbal.Yaw.Angle_Value = YAW_ANGLE;
            Gimbal.Pitch.Angle_Value = PITCH_ANGLE;
            Gimbal.Power_Down_Flag = 1;
        }
        break;

    default:
        break;
    }
}

// Pitch轴初始化以及限位操作
void Gimbal_Handle(void)
{

    switch (Gimbal.Flag)
    {
    case 0:
        if ((Gimbal.Count != 0))
        {
            return;
        }
        if (Gimbal.Control_Flag == 2)
        {

            Gimbal.Flag = 1;
        }
        Gimbal.Count = 100;
        break;

    case 1:

        if ((Gimbal.Count == 0))
        {
            Gimbal.Pitch.Angle_Value += -0.001f;
            Gimbal.Count = 10;
        }

        if (PITCH_UP_LIMIT_SW == 1)
        {
            Gimbal.Pitch.Inited_Flag = 1;                                        // 初始化完成标志位使能
            Gimbal.Pitch.Motor_Angle_Offset = 135.0f - PITCH_MOTOR_ANGLE - 7.0f; // 此时为丝杆位于45mm的高度的电机转子角度的偏移
            Gimbal.Pitch.Motor_Angle = PITCH_MOTOR_ANGLE + Pitch_Motor_Angle_Offset;
            Gimbal.Pitch.Mech_Angle = Gimbal_Screw_Pitch_Get_Angle((Pitch_Motor_Angle)*PIX2) / PI;
            Gimbal.Flag = 2;
        }

        break;

    case 2: // 判断云台是否超限

        if (Gimbal.Pitch.Mech_Angle >= Gimbal.Pitch.Mech_Angle_Max_Limit)
        {
            Gimbal.Pitch.Angle_Value = Gimbal.Pitch.Mech_Angle_Max_Limit; // 将目标累加值设为当前机械角度
            Gimbal.Pitch.IMU_Angle_Overrun_Value = PITCH_ANGLE;           // 记录超上限那一刻的值
            Gimbal.Flag = 3;
            Gimbal.Pitch.Close_Loop_Mode = 1;
            Gimbal.Pitch.Overrun_Flag = 1;
        }
        else if (Gimbal.Pitch.Mech_Angle <= Gimbal.Pitch.Mech_Angle_Min_Limit)
        {
            Gimbal.Pitch.Angle_Value = Gimbal.Pitch.Mech_Angle_Min_Limit; // 将目标累加值设为当前机械角度
            Gimbal.Pitch.IMU_Angle_Overrun_Value = PITCH_ANGLE;           // 记录超下限那一刻的值
            Gimbal.Flag = 4;
            Gimbal.Pitch.Close_Loop_Mode = 1;
            Gimbal.Pitch.Overrun_Flag = 2;
        }
        else
        {
            Gimbal.Pitch.Overrun_Flag = 0;
        }

        // Gimbal.Count = 10;
        break;

    case 3:
        if (PITCH_ANGLE > (Gimbal.Pitch.IMU_Angle_Overrun_Value + 0.001f))
        {
            Gimbal.Pitch.Close_Loop_Mode = 0;       // 进入陀螺仪闭环模式
            Gimbal.Pitch.Angle_Value = PITCH_ANGLE; // 将目标累加值设为当前陀螺仪角度
            Gimbal.Flag = 5;
        }
        else if ((Gimbal.Pitch.Mech_Angle < (Gimbal.Pitch.Mech_Angle_Max_Limit - 0.001f)) && (Gimbal.Pitch.Mech_Angle > (Gimbal.Pitch.Mech_Angle_Min_Limit + 0.001f)))
        {
            Gimbal.Pitch.Overrun_Flag = 0;          // 清除超限标志位
            Gimbal.Pitch.Close_Loop_Mode = 0;       // 进入陀螺仪闭环模式
            Gimbal.Pitch.Angle_Value = PITCH_ANGLE; // 将目标累加值设为当前陀螺仪角度
            Gimbal.Flag = 2;
        }

        break;
    case 4:
        if (PITCH_ANGLE < (Gimbal.Pitch.IMU_Angle_Overrun_Value - 0.001f))
        {
            Gimbal.Pitch.Close_Loop_Mode = 0;       // 进入陀螺仪闭环模式
            Gimbal.Pitch.Angle_Value = PITCH_ANGLE; // 将目标累加值设为当前陀螺仪角度
            Gimbal.Flag = 6;
        }
        else if ((Gimbal.Pitch.Mech_Angle < (Gimbal.Pitch.Mech_Angle_Max_Limit - 0.001f)) && (Gimbal.Pitch.Mech_Angle > (Gimbal.Pitch.Mech_Angle_Min_Limit + 0.001f)))
        {
            Gimbal.Pitch.Overrun_Flag = 0;          // 清除超限标志位
            Gimbal.Pitch.Close_Loop_Mode = 0;       // 进入陀螺仪闭环模式
            Gimbal.Pitch.Angle_Value = PITCH_ANGLE; // 将目标累加值设为当前陀螺仪角度
            Gimbal.Flag = 2;
        }
        break;

    case 5:
        if ((Gimbal.Pitch.Mech_Angle < (Gimbal.Pitch.Mech_Angle_Max_Limit - 0.001f)) && (Gimbal.Pitch.Mech_Angle > (Gimbal.Pitch.Mech_Angle_Min_Limit + 0.001f)))
        {
            Gimbal.Pitch.Overrun_Flag = 0;
            Gimbal.Flag = 2;
        }
        else if (PITCH_ANGLE < Gimbal.Pitch.IMU_Angle_Overrun_Value)
        {
            // Gimbal.Pitch.IMU_Angle_Overrun_Value = PITCH_ANGLE; // 将目标累加值设为当前机械角度
            Gimbal.Pitch.Overrun_Flag = 1;    // 禁止角度增量
            Gimbal.Pitch.Close_Loop_Mode = 1; // 进入机械角闭环模式
            Gimbal.Flag = 3;
        }
        break;
    case 6:
        if ((Gimbal.Pitch.Mech_Angle < (Gimbal.Pitch.Mech_Angle_Max_Limit - 0.01f)) && (Gimbal.Pitch.Mech_Angle > (Gimbal.Pitch.Mech_Angle_Min_Limit + 0.01f)))
        {
            Gimbal.Pitch.Overrun_Flag = 0;
            Gimbal.Flag = 2;
        }
        else if (PITCH_ANGLE > Gimbal.Pitch.IMU_Angle_Overrun_Value)
        {
            // Gimbal.Pitch.IMU_Angle_Overrun_Value = PITCH_ANGLE; // 将目标累加值设为当前机械角度
            Gimbal.Pitch.Overrun_Flag = 2;    // 禁止角度减量
            Gimbal.Pitch.Close_Loop_Mode = 1; // 进入机械角闭环模式
            Gimbal.Flag = 4;
        }
        break;

    default:
        break;
    }
}
// 云台Vofa调试发送
void Gimbal_Vofa_Tx_Handle(void)
{
    if (Gimbal.Vofa_Count > 0)
    {
        Gimbal.Vofa_Count--;
    }
    else
    {
        Gimbal.Vofa_Count = 10;

//        Vofa_Set_TX_Value(VOFA_TX_CURRENT,      Gimbal.Yaw.Angle_Tar);
//        Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR,  Gimbal.Yaw.Filter_Current.Output);
//        Vofa_Set_TX_Value(VOFA_TX_SPEED,        YAW_CURRENT);
//        Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR,    Gimbal.Yaw.Speed_Tar);
//        Vofa_Set_TX_Value(VOFA_TX_ANGLE,        YAW_GYRO_SPEED);
//        Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR,    YAW_ANGLE);

//        Vofa_Transmit();
    }
}
// 云台Vofa调试接收
void Gimbal_Vofa_Rx_Handle(void)
{

//    Gimbal.Yaw.Pid_Current.kp = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KP);
//    Gimbal.Yaw.Pid_Current.ki = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KI);
//    Gimbal.Yaw.Pid_Current.kd = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KD);
//    Gimbal.Yaw.Current_Tar = Vofa_Get_RX_Value(VOFA_RX_CURRENT_TAR);
//    Gimbal.Yaw.Pid_Current.startfalg = Vofa_Get_RX_Value(VOFA_RX_CURRENT_SW) ? PID_ENABLE : PID_DISABLE;

//    Gimbal.Yaw.Pid_Speed.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
//    Gimbal.Yaw.Pid_Speed.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
//    Gimbal.Yaw.Pid_Speed.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
//    Gimbal.Yaw.Speed_Tar = Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
//    Gimbal.Yaw.Pid_Speed.startfalg = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? PID_ENABLE : PID_DISABLE;

//    Gimbal.Yaw.Pid_Angle.kp = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KP);
//    Gimbal.Yaw.Pid_Angle.ki = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KI);
//    Gimbal.Yaw.Pid_Angle.kd = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KD);
//    Yaw_Angle_Kf = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KF);
//    Gimbal.Yaw.Angle_Tar = Vofa_Get_RX_Value(VOFA_RX_ANGLE_TAR);
//    Gimbal.Yaw.Pid_Angle.startfalg = Vofa_Get_RX_Value(VOFA_RX_ANGLE_SW) ? PID_ENABLE : PID_DISABLE;
}
// 云台电机PID控制函数
void Gimbal_Control_Handle(void)
{
    if (Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.RX_Flag != 1 || Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.RX_Flag != 1)
    {

        return;
    }

    if (Gimbal.Control_Count > 0)
    {
        Gimbal.Control_Count--;
    }
    switch (Gimbal.Control_Flag)
    {
    case 0:
        if (Gimbal.Control_Count != 0)
        {
            return;
        }
        Gimbal.Control_Count = 0;
        Gimbal.Control_Flag = 1;

#if GIMBAL_VOFA_DEBUG != 1

        Gimbal.Yaw.Pid_Current.startfalg = PID_ENABLE;
        Gimbal.Yaw.Pid_Speed.startfalg = PID_ENABLE;
        Gimbal.Yaw.Pid_Angle.startfalg = PID_ENABLE;

        Gimbal.Pitch.Pid_Current.startfalg = PID_ENABLE;
        Gimbal.Pitch.Pid_Speed.startfalg = PID_ENABLE;
        Gimbal.Pitch.Pid_Angle.startfalg = PID_ENABLE;
#endif
        break;

    case 1:

        if ((Gimbal.Control_Count != 0))
        {
            return;
        }
        Gimbal.Yaw.Angle_Value = YAW_ANGLE;
        Gimbal.Pitch.Angle_Value = PITCH_ANGLE;
        // Gimbal.Yaw.Angle_Tar = Yaw_Angle_Value;

        Gimbal.Control_Flag = 2;
        break;

    case 2:
#if GIMBAL_VOFA_DEBUG
        Gimbal_Vofa_Rx_Handle();
#else

#endif

        if (Gimbal.Yaw.Angle_Count > 0)
        {
            Gimbal.Yaw.Angle_Count--;
        }
        if (Gimbal.Yaw.Angle_Count == 0)
        {
            Gimbal.Yaw.Angle_Count = 3;
            // 计算目标角度值
            Gimbal_Calculate_Tar_Angel(GIMBAL_MOTOR_YAW);
            // Yaw轴角度环
            Gimbal.Yaw.Pid_Angle.err = Gimbal.Yaw.Angle_Tar - YAW_ANGLE;
            Gimbal.Yaw.Pid_Angle.output = pid_error_input(&Gimbal.Yaw.Pid_Angle, Gimbal.Yaw.Pid_Angle.err);
            Gimbal.Yaw.Speed_Tar = Gimbal.Yaw.Pid_Angle.output;

            // Yaw轴角度环前馈
            Yaw_Angle_F_Err = Gimbal.Yaw.Angle_Tar - Yaw_Angle_Last_Tar;
            Yaw_Angle_F_Out = Yaw_Angle_Kf * (YAW_SPEED - (Yaw_Angle_F_Err / 0.005f));
            Yaw_Angle_Last_Tar = Gimbal.Yaw.Angle_Tar;
            //--------------------------------

            Gimbal.Yaw.Speed_Feedforward_Value =CHASSIS_OMEGA_SPEED * Gimbal.Yaw.Speed_FF_Kf;
             Gimbal.Yaw.Filter_Speed_FF.Output=  Low_Pass_Filter(&Gimbal.Yaw.Filter_Speed_FF, Gimbal.Yaw.Speed_Feedforward_Value);

            // Yaw轴速度环
            if(   Gimbal.Yaw.Speed_Tar <0.35f  && Gimbal.Yaw.Speed_Tar >(-0.35f))  
            {
                if(Gimbal.Yaw.Speed_Tar>0)
                    Gimbal.Yaw.Speed_Tar +=0.010f;
                else
                    Gimbal.Yaw.Speed_Tar -=0.01f;
            }

            Gimbal.Yaw.Pid_Speed.err = Gimbal.Yaw.Speed_Tar + Yaw_Angle_F_Out - YAW_GYRO_SPEED ;
            // Gimbal.Yaw.Pid_Speed.err = Gimbal.Yaw.Speed_Tar - YAW_SPEED;
            Gimbal.Yaw.Pid_Speed.output = pid_error_input(&Gimbal.Yaw.Pid_Speed, Gimbal.Yaw.Pid_Speed.err);

            // Gimbal.Yaw.Speed_Feedforward_Value = 0;
         

            Gimbal.Yaw.Current_Tar = Gimbal.Yaw.Pid_Speed.output + Gimbal.Yaw.Filter_Speed_FF.Output;
        }

        Gimbal.Yaw.Current_Tar = Gimbal.Yaw.Pid_Speed.output + Gimbal.Yaw.Filter_Speed_FF.Output;

        // Yaw轴电流环
        // 电流环目标值低通滤波
        Gimbal.Yaw.Filter_Current.Output = Low_Pass_Filter(&Gimbal.Yaw.Filter_Current, Gimbal.Yaw.Current_Tar);
        Gimbal.Yaw.Pid_Current.err = Gimbal.Yaw.Filter_Current.Output - YAW_CURRENT;
        Gimbal.Yaw.Pid_Current.output = pid_error_input(&Gimbal.Yaw.Pid_Current, Gimbal.Yaw.Pid_Current.err);

        if (Gimbal.Pitch.Angle_Count > 0)
        {
            Gimbal.Pitch.Angle_Count--;
        }
        if (Gimbal.Pitch.Angle_Count == 0)
        {
            Gimbal.Pitch.Angle_Count = 4;
            // 计算目标角度值
            Gimbal_Calculate_Tar_Angel(GIMBAL_MOTOR_PITCH);

            // Pitch轴角度环

            if (Gimbal.Pitch.Close_Loop_Mode == 0)
            {
                Gimbal.Pitch.Pid_Angle.err = Gimbal.Pitch.Angle_Tar - PITCH_ANGLE;
            }
            else
            {
                Gimbal.Pitch.Pid_Angle.err = Gimbal.Pitch.Angle_Tar - Gimbal.Pitch.Mech_Angle;
            }

            Gimbal.Pitch.Pid_Angle.output = pid_error_input(&Gimbal.Pitch.Pid_Angle, Gimbal.Pitch.Pid_Angle.err);

            Gimbal.Pitch.Speed_Tar = Gimbal.Pitch.Pid_Angle.output;

            // Pitch轴速度环
            Gimbal.Pitch.Pid_Speed.err = Gimbal.Pitch.Speed_Tar - (PITCH_SPEED * 0.80f + PITCH_GYRO_SPEED * 2.5f);
            Gimbal.Pitch.Pid_Speed.output = pid_error_input(&Gimbal.Pitch.Pid_Speed, Gimbal.Pitch.Pid_Speed.err);
        }

        GIMBAL_SET_MOTOR_VALUE(
            Gimbal.Yaw.Pid_Current.output,
            Gimbal.Pitch.Pid_Speed.output,
            // 0,
            // 0,
            
            0,
            0);

#if GIMBAL_VOFA_DEBUG

        Gimbal_Vofa_Tx_Handle();
#else
        // Gimbal_Vofa_Tx_Handle();
#endif

        Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.RX_Flag = 0;
        Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.RX_Flag = 0;
        Gimbal.Control_Count = 1;
        break;
    }
}

// 云台任务
void Gimbal_Task(void)
{
    if (Gimbal.Pitch.Inited_Flag == 1) // 只有当初始化完成才能解算机械角度
    {
        Gimbal.Pitch.Mech_Angle = Gimbal_Get_Mech_Angle();
    }
    Gimbal_Power_Down_Handle();
    Gimbal_Handle();
    Gimbal_Control_Handle();
}
