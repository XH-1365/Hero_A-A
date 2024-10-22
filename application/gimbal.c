
#include "gimbal.h"
#include "DJI_Motor.h"
#include "DJI_DR16.h"
#include "CH104_IMU_CAN.h"
#include "vofa.h"
#include "pid.h"
#include <math.h>
#define DR16_YAW_ANGLE -(DJI_DR16_Data.RC_Value.CH2)
#define DR16_PITCH_ANGLE DJI_DR16_Data.RC_Value.CH3

#define YAW_CURRENT Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Current
#define YAW_SPEED -NORMALIZE((float)Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Speed, -320.0, 320.0)
#define YAW_ANGLE -Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Angle_Sum_Process.Angle_Sum_Value
// #define YAW_ANGLE Gimbal_Get_Angle(&(Gimbal.Yaw), Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Angle,1.0,0.0)
// #define YAW_ANGLE Gimbal_Get_Angle(&(Gimbal.Yaw), (float)CH104_IMU_CAN.Euler_Angles.Data.Yaw / 36000, 0.5, -0.5)

#define PITCH_CURRENT Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.Current
#define PITCH_SPEED (float)Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.Speed / 18000
// #define PITCH_ANGLE Gimbal_Get_Angle(&(Gimbal.Pitch), (Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_6].Ret.Angle),1.0,0.0)
// #define PITCH_ANGLE Gimbal_Get_Angle(&(Gimbal.Pitch), -(float)CH104_IMU_CAN.Euler_Angles.Data.Pitch / 36000, 0.5, -0.5)
#define PITCH_ANGLE Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.Angle_Sum_Process.Angle_Sum_Value

// 由于GM6020是发送电压值，而M3508和M2006是发送电流值为了方便控制统一使用Value来代替
#define GIMBAL_SET_MOTOR_VALUE(M5, M6, M7, M8) DJI_Motor_Set_Current_Value(DJI_MGRP2, DJI_Motor_TX_1_4, M5, M6, M7, M8)
#define GIMBAL_VOFA_DEBUG 0

typedef struct __Gimbal_Data_Struct
{
    float Current_Tar;
    pid_controler Pid_Current;

    float Speed_Tar;
    pid_controler Pid_Speed;

    float Angle_Tar;     // 航向轴角度目标值
    uint8_t Angle_Count; // 角度环周期
    pid_controler Pid_Angle;
    float Angle_Value;
    float Angle_Turn;
    float Angle_Now;
    float Angle_Last;
} Gimbal_Data_Struct;

typedef struct __Gimbal_Struct
{
    uint16_t Flag;
    uint16_t Count;
    uint16_t Vofa_Count;

    /*
    云台航向轴电机ID:5
    */
    Gimbal_Data_Struct Yaw;

    /*
    云台俯仰轴电机ID:6
    */
    Gimbal_Data_Struct Pitch;

} Gimbal_Struct;
Gimbal_Struct Gimbal;
float current_value = 0;
float current_tar_value = 0;
float Yaw_Angle_Value = -0.2475f;
float Yaw_Angle_Set = 0;
float Pitch_Angle_Value = -0.0055f;
float Yaw_Current_Out_Value_Last = 0;
float Yaw_Current_Out_Value = 0;

float Gimbal_Get_Angle(Gimbal_Data_Struct *gm_motorx, float Angle_Value, float Max, float Min)
{
    float Jump_Value = Max - Min;

    gm_motorx->Angle_Last = gm_motorx->Angle_Now;
    gm_motorx->Angle_Now = Angle_Value;
    if ((gm_motorx->Angle_Last - gm_motorx->Angle_Now) > (Jump_Value - 0.05f))
    {
        (gm_motorx->Angle_Turn) += Jump_Value;
    }
    else if ((gm_motorx->Angle_Last - gm_motorx->Angle_Now) < -(Jump_Value - 0.05f))
    {
        (gm_motorx->Angle_Turn) -= Jump_Value;
    }
    gm_motorx->Angle_Value = (gm_motorx->Angle_Turn) + (gm_motorx->Angle_Now);
    return gm_motorx->Angle_Value;
}

#define ALPHA 0.04f // EWMA滤波器的权重因子

// 电流滤波器函数，仅使用EWMA滤波
float Gimbal_Current_Tar_Filter(float Current_Input)
{
    static float ewma_filtered_value = 0.0f; // 保存上一次的EWMA滤波值

    // 进行EWMA滤波
    ewma_filtered_value = ALPHA * Current_Input + (1 - ALPHA) * ewma_filtered_value;

    return ewma_filtered_value;
}

void Gimbal_Set_Mode(uint8_t Mode)
{
}
// void Gimbal_Control(float Yaw_Angle, float Pitch_Angle)
// {
//     Gimbal.Yaw.Angle_Tar = Yaw_Angle;
//     Gimbal.Pitch.Angle_Tar = Pitch_Angle;
// }

/// @brief 设置增量角度，输入都是增量的角度，每运行一次增加一次，角度可以是负数
/// @param Yaw_Angle
/// @param Pitch_Angle
void Gimbal_Add_Angle(float Yaw_Add_Angle, float Pitch_Add_Angle)
{
    Yaw_Angle_Value += Yaw_Add_Angle;

    Pitch_Angle_Value += Pitch_Add_Angle;
    // if (Pitch_Angle_Value > 0.095f)
    // {
    //     Pitch_Angle_Value = 0.095f;
    // }
    // else if (Pitch_Angle_Value < -0.054f)
    // {
    //     Pitch_Angle_Value = -0.054f;
    // }
    Gimbal.Yaw.Angle_Tar = Yaw_Angle_Value;
    Gimbal.Pitch.Angle_Tar = Pitch_Angle_Value;
}
void Quadratic_formula(float a, float b, float c, float *x1, float *x2)
{
    float Delta = 0;
    //	a=1;
    //	b=-2*d*cosf(A2);
    //	c=d_square-a2_square;
    Delta = b * b - 4 * a * c;
    // Vofa_print("Delta=%.4f\r\n", Delta);
    (*x1) = (-b + sqrtf(Delta)) / (2 * a);
    (*x2) = (-b - sqrtf(Delta)) / (2 * a);
}
/// @brief 输入角度得到丝杆的位置
/// @param screw_position 单位：弧度
/// @return 丝杆的位置 单位：mm
float Screw_Pitch_Length_To_Angle_Calculate(float Pitch_Angle)
{
    float d = 0;
    float d_square = 0;
    float a1 = 31.42f;
    float c2 = 64.1515f;
    float D = Pitch_Angle;
    //  float D=1.2186f;
    //  float D=1.0f;

    float A = 0.9253;
    float A1 = 0;
    float A2 = 0;

    float a2 = 40.27f;
    float a2_square = 0;
    //  float c1=45.00f;
    float c1 = 0;
    float x1 = 0;
    float x2 = 0;

    d_square = a1 * a1 + c2 * c2 - 2 * a1 * c2 * cosf(D);
    d = sqrtf(d_square);
    // Vofa_print("d=%.4f\r\n", d);

    A1 = asinf((a1 / d) * sinf(D));
    // Vofa_print("A1=%.4f\r\n", A1);

    A2 = A - A1;
    // Vofa_print("A2=%.4f\r\n", A2);
    a2_square = a2 * a2;
    //  a2_square=d_square+c1*c1-2*d*c1*cosf(A2);
    //  a2=sqrtf(a2_square);
    //  printf("a2=%.4f\r\n",a2);
    // Quadratic_formula(1.0f,5.0f,-6.0f,&x1,&x2);//测试
    Quadratic_formula(1.0f, -2 * d * cosf(A2), d_square - a2_square, &x1, &x2);
    // Vofa_print("x1=%.4f,x2=%.4f\r\n", x1, x2);
    if (x1 <= 45 && x1 > 0)
    {
        c1 = x1;
    }
    else if (x2 <= 45 && x2 > 0)
    {
        c1 = x2;
    }
    return c1;
}

void Gimbal_Init(void)
{
    Gimbal.Flag = 0;
    Gimbal.Count = 0;

    Gimbal.Yaw.Current_Tar = 0;
    PID_Init(&Gimbal.Yaw.Pid_Current);
    PID_Set(&Gimbal.Yaw.Pid_Current, 4.3, 0.2, 0, 10, 0.00006, 1.5);

    Gimbal.Yaw.Speed_Tar = 0;
    PID_Init(&Gimbal.Yaw.Pid_Speed);
    PID_Set(&Gimbal.Yaw.Pid_Speed, 12, 0, 0, 0.05, 0.0001, 0.99);

    Gimbal.Yaw.Angle_Tar = 0;
    PID_Init(&Gimbal.Yaw.Pid_Angle);
    PID_Set(&Gimbal.Yaw.Pid_Angle, 2.1, 0, 0, 0.03, 0.00003, 0.3);

    Gimbal.Pitch.Current_Tar = 0;
    PID_Init(&Gimbal.Pitch.Pid_Current);
    PID_Set(&Gimbal.Pitch.Pid_Current, 0.1, 0.05, 0, 12, 0.00006, 1.5);

    Gimbal.Pitch.Speed_Tar = 0;
    PID_Init(&Gimbal.Pitch.Pid_Speed);
    PID_Set(&Gimbal.Pitch.Pid_Speed, 3, 0.02, 0.0, 0.35, 0.0001, 0.9);
    // PID_Set(&Gimbal.Pitch.Pid_Speed, 2, 0.12, 0, 0.1, 0.0001, 0.9);
    Gimbal.Pitch.Angle_Tar = 0;
    PID_Init(&Gimbal.Pitch.Pid_Angle);
    PID_Set(&Gimbal.Pitch.Pid_Angle, 0.1, 0, 0, 0.03, 0.0001, 0.9);
}

void Gimbal_Timing_Handle(void)
{
    if (Gimbal.Count > 0)
    {
        Gimbal.Count--;
    }
}

void Gimbal_Task(void)
{

    if (Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.RX_Flag != 1 || Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.RX_Flag != 1)
    {
        return;
    }

    // if ((Gimbal.Count != 0))
    // {
    //     return;
    // }

    switch (Gimbal.Flag)
    {
    case 0:
        if ((Gimbal.Count != 0))
        {
            return;
        }
        Gimbal.Count = 502;
        Gimbal.Flag = 1;

#if GIMBAL_VOFA_DEBUG

#else
        Gimbal.Yaw.Pid_Current.startfalg = PID_ENABLE;
        Gimbal.Yaw.Pid_Speed.startfalg = PID_ENABLE;
        Gimbal.Yaw.Pid_Angle.startfalg = PID_ENABLE;

        Gimbal.Pitch.Pid_Current.startfalg = PID_ENABLE;
        Gimbal.Pitch.Pid_Speed.startfalg = PID_ENABLE;
        Gimbal.Pitch.Pid_Angle.startfalg = PID_ENABLE;
#endif
        break;

    case 1:

        if ((Gimbal.Count != 0))
        {
            return;
        }
        Yaw_Angle_Value = YAW_ANGLE;
        Pitch_Angle_Value = PITCH_ANGLE;
        Gimbal.Flag = 2;
        break;

    case 2:
#if GIMBAL_VOFA_DEBUG

        Gimbal.Yaw.Pid_Current.kp = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KP);
        Gimbal.Yaw.Pid_Current.ki = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KI);
        Gimbal.Yaw.Pid_Current.kd = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KD);
        Gimbal.Yaw.Current_Tar = Vofa_Get_RX_Value(VOFA_RX_CURRENT_TAR);
        Gimbal.Yaw.Pid_Current.startfalg = Vofa_Get_RX_Value(VOFA_RX_CURRENT_SW) ? PID_ENABLE : PID_DISABLE;

        Gimbal.Yaw.Pid_Speed.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        Gimbal.Yaw.Pid_Speed.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        Gimbal.Yaw.Pid_Speed.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        Gimbal.Yaw.Speed_Tar = Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
        Gimbal.Yaw.Pid_Speed.startfalg = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? PID_ENABLE : PID_DISABLE;

        Gimbal.Yaw.Pid_Angle.kp = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KP);
        Gimbal.Yaw.Pid_Angle.ki = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KI);
        Gimbal.Yaw.Pid_Angle.kd = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KD);
        Gimbal.Yaw.Angle_Tar = Vofa_Get_RX_Value(VOFA_RX_ANGLE_TAR);
        Gimbal.Yaw.Pid_Angle.startfalg = Vofa_Get_RX_Value(VOFA_RX_ANGLE_SW) ? PID_ENABLE : PID_DISABLE;

        // Gimbal.Pitch.Pid_Current.kp = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KP);
        // Gimbal.Pitch.Pid_Current.ki = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KI);
        // Gimbal.Pitch.Pid_Current.kd = Vofa_Get_RX_Value(VOFA_RX_CURRENT_KD);
        // Gimbal.Pitch.Current_Tar = Vofa_Get_RX_Value(VOFA_RX_CURRENT_TAR);
        // Gimbal.Pitch.Pid_Current.startfalg = Vofa_Get_RX_Value(VOFA_RX_CURRENT_SW) ? PID_ENABLE : PID_DISABLE;

        // Gimbal.Pitch.Pid_Speed.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        // Gimbal.Pitch.Pid_Speed.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        // Gimbal.Pitch.Pid_Speed.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        // Gimbal.Pitch.Speed_Tar = Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
        // Gimbal.Pitch.Pid_Speed.startfalg = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? PID_ENABLE : PID_DISABLE;

        // Gimbal.Pitch.Pid_Angle.kp = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KP);
        // Gimbal.Pitch.Pid_Angle.ki = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KI);
        // Gimbal.Pitch.Pid_Angle.kd = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KD);
        // Gimbal.Pitch.Angle_Tar = Vofa_Get_RX_Value(VOFA_RX_ANGLE_TAR);
        // Gimbal.Pitch.Pid_Angle.startfalg = Vofa_Get_RX_Value(VOFA_RX_ANGLE_SW) ? PID_ENABLE : PID_DISABLE;

#else
        // Yaw_Angle_Value += DR16_YAW_ANGLE / 1000;
        // // Yaw_Angle_Value += ((float)DJI_DR16_Data.Mouse.X)/ -32767;
        // // Yaw_Angle_Set=Yaw_Angle_Value;
        // // Gimbal.Yaw.Angle_Tar =Yaw_Angle_Set+ (Yaw_Angle_Set-YAW_ANGLE);
        // Gimbal.Yaw.Angle_Tar = Yaw_Angle_Value;

        // Pitch_Angle_Value += DR16_PITCH_ANGLE / 1000;
        // // Pitch_Angle_Value +=((float)DJI_DR16_Data.Mouse.Y)/ -32767;
        // if (Pitch_Angle_Value > 0.095f)
        // {
        //     Pitch_Angle_Value = 0.095f;
        // }
        // else if (Pitch_Angle_Value < -0.054f)
        // {
        //     Pitch_Angle_Value = -0.054f;
        // }
        // Gimbal.Pitch.Angle_Tar = Pitch_Angle_Value;

#endif

        if (Gimbal.Yaw.Angle_Count > 0)
        {
            Gimbal.Yaw.Angle_Count--;
        }
        if (Gimbal.Yaw.Angle_Count == 0)
        {
            Gimbal.Yaw.Angle_Count = 5;

            // Yaw轴角度环
            Gimbal.Yaw.Pid_Angle.err = Gimbal.Yaw.Angle_Tar - YAW_ANGLE;
            Gimbal.Yaw.Pid_Angle.output = pid_error_input(&Gimbal.Yaw.Pid_Angle, Gimbal.Yaw.Pid_Angle.err);
            Gimbal.Yaw.Speed_Tar = Gimbal.Yaw.Pid_Angle.output;

            // Yaw轴速度环
            Gimbal.Yaw.Pid_Speed.err = Gimbal.Yaw.Speed_Tar - YAW_SPEED;
            Gimbal.Yaw.Pid_Speed.output = pid_error_input(&Gimbal.Yaw.Pid_Speed, Gimbal.Yaw.Pid_Speed.err);
            Gimbal.Yaw.Current_Tar = Gimbal.Yaw.Pid_Speed.output;
        }
        // Gimbal_Current_Tar_Filter(float Current_Input)

        Gimbal.Yaw.Current_Tar = Gimbal.Yaw.Pid_Speed.output;
        // current_tar_value=  Gimbal.Yaw.Current_Tar;
        current_tar_value = Gimbal_Current_Tar_Filter(Gimbal.Yaw.Current_Tar);
        // Yaw轴电流环
        // current_value = Gimbal_Current_Filter(YAW_CURRENT);
        current_value = YAW_CURRENT;
        // Gimbal.Yaw.Pid_Current.err = Gimbal.Yaw.Current_Tar - current_value;
        Gimbal.Yaw.Pid_Current.err = current_tar_value - current_value;
        Gimbal.Yaw.Pid_Current.output = pid_error_input(&Gimbal.Yaw.Pid_Current, Gimbal.Yaw.Pid_Current.err);
        if (Gimbal.Yaw.Pid_Current.output == 0.0f)
        {
            Yaw_Current_Out_Value = 0.0f;
            Yaw_Current_Out_Value_Last = 0.0f;
            Gimbal.Yaw.Pid_Current.output = 0.0f;
        }
        else if (Gimbal.Yaw.Pid_Current.output - Yaw_Current_Out_Value_Last > 0.1f)
        {
            Yaw_Current_Out_Value += 0.1f;
        }
        else if (Gimbal.Yaw.Pid_Current.output - Yaw_Current_Out_Value_Last < -0.1f)
        {
            Yaw_Current_Out_Value += -0.1f;
        }
        else if (Gimbal.Yaw.Pid_Current.output != 0.0f)
        {
            Yaw_Current_Out_Value = Gimbal.Yaw.Pid_Current.output;
        }
        Yaw_Current_Out_Value_Last = Yaw_Current_Out_Value;
        // Gimbal.Yaw.Pid_Current.output = pid_error_input(&Gimbal.Yaw.Pid_Current, Gimbal.Yaw.Pid_Current.err);
        // Yaw_Current_Out_Value=Gimbal_Current_Filter(Gimbal.Yaw.Pid_Current.output);

        if (Gimbal.Pitch.Angle_Count > 0)
        {
            Gimbal.Pitch.Angle_Count--;
        }
        if (Gimbal.Pitch.Angle_Count == 0)
        {
            Gimbal.Pitch.Angle_Count = 6;

            // Pitch轴角度环
            Gimbal.Pitch.Pid_Angle.err = Gimbal.Pitch.Angle_Tar - PITCH_ANGLE;
            Gimbal.Pitch.Pid_Angle.output = pid_error_input(&Gimbal.Pitch.Pid_Angle, Gimbal.Pitch.Pid_Angle.err);

            Gimbal.Pitch.Speed_Tar = Gimbal.Pitch.Pid_Angle.output;

            // Pitch轴速度环
            Gimbal.Pitch.Pid_Speed.err = Gimbal.Pitch.Speed_Tar - PITCH_SPEED;
            Gimbal.Pitch.Pid_Speed.output = pid_error_input(&Gimbal.Pitch.Pid_Speed, Gimbal.Pitch.Pid_Speed.err);

            // Gimbal.Pitch.Current_Tar = Gimbal.Pitch.Pid_Speed.output;
        }

        // Gimbal.Pitch.Current_Tar = Gimbal.Pitch.Pid_Speed.output;
        // // Pitch轴电流环
        // Gimbal.Pitch.Pid_Current.err = Gimbal.Pitch.Current_Tar - PITCH_CURRENT;
        // Gimbal.Pitch.Pid_Current.output = pid_error_input(&Gimbal.Pitch.Pid_Current, Gimbal.Pitch.Pid_Current.err);

        GIMBAL_SET_MOTOR_VALUE(
            Gimbal.Yaw.Pid_Current.output,
            // Yaw_Current_Out_Value,
            // 0,
            Gimbal.Pitch.Pid_Speed.output,
            0,
            0);

#if GIMBAL_VOFA_DEBUG
        if (Gimbal.Vofa_Count > 0)
        {
            Gimbal.Vofa_Count--;
        }
        else
        {
            Gimbal.Vofa_Count = 10;

            Vofa_Set_TX_Value(VOFA_TX_CURRENT, YAW_CURRENT);
            Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, Gimbal.Yaw.Current_Tar);
            // Vofa_Set_TX_Value(VOFA_TX_SPEED,Yaw_Current_Out_Value);
            //  Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR,Gimbal.Yaw.Pid_Current.output);
            Vofa_Set_TX_Value(VOFA_TX_SPEED, YAW_SPEED);
            Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Gimbal.Yaw.Speed_Tar);
            Vofa_Set_TX_Value(VOFA_TX_ANGLE, YAW_ANGLE);
            Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Gimbal.Yaw.Angle_Tar);

            // Vofa_Set_TX_Value(VOFA_TX_CURRENT, PITCH_CURRENT);
            // Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, Gimbal.Pitch.Current_Tar);
            // // Vofa_Set_TX_Value(VOFA_TX_SPEED, Gimbal.Pitch.Pid_Current.output);
            // Vofa_Set_TX_Value(VOFA_TX_SPEED, PITCH_SPEED);
            // Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Gimbal.Pitch.Speed_Tar);
            // Vofa_Set_TX_Value(VOFA_TX_ANGLE, PITCH_ANGLE);
            // Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Gimbal.Pitch.Angle_Tar);

            Vofa_Transmit();
        }

#else

#endif
        // if (Gimbal.Vofa_Count > 0)
        // {
        //     Gimbal.Vofa_Count--;
        // }
        // else
        // {
        //     Gimbal.Vofa_Count = 10;

        //     Vofa_Set_TX_Value(VOFA_TX_CURRENT, PITCH_CURRENT);
        //     Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, Gimbal.Pitch.Current_Tar);
        //     Vofa_Set_TX_Value(VOFA_TX_SPEED, PITCH_SPEED);
        //     Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Gimbal.Pitch.Speed_Tar);
        //     Vofa_Set_TX_Value(VOFA_TX_ANGLE, PITCH_ANGLE);
        //     Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Gimbal.Pitch.Angle_Tar);

        //     //     // Vofa_Set_TX_Value(VOFA_TX_CURRENT, ((float)DJI_DR16_Data.Mouse.X));
        //     //     // Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, (float)DJI_DR16_Data.Mouse.Y);
        //     //     // // Vofa_Set_TX_Value(VOFA_TX_SPEED, Gimbal.Yaw.Pid_Current.output);
        //     //     // Vofa_Set_TX_Value(VOFA_TX_SPEED, (float)DJI_DR16_Data.Mouse.Z / 32767);
        //     //     // Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, (float)DJI_DR16_Data.Mouse.Key_L);
        //     //     // Vofa_Set_TX_Value(VOFA_TX_ANGLE, (float)DJI_DR16_Data.Mouse.Key_R);
        //     //     // Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Yaw_Angle_Value);

        //     Vofa_Transmit();
        // }
        Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.RX_Flag = 0;
        Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_M2006_RX_6].Ret.RX_Flag = 0;
        Gimbal.Count = 5;
        break;
    }
}
