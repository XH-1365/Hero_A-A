/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 13:32:46
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-22 11:04:53
 * @FilePath: \RM_Template\application\shoot.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "shoot.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DJI_DR16.h"
#include "vofa.h"
#include "pid.h"
#include <math.h>

#define DR16_FEEDING_ANGLE DJI_DR16_Data.RC_Value.CH1 / 2
#define DR16_FRICTION_WHEEL DJI_DR16_Data.RC_Value.CH3

#define SHOOT_PILL_LIMIT (float)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
// #define SHOOT_SET_MOTOR_CURRENT(M1, M2, M3, M4) DJI_Motor_Set_Current_Value(DJI_MGRP3, DJI_Motor_TX_1_4, M1, M2, M3, M4)
#define SHOOT_SET_MOTOR_CURRENT(M1, M2, M3, M4) DJI_Motor_Set_Current_Value(DJI_MGRP2, DJI_Motor_TX_1_4, M1*0, M2*0, M3*0, M4*0)
// 发射一个弹丸电机所需旋转的角度
#define SHOOT_PILL_ANGLE_VALUE 2.37364f

// #define FEEDING_MOTOR_ANGLE Shoot_Get_Angle(&(Shoot.Feeding_Motor), Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M2006_RX_5].Ret.Angle, 1.0, 0.0)
#define FEEDING_MOTOR_ANGLE DM_Motor_4310.Ret_Value.Angle_Sum_Process.Angle_Sum_Value
#define FEEDING_MOTOR_SPEED DM_Motor_4310.Ret_Value.Speed
#define FEEDING_MOTOR_TPRQUE DM_Motor_4310.Ret_Value.Tprque

// #define RIGHT_WHEEL_SPEED (float)Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M3508_RX_1].Ret.Speed / 9000
#define RIGHT_WHEEL_SPEED 0.0f
// #define LEFT_WHEEL_SPEED (float)Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M3508_RX_2].Ret.Speed / 9000
 #define LEFT_WHEEL_SPEED 0.0f
 
#define SHOOT_VOFA_DEBUG 0

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

    float Angle_Value;
    float Angle_Turn;
    float Angle_Now;
    float Angle_Last;

    float Speed_Limit;  // 速度限定值
    float Tprque_Limit; // 力矩限定值

} Shoot_Feeding_Motor_Struct;

/// @brief 发射机结构体声明
typedef struct __Shoot_Struct
{
    uint8_t Flag;
    uint16_t Count;

    uint8_t Control_Flag;
    uint16_t Control_Count;

    uint16_t Vofa_Count;

    Shoot_Mode_enum Mode;

    Shoot_Feeding_Motor_Struct Feeding_Motor; // 弹丸拨盘电机
    Shoot_Friction_Wheel_Struct Right_Wheel;  // 头正对自己右电机摩擦轮
    Shoot_Friction_Wheel_Struct Left_Wheel;   // 头正对自己左电机摩擦轮
} Shoot_Struct;

Shoot_Struct Shoot;
float Feeding_Angle_Value = 0;
float Feeding_Angle_Value_Last = 0;
uint8_t Firing_Switch = 0;

float Shoot_Get_Angle(Shoot_Feeding_Motor_Struct *gm_motorx, float Angle_Value, float Max, float Min)
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

void Shoot_Init(void)
{
    Shoot.Count = 0;
    Shoot.Flag = 0;

    Shoot.Control_Count = 0;
    Shoot.Control_Flag = 0;

    PID_Init(&Shoot.Right_Wheel.Pid_Speed);
    PID_Set(&Shoot.Right_Wheel.Pid_Speed, 10, 0.01, 0.08, 0.35, 0.001, 1.0);

    PID_Init(&Shoot.Left_Wheel.Pid_Speed);
    PID_Set(&Shoot.Left_Wheel.Pid_Speed, 10, 0.01, 0.08, 0.35, 0.001, 1.0);

    PID_Init(&Shoot.Feeding_Motor.Pid_Speed);
    PID_Set(&Shoot.Feeding_Motor.Pid_Speed, 3, 0.02, 0.0, 0.35, 0.0001, 0.9);

    PID_Init(&Shoot.Feeding_Motor.Pid_Angle);
    PID_Set(&Shoot.Feeding_Motor.Pid_Angle, 0.1, 0, 0, 0.03, 0.0001, 0.9);
}

void Shoot_Set_Mode(Shoot_Mode_enum Mode)
{

    Shoot.Mode = Mode;
}

void Shoot_Set_Fire_Add_Speed(float Firing_Add_Speed)
{

    if (Firing_Add_Speed > 0.5f)
    {
        Firing_Switch = 1;
    }
    else
    {
        Firing_Switch = 0;
    }
    // Vofa_Set_TX_Value(VOFA_TX_CURRENT,(float) Firing_Switch);
    // if (SHOOT_PILL_LIMIT == 0)
    // {
    //     Feeding_Angle_Value += (Firing_Add_Speed < 0) ? 0 : Firing_Add_Speed;
    // }
    // else
    // {
    //     Feeding_Angle_Value += Firing_Add_Speed;
    // }
}

void Shoot_Timing_Handle(void)
{
    if (Shoot.Count > 0)
    {
        Shoot.Count--;
    }
    if (Shoot.Control_Count > 0)
    {
        Shoot.Control_Count--;
    }
}

void Shoot_Control_Handle(void)
{
    if (Shoot.Control_Count != 0)
    {
        return;
    }

    switch (Shoot.Control_Flag)
    {
    case 0:
#if SHOOT_VOFA_DEBUG

#else
        Shoot.Feeding_Motor.Pid_Angle.startfalg = PID_ENABLE;
        Shoot.Feeding_Motor.Pid_Speed.startfalg = PID_ENABLE;
        Shoot.Right_Wheel.Pid_Speed.startfalg = PID_ENABLE;
        Shoot.Left_Wheel.Pid_Speed.startfalg = PID_ENABLE;
#endif
        DM_Motor_Save_Angle_Zero(&DM_Motor_4310);
        DM_Motor_Set_State(&DM_Motor_4310, DISABLE);
        Shoot.Control_Flag = 1;
        Shoot.Control_Count = 1000;
        break;

    case 1:
        Shoot.Feeding_Motor.Speed_Limit = PI * 2;
        Shoot.Feeding_Motor.Tprque_Limit = 3.0f;
        DM_Motor_Save_Angle_Zero(&DM_Motor_4310);
        DM_Motor_Set_State(&DM_Motor_4310, ENABLE);
        Shoot.Control_Flag = 2;
        Shoot.Control_Count = 300;

        break;

    case 2:
#if SHOOT_VOFA_DEBUG

        // Shoot.Feeding_Motor.Pid_Angle.kp = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KP);
        // Shoot.Feeding_Motor.Pid_Angle.ki = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KI);
        // Shoot.Feeding_Motor.Pid_Angle.kd = Vofa_Get_RX_Value(VOFA_RX_ANGLE_KD);
        // Shoot.Feeding_Motor.Angle_Tar = Vofa_Get_RX_Value(VOFA_RX_ANGLE_TAR) * 36;
        // Shoot.Feeding_Motor.Pid_Angle.startfalg = Vofa_Get_RX_Value(VOFA_RX_ANGLE_SW) ? PID_ENABLE : PID_DISABLE;

        // Shoot.Feeding_Motor.Pid_Speed.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        // Shoot.Feeding_Motor.Pid_Speed.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        // Shoot.Feeding_Motor.Pid_Speed.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        // Shoot.Feeding_Motor.Speed_Tar = Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
        // Shoot.Feeding_Motor.Pid_Speed.startfalg = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? PID_ENABLE : PID_DISABLE;

        Shoot.Right_Wheel.Pid_Speed.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        Shoot.Right_Wheel.Pid_Speed.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        Shoot.Right_Wheel.Pid_Speed.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        Shoot.Right_Wheel.Speed_Tar = -Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
        Shoot.Right_Wheel.Pid_Speed.startfalg = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? PID_ENABLE : PID_DISABLE;

        // Shoot.Left_Wheel.Pid_Speed.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        // Shoot.Left_Wheel.Pid_Speed.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        // Shoot.Left_Wheel.Pid_Speed.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        // Shoot.Left_Wheel.Speed_Tar = Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
        // Shoot.Left_Wheel.Pid_Speed.startfalg = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? PID_ENABLE : PID_DISABLE;

#else

        // Shoot.Feeding_Motor.Angle_Tar += DR16_FEEDING_ANGLE * 36 / 10;
        // Shoot.Right_Wheel.Speed_Tar = -DR16_FRICTION_WHEEL;
        // Shoot.Left_Wheel.Speed_Tar = DR16_FRICTION_WHEEL;

        switch (Shoot.Mode)
        {
        case SHOOT_STOP_FIRE:
            Shoot.Right_Wheel.Speed_Tar = 0;
            Shoot.Left_Wheel.Speed_Tar = 0;
            Feeding_Angle_Value = Shoot.Feeding_Motor.Angle_Tar;
            break;

        case SHOOT_SINGE_FIRE:

            break;

        case SHOOT_BURST_FIRE:
            Shoot.Right_Wheel.Speed_Tar = -0.1;
            Shoot.Left_Wheel.Speed_Tar = 0.1;

            Shoot.Feeding_Motor.Angle_Tar = Feeding_Angle_Value;
            break;

        default:
            break;
        }

#endif

        Shoot.Feeding_Motor.Pid_Angle.err = Shoot.Feeding_Motor.Angle_Tar - FEEDING_MOTOR_ANGLE;
        Shoot.Feeding_Motor.Pid_Angle.output = pid_error_input(&Shoot.Feeding_Motor.Pid_Angle, Shoot.Feeding_Motor.Pid_Angle.err);

        Shoot.Feeding_Motor.Speed_Tar = Shoot.Feeding_Motor.Pid_Angle.output;

        Shoot.Feeding_Motor.Pid_Speed.err = Shoot.Feeding_Motor.Speed_Tar - FEEDING_MOTOR_SPEED;
        Shoot.Feeding_Motor.Pid_Speed.output = pid_error_input(&Shoot.Feeding_Motor.Pid_Speed, Shoot.Feeding_Motor.Pid_Speed.err);

        Shoot.Right_Wheel.Pid_Speed.err = Shoot.Right_Wheel.Speed_Tar - RIGHT_WHEEL_SPEED;
        Shoot.Right_Wheel.Pid_Speed.output = pid_error_input(&Shoot.Right_Wheel.Pid_Speed, Shoot.Right_Wheel.Pid_Speed.err);

        Shoot.Left_Wheel.Pid_Speed.err = Shoot.Left_Wheel.Speed_Tar - LEFT_WHEEL_SPEED;
        Shoot.Left_Wheel.Pid_Speed.output = pid_error_input(&Shoot.Left_Wheel.Pid_Speed, Shoot.Left_Wheel.Pid_Speed.err);

        SHOOT_SET_MOTOR_CURRENT(
            Shoot.Right_Wheel.Pid_Speed.output,
            Shoot.Left_Wheel.Pid_Speed.output,
            0,
            0);
        // 负值为向上供电，正值为退弹
        // DM_Motor_EMIT_Control(&DM_Motor_4310, Shoot.Feeding_Motor.Angle_Tar * PI, 628, 1500);
        // Shoot.Feeding_Motor.Angle_Tar = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        // Shoot.Feeding_Motor.Speed_Limit = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        // Shoot.Feeding_Motor.Tprque_Limit = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        DM_Motor_EMIT_Control(&DM_Motor_4310, Shoot.Feeding_Motor.Angle_Tar, Shoot.Feeding_Motor.Speed_Limit, Shoot.Feeding_Motor.Tprque_Limit);
#if SHOOT_VOFA_DEBUG
        if (Shoot.Vofa_Count > 0)
        {
            Shoot.Vofa_Count--;
        }
        else
        {
            Shoot.Vofa_Count = 1;

            // Vofa_Set_TX_Value(VOFA_TX_CURRENT, YAW_CURRENT);
            // Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, Gimbal.Yaw.Current_Tar);
            // Vofa_Set_TX_Value(VOFA_TX_SPEED, Gimbal.Yaw.Pid_Current.output);
            // Vofa_Set_TX_Value(VOFA_TX_SPEED, FEEDING_MOTOR_SPEED);
            // Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Shoot.Feeding_Motor.Speed_Tar);
            // Vofa_Set_TX_Value(VOFA_TX_ANGLE, FEEDING_MOTOR_ANGLE);
            // Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Shoot.Feeding_Motor.Angle_Tar);

            Vofa_Set_TX_Value(VOFA_TX_CURRENT, Shoot.Right_Wheel.Pid_Speed.output);
            Vofa_Set_TX_Value(VOFA_TX_SPEED, RIGHT_WHEEL_SPEED);
            Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Shoot.Right_Wheel.Speed_Tar);

            Vofa_Transmit();
        }
#else
        Vofa_Set_TX_Value(VOFA_TX_SPEED, RIGHT_WHEEL_SPEED);
        Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Shoot.Feeding_Motor.Speed_Tar);
        Vofa_Set_TX_Value(VOFA_TX_ANGLE, LEFT_WHEEL_SPEED);
        Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Shoot.Feeding_Motor.Angle_Tar);

        // Vofa_Set_TX_Value(VOFA_TX_SPEED, FEEDING_MOTOR_SPEED);
        // Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Shoot.Feeding_Motor.Speed_Tar);
        // Vofa_Set_TX_Value(VOFA_TX_ANGLE, FEEDING_MOTOR_ANGLE);
        // Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Shoot.Feeding_Motor.Angle_Tar);

        Vofa_Transmit();
#endif

        Vofa_Set_TX_Value(VOFA_TX_CURRENT, DM_Motor_4310.Ret_Value.Tprque);
        Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, Shoot.Feeding_Motor.Tprque_Limit);
        Vofa_Set_TX_Value(VOFA_TX_SPEED, DM_Motor_4310.Ret_Value.Speed);
        Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Shoot.Feeding_Motor.Speed_Limit);
        Vofa_Set_TX_Value(VOFA_TX_ANGLE, FEEDING_MOTOR_ANGLE);
        Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Shoot.Feeding_Motor.Angle_Tar);

        // Vofa_Set_TX_Value(VOFA_TX_CURRENT, Shoot.Feeding_Motor.Angle_Tar * PI);
        // Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, SHOOT_PILL_LIMIT);
        Vofa_Transmit();
        Shoot.Control_Count = 6;
        break;
    }
}

/// @brief 检测是否堵弹
/// @param
/// @return 1表示发射堵塞，-1表示回退堵塞,0表示没有堵塞
int8_t Shoot_Jamming_Detect(void)
{
    if (FEEDING_MOTOR_TPRQUE > 2.8f)
    {
        return -1;
    }
    else if (FEEDING_MOTOR_TPRQUE < -2.8f)
    {
        return 1;
    }
    return 0;
}

/// @brief
/// @param Deviation +-偏差
/// @return 返回1为
uint8_t Shoot_Angle_Detect(float Deviation)
{
    float Temp = 0;
    Temp = Shoot.Feeding_Motor.Angle_Tar - FEEDING_MOTOR_ANGLE;
    if (Temp > 0 && (Temp < Deviation))
    {
        return 1;
    }
    else if (Temp < 0 && (Temp > Deviation))
    {
        return 1;
    }

    return 0;
}

void Shoot_Handle(void)
{
    if (Shoot.Count != 0)
    {
        return;
    }

    switch (Shoot.Flag)
    {

    case 0: // 复位态
        if (Shoot.Control_Flag == 2)
        {
            Shoot.Flag = 1;
        }
        Shoot.Count = 100;

        break;

    case 1:
        if (SHOOT_PILL_LIMIT == 0)
        {
            DM_Motor_Save_Angle_Zero(&DM_Motor_4310);
            Feeding_Angle_Value = 0;
            Shoot.Count = 2;
            Shoot.Flag = 2;
        }
        else
        {
            Feeding_Angle_Value += -0.0225f;
            Shoot.Count = 5;
        }
        break;

    case 2:
        Feeding_Angle_Value += -2.374f;
        Shoot.Count = 5;
        Shoot.Flag = 3;
        break;

    case 3: // 就绪态

        if (Firing_Switch == 1)
        {
            Shoot.Flag = 4;
        }
        else if (Shoot_Jamming_Detect() == 1)
        {
            Feeding_Angle_Value_Last = Feeding_Angle_Value;
            Feeding_Angle_Value = FEEDING_MOTOR_ANGLE;
            Shoot.Flag = 5;
        }
        Shoot.Count = 5;
        break;

    case 4: // 发射态
        if (Firing_Switch == 1)
        {
            Feeding_Angle_Value += -0.0225f;
        }
        else
        {
            Shoot.Flag = 3;
        }

        // 检测是否堵弹如果堵弹则跳转到堵弹态
        if (Shoot_Jamming_Detect() == 1)
        {
            Feeding_Angle_Value_Last = Feeding_Angle_Value;
            Feeding_Angle_Value = FEEDING_MOTOR_ANGLE;
            Shoot.Flag = 5;
        }
        Shoot.Count = 5;
        break;

    case 5: // 堵弹态
        Feeding_Angle_Value += SHOOT_PILL_ANGLE_VALUE / 2;
        Shoot.Count = 100;
        Shoot.Flag = 6;
        break;

    case 6:
        if (Shoot_Angle_Detect(0.1))
        {
            Shoot.Flag = 7;
        }
        else if (Shoot_Jamming_Detect() == -1)
        {
            Shoot.Flag = 7;
        }
        Shoot.Count = 1;
        break;

    case 7:
        Feeding_Angle_Value = Feeding_Angle_Value_Last + SHOOT_PILL_ANGLE_VALUE ;
        Shoot.Flag = 8;
        Shoot.Count = 5;
        break;

    case 8:
        Shoot.Flag = 3;
        Shoot.Count = 5;
        break;
    }
}
void Shoot_Task(void)
{
    Shoot_Handle();
    Shoot_Control_Handle();
}
