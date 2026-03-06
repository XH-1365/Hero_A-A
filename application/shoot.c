/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 13:32:46
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-05-19 22:35:41
 * @FilePath: \RM_Template\application\shoot.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "shoot.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DJI_DR16.h"
#include "vofa.h"
#include <math.h>
#include "gpio.h"
#include "super_cap.h"
#include "filter.h"
#include "can_comm.h"
#define DR16_FEEDING_ANGLE DJI_DR16_Data.RC_Value.CH1 / 2
#define DR16_FRICTION_WHEEL DJI_DR16_Data.RC_Value.CH3

#define SHOOT_PILL_LIMIT_SW (float)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
#define SHOOT_SET_MOTOR_CURRENT(M1, M2, M3, M4) DJI_Motor_Set_Current_Value(DJI_MGRP3, DJI_Motor_TX_1_4, M1, M2, M3, M4)
// 发射一个弹丸电机所需旋转的角度
#define SHOOT_PILL_ANGLE_VALUE 2.37364778528888f

#define FEEDING_MOTOR_ANGLE DM_Motor_4310.Ret_Value.Angle_Sum_Process.Angle_Sum_Value
#define FEEDING_MOTOR_SPEED DM_Motor_4310.Ret_Value.Speed
#define FEEDING_MOTOR_TPRQUE DM_Motor_4310.Ret_Value.Tprque

#define RIGHT_WHEEL_SPEED (float)Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M3508_RX_1].Ret.Speed / 9000
#define LEFT_WHEEL_SPEED (float)Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M3508_RX_2].Ret.Speed / 9000

#define SHOOT_VOFA_DEBUG 0

// #define SHOOT_HEAT UP_Board_RX_Data.Data.Shoot_Heat
#define SHOOT_HEAT Heat

#if HAVE_REFEREE
#define SHOOT_LEVEL UP_Board_RX_Data.Data.Robot_Level
#else
#define SHOOT_LEVEL 10
#endif

// #define SHOOT_LEVEL 5
#define SHOOT_HEAT_MARGIN_TIME (108 - SHOOT_LEVEL * 8)
#define SHOOT_HEAT_LIMT (60 + (SHOOT_LEVEL * 40))
// 冷却速率热量每秒
#define SHOOT_COOL_RATE ((SHOOT_LEVEL == 1) ? 70 : ((32 + ((SHOOT_LEVEL == 10) ? 88 : (SHOOT_LEVEL * 8))) * 2) - 20)
// 冷却时间余量单位ms
#define SHOOT_COOL_OFFSET 100
//-9000~9000归一化到-1~1  0.93=8400rpm
#define SHOOT_WHELL_SPEED_LIMIT 0.98f
// #define SHOOT_WHELL_SPEED_LIMIT 0.38f
// 供弹最大扭矩
#define SHOOT_FORWARD_TPRQUE 5.0f
// 供弹最大转速
#define SHOOT_FORWARD_SPEED PI * 20.0f
// 反转最大扭矩
#define SHOOT_REVERSAL_TPRQUE 0.5f
// 反转最大转速
#define SHOOT_REVERSAL_SPEED PI * 4.0f

Shoot_Struct Shoot = {0};
float Feeding_Angle_Value = 0;
float Feeding_Angle_Value_Last = 0;
int8_t Firing_Switch = 0;
uint8_t Oline_Flag_Last = 0;
int16_t Heat_Diff = 0;
float Heat = 0;
float Heat_Reduction = 0;
uint8_t Heat_Flag = 0;
uint16_t Heat_Count = 0;
void Shoot_Heat_Add(void)
{
    Heat += 100.0f;
}

void Shoot_Enable_DM(void)
{
    Shoot.DM_Enable_Flag = 1;
}
// 打开弹丸检测
void Shoot_Open_Ball_Detect()
{
    Heat_Flag = 1;
}

uint8_t Shoot_Get_Ball_Output()
{
    if (Heat_Flag == 3) // 弹丸已射出
    {
        return 3;
    }
    else if (Heat_Flag == 2) // 当进入弹丸检测状态
    {
        return 2;
    }

    return 0;
}
void Shoot_Heat_Control_Handle(void)
{
    if (Heat_Count > 0)
    {
        return;
    }
    switch (Heat_Flag)
    {
    case 0:
        Heat_Flag = 1;
        break;

    case 1:
        if ((RIGHT_WHEEL_SPEED < (-SHOOT_WHELL_SPEED_LIMIT + 0.06f)) && (LEFT_WHEEL_SPEED > (SHOOT_WHELL_SPEED_LIMIT - 0.06f)))
        {
            Heat_Flag = 2;
        }
        break;

    case 2:
        if (Shoot.Mode == SHOOT_STOP_FIRE)
        {
            Heat_Flag = 1;
        }
        else if (((RIGHT_WHEEL_SPEED > (-SHOOT_WHELL_SPEED_LIMIT + 0.08f)) && (RIGHT_WHEEL_SPEED < (-SHOOT_WHELL_SPEED_LIMIT + 0.3f))) &&
                 ((LEFT_WHEEL_SPEED < (SHOOT_WHELL_SPEED_LIMIT - 0.08f)) && (LEFT_WHEEL_SPEED > (SHOOT_WHELL_SPEED_LIMIT - 0.3f))))
        {
            Shoot_Heat_Add();
            Feeding_Angle_Value = FEEDING_MOTOR_ANGLE; // 拨盘电机停止

            Heat_Flag = 3;
        }
        break;

    case 3:
        break;
    }
}
void Shoot_Heat_Timing(void)
{

    if (Heat_Count > 0)
    {
        Heat_Count--;
    }

    Shoot_Heat_Control_Handle();

    if (Heat > 0)
    {
        if ((Heat - ((float)(SHOOT_COOL_RATE) / 1000.0f)) < 0)
        {
            Heat = 0;
        }
        else
        {
            Heat -= ((float)(SHOOT_COOL_RATE) / 1000.0f);
        }
    }
}

void Shoot_Init(void)
{
    Shoot.Count = 0;
    Shoot.Flag = 0;

    Shoot.Control_Count = 0;
    Shoot.Control_Flag = 0;

    Shoot.DM_Control_Count = 0;
    Shoot.DM_Control_Flag = 0;

    PID_Init(&Shoot.Right_Wheel.Pid_Speed);
    PID_Set(&Shoot.Right_Wheel.Pid_Speed, 10, 0.001, 0.08, 2.0, 0.001, 1.0);

    PID_Init(&Shoot.Left_Wheel.Pid_Speed);
    PID_Set(&Shoot.Left_Wheel.Pid_Speed, 10, 0.001, 0.08, 2.0, 0.001, 1.0);

    PID_Init(&Shoot.Heat_Control.Pid_Heat);
    PID_Set(&Shoot.Heat_Control.Pid_Heat, 0.0005f, 0, 0, 0.03, 0.0001, 0.1);

    // SHOOT_OPEN_LASER(); // 打开激光
}

// 达妙电机复位
void Shoot_DM_Motor_Auto_RST(void)
{
    static uint8_t Flag = 0;
    static uint16_t Count = 0;

    if (Count > 0)
    {
        Count--;
    }
    if (Count > 0)
    {
        return;
    }

    switch (Flag)
    {
    case 0:
        Flag = 1;
        break;

    case 1:

        if (DM_Motor_4310.Daemon->Online_Flag == 0)
        {

            DM_Motor_Clear_Error(&DM_Motor_4310);

            Count = 20;
            Flag = 2;
        }
        break;
    case 2:

        DM_Motor_Set_State(&DM_Motor_4310, ENABLE);

        Count = 20;
        Flag = 3;

        break;
    case 3:
        DM_Motor_Save_Angle_Zero(&DM_Motor_4310);
        DM_Motor_Clean_Angle_Sum(&(DM_Motor_4310.Ret_Value.Angle_Sum_Process));
        Feeding_Angle_Value = FEEDING_MOTOR_ANGLE;
        Shoot.Feeding_Motor.Angle_Tar = Feeding_Angle_Value;
        Flag = 1;
        Count = 100;
        break;
    }
}

void Shoot_Set_Mode(Shoot_Mode_enum Mode)
{

    Shoot.Mode = Mode;
}

uint8_t Shoot_Get_Mode(void)
{
    return Shoot.Mode;
}

// 返回1为发射，返回-1为回退，返回0则不发射
void Shoot_Set_Fire_Add_Speed(float Firing_Add_Speed)
{

    if (Firing_Add_Speed > 0.5f)
    {
        Firing_Switch = 1;
    }
    else if (Firing_Add_Speed < -0.5f)
    {
        Firing_Switch = -1;
    }
    else
    {
        Firing_Switch = 0;
    }
}

void Shoot_Timing_Handle(void)
{
    if (Shoot.Count > 0)
    {
        Shoot.Count--;
    }

    if (Shoot.Vofa_Count > 0)
    {
        Shoot.Vofa_Count--;
    }

    if (Shoot.DM_Control_Count > 0)
    {
        Shoot.DM_Control_Count--;
    }

    Shoot_Heat_Timing();
    // if (Shoot.Control_Count > 0)
    // {
    //     Shoot.Control_Count--;
    // }
}

void Shoot_DM_Control_Handle(void)
{
    static uint8_t Enable_Count = 0; // 使能计数
    if (Shoot.DM_Control_Count > 0)
    {
        return;
    }

    switch (Shoot.DM_Control_Flag)
    {
    case 0:
        Shoot.DM_Control_Flag = 1;
        Shoot.DM_Control_Count = 500;
        break;

    case 1:
        DM_Motor_Clear_Error(&DM_Motor_4310);
        Shoot.DM_Control_Flag = 2;
        Shoot.DM_Control_Count = 10;
        break;
    case 2:
        DM_Motor_Set_State(&DM_Motor_4310, ENABLE);

        Shoot.DM_Control_Flag = 3;
        Shoot.DM_Control_Count = 10;
        break;

    case 3:
        // DM_Motor_Save_Angle_Zero(&DM_Motor_4310);
        // DM_Motor_Clean_Angle_Sum(&(DM_Motor_4310.Ret_Value.Angle_Sum_Process));

        Shoot.Feeding_Motor.Speed_Limit = SHOOT_FORWARD_SPEED;
        Shoot.Feeding_Motor.Tprque_Limit = SHOOT_FORWARD_TPRQUE;

        Feeding_Angle_Value = FEEDING_MOTOR_ANGLE;

        Shoot.DM_Control_Flag = 4;
        Shoot.DM_Control_Count = 10;
        break;

    case 4:

        DM_Motor_Set_State(&DM_Motor_4310, DISABLE);
        Shoot.DM_Control_Flag = 5;
        Shoot.DM_Control_Count = 10;

        break;

    case 5:
        if (Enable_Count < 10)
        {
            DM_Motor_Set_State(&DM_Motor_4310, ENABLE);
            Enable_Count++;
            Shoot.DM_Control_Count = 10;
        }
        else if (Enable_Count == 10)
        {
            Enable_Count = 0;
            Shoot.DM_Control_Flag = 6;
            Shoot.DM_Control_Count = 10;
        }

        break;

    case 6:
        DM_Motor_Save_Angle_Zero(&DM_Motor_4310);
        Shoot.DM_Control_Flag = 7;
        Shoot.DM_Control_Count = 10;
        break;

    case 7:
        DM_Motor_Read_Data_Cmd(&DM_Motor_4310);

        Shoot.DM_Control_Flag = 8;
        Shoot.DM_Control_Count = 30;
        break;

    case 8:
        DM_Motor_Set_State(&DM_Motor_4310, ENABLE);
        DM_Motor_Clean_Angle_Sum(&(DM_Motor_4310.Ret_Value.Angle_Sum_Process));
        Shoot.Feeding_Motor.Speed_Limit = SHOOT_FORWARD_SPEED;
        Shoot.Feeding_Motor.Tprque_Limit = SHOOT_FORWARD_TPRQUE;
        Feeding_Angle_Value = FEEDING_MOTOR_ANGLE;
        Shoot.Feeding_Motor.Angle_Tar = Feeding_Angle_Value;
        Daemon_Reload(DM_Motor_4310.Daemon);

        Shoot.DM_Control_Flag = 9;
        Shoot.DM_Control_Count = 30;
        break;

    case 9:
        switch (Shoot.Mode)
        {
        case SHOOT_STOP_FIRE:

            Shoot.Feeding_Motor.Angle_Tar = Feeding_Angle_Value;
            break;

        case SHOOT_BURST_FIRE:

            Shoot.Feeding_Motor.Angle_Tar = Feeding_Angle_Value;
            break;

        case SHOOT_SINGE_FIRE:

            Shoot.Feeding_Motor.Angle_Tar = Feeding_Angle_Value;

            break;

        default:
            break;
        }

        DM_Motor_EMIT_Control(&DM_Motor_4310, Shoot.Feeding_Motor.Angle_Tar, Shoot.Feeding_Motor.Speed_Limit, Shoot.Feeding_Motor.Tprque_Limit);
        if ((DM_Motor_4310.Daemon->Online_Flag == 0) || (DM_Motor_4310.Ret_Data.Data.ERR != 1))
        {
            Shoot.DM_Control_Flag = 1;
            Shoot.DM_Control_Count = 100;
        }
        else if (Shoot.DM_Enable_Flag >= 1)
        {
            Shoot.DM_Enable_Flag = 0;
            Shoot.DM_Control_Flag = 1;
            Shoot.DM_Control_Count = 100;
        }
        else
        {
            Shoot.DM_Control_Count = 10;
        }

        // Shoot_DM_Motor_Auto_RST();
        break;

    default:
        break;
    }
}

void Shoot_Control_Handle(void)
{
    if (Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M3508_RX_1].Ret.RX_Flag != 1 || Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M3508_RX_2].Ret.RX_Flag != 1)
    {
        return;
    }

    Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M3508_RX_1].Ret.RX_Flag = 0;
    Motor_Group[DJI_MGRP3].Data[DJI_MGRP3_M3508_RX_2].Ret.RX_Flag = 0;
    if (Shoot.Control_Count > 0)
    {
        Shoot.Control_Count--;
    }
    if (Shoot.Control_Count != 0)
    {
        return;
    }

    switch (Shoot.Control_Flag)
    {
    case 0:

        Shoot.Feeding_Motor.Pid_Angle.startfalg = PID_ENABLE;
        Shoot.Feeding_Motor.Pid_Speed.startfalg = PID_ENABLE;
        Shoot.Right_Wheel.Pid_Speed.startfalg = PID_ENABLE;
        Shoot.Left_Wheel.Pid_Speed.startfalg = PID_ENABLE;
        Shoot.Heat_Control.Pid_Heat.startfalg = PID_ENABLE;

        Shoot.Control_Flag = 1;
        Shoot.Control_Count = 200;
        break;

    case 1:

        switch (Shoot.Mode)
        {
        case SHOOT_STOP_FIRE:
            Shoot.Right_Wheel.Speed_Tar = 0;
            Shoot.Left_Wheel.Speed_Tar = 0;
            // Feeding_Angle_Value = FEEDING_MOTOR_ANGLE;
            break;

        case SHOOT_BURST_FIRE:
            Shoot.Right_Wheel.Speed_Tar = -SHOOT_WHELL_SPEED_LIMIT;
            Shoot.Left_Wheel.Speed_Tar = SHOOT_WHELL_SPEED_LIMIT;

            break;

        case SHOOT_SINGE_FIRE:
            Shoot.Right_Wheel.Speed_Tar = -SHOOT_WHELL_SPEED_LIMIT;
            Shoot.Left_Wheel.Speed_Tar = SHOOT_WHELL_SPEED_LIMIT;

            break;

        default:
            break;
        }

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

        Shoot.Control_Count = 6;
        break;
    }
}

/// @brief 检测是否堵弹
/// @param
/// @return 1表示发射堵塞，-1表示回退堵塞,0表示没有堵塞
int8_t Shoot_Jamming_Detect(void)
{

    // 此处还需改进，需要判断速度是否也达到堵弹的标准，且目标值和实际值的差距是否在阈值内
    if (FEEDING_MOTOR_TPRQUE > SHOOT_REVERSAL_TPRQUE + 0.2f)
    {
        return -1;
    }
    else if (FEEDING_MOTOR_TPRQUE < -(SHOOT_FORWARD_TPRQUE + 0.2f))
    {
        return 1;
    }
    return 0;
}

/// @brief 判断电机是否旋转到位
/// @param Deviation +-偏差
/// @return 返回1为位置达到目标值
uint8_t Shoot_Angle_Detect(float Deviation)
{
    float Temp = 0;
    Temp = Feeding_Angle_Value - FEEDING_MOTOR_ANGLE;
    if ((Temp > 0.0f) && (Temp < Deviation))
    {
        return 1;
    }
    else if ((Temp < 0.0f) && (Temp > (Deviation * (-1.0f))))
    {
        return 1;
    }

    return 0;
}

void Shoot_Handle(void)
{

    if (DM_Motor_4310.Daemon->Online_Flag != 1)
    {
        return;
    }
    switch (Shoot.Flag)
    {

    case 0: // 复位态
        if (Shoot.Count > 0)
        {
            return;
        }
        if ((Shoot.Control_Flag == 1))
        {
            if (SHOOT_PILL_LIMIT_SW == 0)
            {
                Shoot.Flag = 3;
            }
            else
            {
                Shoot.Flag = 1;
            }

            Shoot.Count = 100;
        }

        break;

    case 1:
        if (Shoot.Count > 0)
        {
            return;
        }
        // 当接了限位开关的时候取消注释
        if (SHOOT_PILL_LIMIT_SW == 0)
        {
            Shoot.Count = 2;
            Shoot.Flag = 2;
        }
        else
        {
            Feeding_Angle_Value -= 0.03f;

            Shoot.Count = 5;
        }
        break;

    case 2:
        if (Shoot.Count > 0)
        {
            return;
        }
        Feeding_Angle_Value -= SHOOT_PILL_ANGLE_VALUE / 4;
        Shoot.Count = 100;
        Shoot.Flag = 3;
        break;

    case 3: // 就绪态
        if (Shoot.Count > 0)
        {
            return;
        }
        if (Shoot.Mode != SHOOT_SINGE_FIRE)
        {
            return;
        }

        if (Firing_Switch == 1)
        {
            if (((SHOOT_HEAT_LIMT - SHOOT_HEAT) >= 100) && (Shoot_Get_Ball_Output() == 2)) // 当热量在合理范围内时才可以发射
            {
                //
                // if (1)
                // {

                Feeding_Angle_Value -= SHOOT_PILL_ANGLE_VALUE;

                Shoot.Feeding_Motor.Speed_Limit = SHOOT_FORWARD_SPEED;
                Shoot.Feeding_Motor.Tprque_Limit = SHOOT_FORWARD_TPRQUE;
                Shoot.Flag = 4;
                Shoot.Count = SHOOT_HEAT_MARGIN_TIME;
            }
            else if (Shoot_Get_Ball_Output() == 3)
            {
                Shoot_Open_Ball_Detect();
                Shoot.Count = SHOOT_HEAT_MARGIN_TIME;
            }
            else
            {
                Shoot.Count = 20;
            }
        }
        else if (Firing_Switch == -1)
        {

            Shoot.Feeding_Motor.Speed_Limit = SHOOT_REVERSAL_SPEED;
            Shoot.Feeding_Motor.Tprque_Limit = SHOOT_REVERSAL_TPRQUE;
            Feeding_Angle_Value += 0.06f;
            Shoot.Count = 10;
        }
        else if (Firing_Switch == 0)
        {

            // Feeding_Angle_Value = FEEDING_MOTOR_ANGLE;
            Shoot.Count = 10;
        }

        break;

    case 4: // 发射态

        if ((SHOOT_HEAT_LIMT - SHOOT_HEAT) < 100) // 当前热量超限的时候电机立刻停止
        {
            Feeding_Angle_Value = FEEDING_MOTOR_ANGLE; // 拨盘电机停止
        }

        if (Shoot.Count == 0)
        {
            if ((Firing_Switch == 0))
            {
                Shoot_Open_Ball_Detect();
                Shoot.Count = SHOOT_HEAT_MARGIN_TIME;
                Shoot.Flag = 3;
            }
            else
            {
                Shoot.Count = 10;
            }
        }

        break;
    }
}
void Shoot_Task(void)
{
    Shoot_Handle();
    Shoot_Control_Handle();
    Shoot_DM_Control_Handle();
    if (Shoot.Vofa_Count == 0)
    {
        Shoot.Vofa_Count = 10;

//        Vofa_Set_TX_Value(VOFA_TX_CURRENT, SHOOT_HEAT_LIMT);
//        Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, SHOOT_HEAT);
//        Vofa_Set_TX_Value(VOFA_TX_SPEED, RIGHT_WHEEL_SPEED);
//        Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Heat_Flag);
//        Vofa_Set_TX_Value(VOFA_TX_ANGLE, FEEDING_MOTOR_ANGLE);
//        Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Shoot.Feeding_Motor.Angle_Tar);
//        Vofa_Transmit(); //
    }
}
