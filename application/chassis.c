#include "chassis.h"
#include "DJI_Motor.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "DJI_DR16.h"
#include "vofa.h"
#include "power.h"
#include <math.h>
#include "rm_referee.h"
#include "super_cap.h"

/*箭头方向则是电机速度为正的旋转方向

M2↓        M1↑

      o

M3↓        M4↑
*/

// 各个电机的转速，归一化为-1~1 对应RPM -9000~9000 对应Rad/s -942.47779607693793 ~ 942.47779607693793
#define M1_SPEED (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_1].Ret.Speed *SPEED_RPM_NORMALIZE
#define M2_SPEED (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_2].Ret.Speed *SPEED_RPM_NORMALIZE
#define M3_SPEED (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_3].Ret.Speed *SPEED_RPM_NORMALIZE
#define M4_SPEED (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_4].Ret.Speed *SPEED_RPM_NORMALIZE
// 各个电机的电流，归一化为-1~1
#define M1_CURRENT (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_1].Ret.Current
#define M2_CURRENT (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_2].Ret.Current
#define M3_CURRENT (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_3].Ret.Current
#define M4_CURRENT (float)Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_4].Ret.Current

// Yaw轴电机角度归一化为0~1 对应0-360度 但是记录了多圈所以不止360度
#define YAW_MOTOR_ANGLE (Motor_Group[DJI_MGRP2].Data[DJI_MGRP2_GM6020_RX_5].Ret.Angle_Sum_Process.Angle_Sum_Value)

// 云台相对于底盘的角度，逆时针旋转为正角度 归一化为0~1 对应0-360度 但是记录了多圈所以不止360度
#define GIMBAL_YAW_ANGLE (((YAW_MOTOR_ANGLE - Gimbal_Yaw_Angle_Offset) * -7.0f) / 13.0f)

#define X_VALUE Chassis.X_Speed_Tar
#define Y_VALUE Chassis.Y_Speed_Tar

#define COS_ALPHA() cos((PI / 4) - ((GIMBAL_YAW_ANGLE) * 2 * PI))
#define SIN_ALPHA() sin((PI / 4) - ((GIMBAL_YAW_ANGLE) * 2 * PI))

#define CHASSIS_SET_MOTOR_CURRENT(M1, M2, M3, M4) DJI_Motor_Set_Current_Value(DJI_MGRP1, DJI_Motor_TX_1_4, M1, M2, M3, M4)

#define CHASSIS_VOFA_DEBUG 0

// #define CHASSIA_LEVEL  referee_info.GameRobotState.robot_level

#if HAVE_REFEREE
#define CHASSIA_LEVEL referee_info.GameRobotState.robot_level
#else
#define CHASSIA_LEVEL 10
#endif

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
    Chassis.Mode = CHASSIS_FOLLOW;

    Chassis.M1_Speed = 0;
    PID_Init(&Chassis.M1_Pid_Speed);
    PID_Set(&Chassis.M1_Pid_Speed, 2.25, 0.06, 0.4, 0.35, 0.001, 1.0);

    Chassis.M2_Speed = 0;
    PID_Init(&Chassis.M2_Pid_Speed);
    PID_Set(&Chassis.M2_Pid_Speed, 2.25, 0.06, 0.4, 0.35, 0.001, 1.0);

    Chassis.M3_Speed = 0;
    PID_Init(&Chassis.M3_Pid_Speed);
    PID_Set(&Chassis.M3_Pid_Speed, 2.25, 0.06, 0.4, 0.35, 0.001, 1.0);

    Chassis.M4_Speed = 0;
    PID_Init(&Chassis.M4_Pid_Speed);
    PID_Set(&Chassis.M4_Pid_Speed, 2.25, 0.06, 0.4, 0.35, 0.001, 1.0);

    Chassis.Gimbal_Yaw_Angle_Tar = 0;
    PID_Init(&Chassis.Gimbal_Yaw_Pid_Angle);
    PID_Set(&Chassis.Gimbal_Yaw_Pid_Angle, 360, 0, 0, 0.03, 0.05, MOTOR_SPEED_RPM_MAX * RPM_TO_RADIAN * 0.5f);
    // 以下是底盘功率控制相关

    Low_Pass_Filter_Init(&Chassis.Power_Zeta_Filter, 0.1f, 0.0f);

    PID_Init(&Chassis.Power_Control_Pid);
    PID_Set(&Chassis.Power_Control_Pid, -1.0f, 0, 0, 0.03, 0.01, POWER_MX_LIMIT);

    Speed_Ramp_Init(&Chassis.X_Speed_Ramp, 2.0f, 30.0f, 1.0f);
    Speed_Ramp_Init(&Chassis.Y_Speed_Ramp, 2.0f, 30.0f, 1.0f);
    Speed_Ramp_Init(&Chassis.OMEGA_Speed_Ramp, PIX2 * 50.0f, PIX2 * 100.0f, PIX2 * 100.0f);

    Chassis.Power_Buffer_Tar = 50.0f;
    Chassis.Power_Mx_Limit = 30.0f;
    Chassis.Referee_Power_Mx_Limit = 50.0f;
    Chassis.Power_Mx_Add = 20.0F; // 超级模式的功率为20w
    Chassis.Power_Zeta = 1.0f;
} //
// 麦克纳姆轮正解算

void Chassis_Forward_Kinematics(void)
{
    float vx, vy, omega;

    // Calculate the chassis speed in the x and y directions and the angular velocity
    vx = (M1_SPEED + M2_SPEED - M3_SPEED - M4_SPEED) * MOTOR_SPEED_RPM_MAX * RPM_TO_RADIAN * (PERIMETER_WHEEL / 4.0f);
    vy = (M1_SPEED - M2_SPEED - M3_SPEED + M4_SPEED) * MOTOR_SPEED_RPM_MAX * RPM_TO_RADIAN * (PERIMETER_WHEEL / 4.0f);
    omega = ((M1_SPEED + M2_SPEED + M3_SPEED + M4_SPEED) * MOTOR_SPEED_RPM_MAX / MOTOR_GEAR_RATIO) * RPM_TO_RADIAN * (PERIMETER_WHEEL / (4.0f * (HALF_WHEEL_BASE + HALF_TRACK_WIDTH)));

    // Update the chassis structure with the calculated speeds
    Chassis.X_Speed = vx;
    Chassis.Y_Speed = vy;
    Chassis.OMEGA_Speed = omega * 100000.0f;
}

// 当小车直行的时候让底盘归位
void Chassis_Gimbal_Yaw_Err_Dz_Control(void)
{
    if ((fabs(Chassis.Y_Speed_Tar) > 0.01f) || (fabs(Chassis.X_Speed_Tar) > 0.01f))
    {
        Chassis.Gimbal_Yaw_Pid_Angle.err_dz = 0.001f;
    }
    else
    {
        Chassis.Gimbal_Yaw_Pid_Angle.err_dz = 0.05f;
    }
}

void Chassis_Set_Mode(Chassis_Mode_enum Mode)
{
    Chassis.Mode = Mode;
}
void Chassis_Set_Power_Mode(Chassis_Power_Mode_enum Mode)
{
    Chassis.Power_Mode = Mode;
}

void Chassis_Control(float X_Speed, float Y_Speed)
{
    Chassis.X_Speed_Tar = X_Speed;
    Chassis.Y_Speed_Tar = Y_Speed;
}

void Chassis_Timing_Handle(void)
{
    if (Chassis.Count > 0)
    {
        Chassis.Count--;
    }
}
// 获取裁判系统最大限制功率 单位：w
float Chassis_Get_Referee_Power_Mx_Limit(uint8_t Level)
{
    switch (Level)
    {
    case 1:
        return 45.0f;

    case 2:
        return 50.0f;

    case 3:
        return 55.0f;

    case 4:

        return 60.0f;

    case 5:
        return 65.0f;

    case 6:
        return 70.0f;

    case 7:
        return 75.0f;

    case 8:
        return 80.0f;

    case 9:
        return 90.0f;

    case 10:
        return 100.0f;

    default:
        return 45.0f;
    }
}

// 不同等级的设置（超电设置，速度曲线测试）
void Chassis_Level_Control(void)
{
    static uint16_t Count = 0;
    static uint16_t Flag = 0;
    uint16_t Super_Cup_Tar_Power;
    if (Count > 0)
    {
        Count--;
    }
    Chassis.Level = CHASSIA_LEVEL;
    // Chassis.Level = referee_info.GameRobotState.robot_level;
    //   测试用————————————————————————————————————
    // Chassis.Level =10;
    switch (Flag)
    {
    case 0:

        if (Chassis.Level != Chassis.Level_Last)
        {
            Super_Cup_Tar_Power = (uint16_t)(Chassis_Get_Referee_Power_Mx_Limit(Chassis.Level) * 100.0f) + 2000;
            Super_Cap_Set_Power(&Super_Cap, Super_Cup_Tar_Power);
            Count = 20;
        }
        Chassis.Level_Last = Chassis.Level;

        break;
    }
}

void Chassis_Power_Control(void)
{
    float Power_Zeta;

    Chassis.Motor_Power_Now[0] = Motor_3508_Calculat_Power(M1_SPEED * MOTOR_SPEED_RADIAN_MAX, M1_CURRENT * MOTOR_CURRENT_MAX);
    Chassis.Motor_Power_Now[1] = Motor_3508_Calculat_Power(M2_SPEED * MOTOR_SPEED_RADIAN_MAX, M2_CURRENT * MOTOR_CURRENT_MAX);
    Chassis.Motor_Power_Now[2] = Motor_3508_Calculat_Power(M3_SPEED * MOTOR_SPEED_RADIAN_MAX, M3_CURRENT * MOTOR_CURRENT_MAX);
    Chassis.Motor_Power_Now[3] = Motor_3508_Calculat_Power(M4_SPEED * MOTOR_SPEED_RADIAN_MAX, M4_CURRENT * MOTOR_CURRENT_MAX);
    Chassis.Power_Sum_Now = Calculat_Power_Sum(Chassis.Motor_Power_Now, sizeof(Chassis.Motor_Power_Now) / sizeof(float));

    // 测试用————————————————————————————————————
    //  Chassis.Level =10;

    Chassis.Referee_Power_Mx_Limit = Chassis_Get_Referee_Power_Mx_Limit(Chassis.Level); // 获取裁判系统底盘功率最大上限值
    Chassis.Power_Buffer = referee_info.PowerHeatData.chassis_power_buffer;             // 获取裁判系统当前功率缓冲能量

    Chassis.Power_Control_Pid.err = Chassis.Power_Buffer_Tar - Chassis.Power_Buffer;
    Chassis.Power_Control_Pid.output = pid_error_input(&Chassis.Power_Control_Pid, Chassis.Power_Control_Pid.err);

    // Chassis.Power_Control_Pid.err = Chassis.Power_Mx_Limit - Chassis.Power_Sum_Now;
    // Chassis.Power_Control_Pid.output = pid_error_input(&Chassis.Power_Control_Pid, Chassis.Power_Control_Pid.err);

    Chassis.Power_Mx_Limit_Sum = Chassis.Referee_Power_Mx_Limit +
                                 Chassis.Power_Control_Pid.output +
                                 ((Chassis.Power_Mode == POWER_SUPER) ? Chassis.Power_Mx_Add : 0.0f);

    Power_Zeta = Power_Calculat_Damping_Coefficient(Chassis.Power_Mx_Limit_Sum, Chassis.Power_Sum_Now);
    // Chassis.Power_Zeta = Power_Zeta;
    Chassis.Power_Zeta = Low_Pass_Filter(&Chassis.Power_Zeta_Filter, Power_Zeta); // 对输出值进行低通滤波
}

// 判断Yaw在结束小陀螺后的朝向，并给定合适的目标角度
float calculateYawAngle(float Gimbal_Yaw_Angle)
{
    int32_t Yaw_Angle_Decimal = (int32_t)(Gimbal_Yaw_Angle * 1000) % 1000; // 求小数部分
    int direct = 0;                                                        // 0表示朝前，1表示朝后
    float Gimbal_Yaw_Angle_Tar = 0;
    // 根据小数部分判断方向
    int remainder = Yaw_Angle_Decimal / 250;

    if (remainder == 0)
    {
        direct = 0;
    }
    else if (remainder == 1 || remainder == 2 || remainder == -1 || remainder == -2)
    {
        direct = 1;
    }
    else if (remainder == 3 || remainder == -3)
    {
        direct = 2;
    }
    // 计算目标角度
    switch (direct)
    {
    case 0:
        Gimbal_Yaw_Angle_Tar = GIMBAL_YAW_ANGLE - (Yaw_Angle_Decimal / 1000.0f);

        break;

    case 1:
        Gimbal_Yaw_Angle_Tar = GIMBAL_YAW_ANGLE - (Yaw_Angle_Decimal / 1000.0f) + (Yaw_Angle_Decimal > 0 ? 0.5f : -0.5f);
        break;

    case 2:
        Gimbal_Yaw_Angle_Tar = GIMBAL_YAW_ANGLE - (Yaw_Angle_Decimal / 1000.0f) + (Yaw_Angle_Decimal > 0 ? 1.0f : -1.0f);
        break;
    }
    return Gimbal_Yaw_Angle_Tar;
}

void Chassis_Task(void)
{

    if (Chassis.Count != 0)
    {
        return;
    }

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
        Chassis.Power_Control_Pid.startfalg = PID_ENABLE;

#endif
        break;

    case 1:
        Chassis.Count = 500;
        Gimbal_Yaw_Angle_Offset = YAW_MOTOR_ANGLE;
        Chassis.Gimbal_Yaw_Angle_Tar = GIMBAL_YAW_ANGLE;
        Chassis.Flag = 2;
        break;

    case 2:
#if CHASSIS_VOFA_DEBUG
        // Chassis.M1_Pid_Speed.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        // Chassis.M1_Pid_Speed.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        // Chassis.M1_Pid_Speed.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        // Chassis.M1_Speed = Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
        // Chassis.M1_Pid_Speed.output = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? Chassis.M1_Pid_Speed.output : 0;

        Chassis.M1_Pid_Speed.startfalg = PID_ENABLE;
        Chassis.M2_Pid_Speed.startfalg = PID_ENABLE;
        Chassis.M3_Pid_Speed.startfalg = PID_ENABLE;
        Chassis.M4_Pid_Speed.startfalg = PID_ENABLE;
        Chassis.Gimbal_Yaw_Pid_Angle.startfalg = PID_ENABLE;

        Chassis.Power_Control_Pid.kp = Vofa_Get_RX_Value(VOFA_RX_SPEED_KP);
        Chassis.Power_Control_Pid.ki = Vofa_Get_RX_Value(VOFA_RX_SPEED_KI);
        Chassis.Power_Control_Pid.kd = Vofa_Get_RX_Value(VOFA_RX_SPEED_KD);
        Chassis.Power_Mx_Limit = Vofa_Get_RX_Value(VOFA_RX_SPEED_TAR);
        Chassis.Power_Control_Pid.startfalg = Vofa_Get_RX_Value(VOFA_RX_SPEED_SW) ? PID_ENABLE : PID_DISABLE;

        Chassis.Referee_Power_Mx_Limit = Vofa_Get_RX_Value(VOFA_RX_CURRENT_TAR);

#else
        // 当 CHASSIS_VOFA_DEBUG 为 0 时，这里可以放置替代代码或留空
#endif
        // 当小车直行的时候让底盘归位的函数
        Chassis_Gimbal_Yaw_Err_Dz_Control();

        Chassis.Gimbal_Yaw_Angle_Tar = calculateYawAngle(GIMBAL_YAW_ANGLE); // 判断底盘的朝向
        Chassis.Gimbal_Yaw_Pid_Angle.err = Chassis.Gimbal_Yaw_Angle_Tar - GIMBAL_YAW_ANGLE;
        Chassis.Gimbal_Yaw_Pid_Angle.output = pid_error_input(&Chassis.Gimbal_Yaw_Pid_Angle, Chassis.Gimbal_Yaw_Pid_Angle.err);
        switch (Chassis.Mode_Flag)
        {
        case 1:
            if (Chassis.Mode == CHASSIS_PEG_TOP)
            {
                Chassis.Mode_Flag = 2;
            }
            else
            {
                OMEGA_VALUE = Chassis.Gimbal_Yaw_Pid_Angle.output;
            }

            break;
        case 2:
            if (Chassis.Mode == CHASSIS_FOLLOW)
            {

                // Chassis.Gimbal_Yaw_Angle_Tar = calculateYawAngle(GIMBAL_YAW_ANGLE); // 判断底盘的朝向
                Chassis.Mode_Flag = 3;
            }
            else
            {
                OMEGA_VALUE = PIX2 * 20.0f;
            }

            break;
        case 3:
            OMEGA_VALUE = 0;
            // PID_Set(&Chassis.Gimbal_Yaw_Pid_Angle, 3.5, 0, 0, 0.03, 0.001, 1.0);
            PID_Set(&Chassis.Gimbal_Yaw_Pid_Angle, 350.5, 0, 0, 0.03, 0.015, MOTOR_SPEED_RPM_MAX * RPM_TO_RADIAN * 0.07f);
            Chassis.Mode_Flag = 4;
            break;
        case 4:
            OMEGA_VALUE = Chassis.Gimbal_Yaw_Pid_Angle.output;
            // Chassis.Gimbal_Yaw_Angle_Tar = calculateYawAngle(GIMBAL_YAW_ANGLE); // 判断底盘的朝向
            if ((Chassis.Gimbal_Yaw_Angle_Tar - GIMBAL_YAW_ANGLE) < 0.04f && (Chassis.Gimbal_Yaw_Angle_Tar - GIMBAL_YAW_ANGLE) > -0.04f)
            {
                PID_Set(&Chassis.Gimbal_Yaw_Pid_Angle, 350.5, 0, 0, 0.03, 0.015, MOTOR_SPEED_RPM_MAX * RPM_TO_RADIAN * 0.5f);

                Chassis.Mode_Flag = 1;
            }

            break;

        default:
            break;
        }
        Chassis.X_Speed_Ramp.output_speed = Apply_Speed_Ramp(&Chassis.X_Speed_Ramp, X_VALUE, 0.006f);
        Chassis.Y_Speed_Ramp.output_speed = Apply_Speed_Ramp(&Chassis.Y_Speed_Ramp, Y_VALUE, 0.006f);
        Chassis.OMEGA_Speed_Ramp.output_speed = Apply_Speed_Ramp(&Chassis.OMEGA_Speed_Ramp, OMEGA_VALUE, 0.006f);
        // 麦克纳姆轮逆解算 求出四个轮子的目标转速
        Chassis.M1_Speed = (Chassis.X_Speed_Ramp.current_speed) * SIN_ALPHA() + (Chassis.Y_Speed_Ramp.current_speed) * COS_ALPHA() + (((Chassis.OMEGA_Speed_Ramp.current_speed) * RADIUS_WHEEL) / M1_L1) * MOTOR_GEAR_RATIO * RADIAN_TO_RPM / 9000.0f;
        Chassis.M2_Speed = (Chassis.X_Speed_Ramp.current_speed) * COS_ALPHA() - (Chassis.Y_Speed_Ramp.current_speed) * SIN_ALPHA() + (((Chassis.OMEGA_Speed_Ramp.current_speed) * RADIUS_WHEEL) / M2_L2) * MOTOR_GEAR_RATIO * RADIAN_TO_RPM / 9000.0f;
        Chassis.M3_Speed = -(Chassis.X_Speed_Ramp.current_speed) * SIN_ALPHA() - (Chassis.Y_Speed_Ramp.current_speed) * COS_ALPHA() + (((Chassis.OMEGA_Speed_Ramp.current_speed) * RADIUS_WHEEL) / M3_L3) * MOTOR_GEAR_RATIO * RADIAN_TO_RPM / 9000.0f;
        Chassis.M4_Speed = -(Chassis.X_Speed_Ramp.current_speed) * COS_ALPHA() + (Chassis.Y_Speed_Ramp.current_speed) * SIN_ALPHA() + (((Chassis.OMEGA_Speed_Ramp.current_speed) * RADIUS_WHEEL) / M4_L4) * MOTOR_GEAR_RATIO * RADIAN_TO_RPM / 9000.0f;

        // Chassis.M1_Speed = X_VALUE * SIN_ALPHA() + Y_VALUE * COS_ALPHA() + ((OMEGA_VALUE * RADIUS_WHEEL) / M1_L1) * MOTOR_GEAR_RATIO * RADIAN_TO_RPM / 9000.0f;
        // Chassis.M2_Speed = X_VALUE * COS_ALPHA() - Y_VALUE * SIN_ALPHA() + ((OMEGA_VALUE * RADIUS_WHEEL) / M2_L2) * MOTOR_GEAR_RATIO * RADIAN_TO_RPM / 9000.0f;
        // Chassis.M3_Speed = -X_VALUE * SIN_ALPHA() - Y_VALUE * COS_ALPHA() + ((OMEGA_VALUE * RADIUS_WHEEL) / M3_L3) * MOTOR_GEAR_RATIO * RADIAN_TO_RPM / 9000.0f;
        // Chassis.M4_Speed = -X_VALUE * COS_ALPHA() + Y_VALUE * SIN_ALPHA() + ((OMEGA_VALUE * RADIUS_WHEEL) / M4_L4) * MOTOR_GEAR_RATIO * RADIAN_TO_RPM / 9000.0f;

        // 底盘等级功能设置
        Chassis_Level_Control();
        // 底盘功率计算 算出衰减系数 Chassis.Power_Zeta
        Chassis_Power_Control();

        Chassis.M1_Pid_Speed.err = Chassis.M1_Speed * Chassis.Power_Zeta - M1_SPEED;
        Chassis.M1_Pid_Speed.output = pid_error_input(&Chassis.M1_Pid_Speed, Chassis.M1_Pid_Speed.err);

        Chassis.M2_Pid_Speed.err = Chassis.M2_Speed * Chassis.Power_Zeta - M2_SPEED;
        Chassis.M2_Pid_Speed.output = pid_error_input(&Chassis.M2_Pid_Speed, Chassis.M2_Pid_Speed.err);

        Chassis.M3_Pid_Speed.err = Chassis.M3_Speed * Chassis.Power_Zeta - M3_SPEED;
        Chassis.M3_Pid_Speed.output = pid_error_input(&Chassis.M3_Pid_Speed, Chassis.M3_Pid_Speed.err);

        Chassis.M4_Pid_Speed.err = Chassis.M4_Speed * Chassis.Power_Zeta - M4_SPEED;
        Chassis.M4_Pid_Speed.output = pid_error_input(&Chassis.M4_Pid_Speed, Chassis.M4_Pid_Speed.err);

        CHASSIS_SET_MOTOR_CURRENT(
            Chassis.M1_Pid_Speed.output,
            Chassis.M2_Pid_Speed.output,
            Chassis.M3_Pid_Speed.output,
            Chassis.M4_Pid_Speed.output);

        Chassis.OMEGA_Speed_Tar = OMEGA_VALUE;

        // 麦克纳姆轮正解算目标当前底盘的x，y速度和角速度
        Chassis_Forward_Kinematics();
#if CHASSIS_VOFA_DEBUG
        Vofa_Set_TX_Value(VOFA_TX_CURRENT, Chassis.Motor_Power_Now[0]);
        Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, Chassis.Power_Mx_Limit_Sum);
        Vofa_Set_TX_Value(VOFA_TX_SPEED, Chassis.Power_Sum_Now);
        Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Chassis.Power_Mx_Limit);
        Vofa_Set_TX_Value(VOFA_TX_ANGLE, Chassis.Power_Control_Pid.output);
        Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Chassis.Power_Zeta);
        Vofa_Transmit();
#else

        Vofa_Set_TX_Value(VOFA_TX_CURRENT, Chassis.Motor_Power_Now[0]);
        Vofa_Set_TX_Value(VOFA_TX_CURRENT_TAR, Chassis.Motor_Power_Now[1]);
        Vofa_Set_TX_Value(VOFA_TX_SPEED, Chassis.Motor_Power_Now[2]);
        Vofa_Set_TX_Value(VOFA_TX_SPEED_TAR, Chassis.Motor_Power_Now[3]);
        Vofa_Set_TX_Value(VOFA_TX_ANGLE, Chassis.Power_Zeta);
        Vofa_Set_TX_Value(VOFA_TX_ANGLE_TAR, Chassis.Power_Sum_Now);
        Vofa_Transmit();
#endif

        Chassis.Count = 6;
        break;

    default:
        break;
    }
}
