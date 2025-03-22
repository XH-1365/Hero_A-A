/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 14:32:58
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-17 11:15:09
 * @FilePath: \RM_Template\application\robot_cmd.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_cmd.h"
#include "DJI_DR16.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "can_comm.h"
#include "rm_referee.h"
typedef struct __Robot_Cmd_Struct
{
    uint8_t Flag;
    uint16_t Count;

    float Yaw_Angle_Value;
    float Pitch_Angle_Value;
} Robot_Cmd_Struct;

Robot_Cmd_Struct Robot_Cmd;
Chassis_Control_Struct Chassis_Control_Data;
Gimbal_Tx_Struct Gimbal_Tx_Data;
void Robot_Cmd_Init(void)
{
    Robot_Cmd.Flag = 0;
    Robot_Cmd.Count = 0;
}

void Robot_Cmd_Timing_Handle(void)
{
    if (Robot_Cmd.Count > 0)
    {
        Robot_Cmd.Count--;
    }
}

void Remote_Control_Set()
{

    // CAN_Comm_TX_Send_Data(&Down_Board, (uint8_t *)Down_Board_TX_Data, 8);
    Gimbal_Tx_Data.Chassis_OMEGA_Speed = Chassis.OMEGA_Speed;
    Gimbal_Tx_Data.Shoot_Heat = referee_info.PowerHeatData.shooter_heat_42mm;
    Gimbal_Tx_Data.Robot_Level = referee_info.GameRobotState.robot_level;
    CAN_Comm_TX_Send_Data(&Down_Board, (uint8_t *)&Gimbal_Tx_Data, 7); // 给云台发送底盘角速度用于云台补偿

    Chassis_Control_Data = *(Chassis_Control_Struct *)CAN_Comm_Get_RX_Data(&Down_Board);
    Chassis_Control(
        ((float)Chassis_Control_Data.X_Speed / 1000) / 1,
        ((float)Chassis_Control_Data.Y_Speed / 1000) / -1);
    // Chassis_Control(
    //     DJI_DR16_Data.RC_Value.CH0 / 2,
    //     DJI_DR16_Data.RC_Value.CH1 / -2);

    // switch (DJI_DR16_Data.RC_Value.S1)
    switch (Chassis_Control_Data.Mode)
    {
    case 2:
        Chassis_Set_Mode(CHASSIS_PEG_TOP);

        break;

    case 3:
        Chassis_Set_Mode(CHASSIS_FOLLOW);
        break;

    default:
        break;
    }

    Chassis_Set_Power_Mode(Chassis_Control_Data.Super_Mode);
}

void Mouse_Key_Set()
{
}

void Robot_Cmd_Task(void)
{
    if (Robot_Cmd.Count > 0)
    {
        return;
    }
    Robot_Cmd.Count = 5;
    Remote_Control_Set();
    // Mouse_Key_Set();
}
