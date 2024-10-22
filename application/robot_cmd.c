/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 14:32:58
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-22 15:26:55
 * @FilePath: \RM_Template\application\robot_cmd.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_cmd.h"
#include "DJI_DR16.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "can_comm.h"
typedef struct __Robot_Cmd_Struct
{
    uint8_t Flag;
    uint16_t Count;

    float Yaw_Angle_Value;
    float Pitch_Angle_Value;
} Robot_Cmd_Struct;

Robot_Cmd_Struct Robot_Cmd;
Chassis_Control_Struct Chassis_Control_Data;
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

    Gimbal_Add_Angle(
        DJI_DR16_Data.RC_Value.CH2 / -45,
        DJI_DR16_Data.RC_Value.CH3 * 2.0f);

    // CAN_Comm_TX_Send_Data(&Down_Board, (uint8_t *)Down_Board_TX_Data, 8);
    Chassis_Control_Data = *(Chassis_Control_Struct *)CAN_Comm_Get_RX_Data(&Down_Board);
    Chassis_Control(
        ((float)Chassis_Control_Data.X_Speed / 1000) / 2,
        ((float)Chassis_Control_Data.Y_Speed / 1000) / -2);
    // Chassis_Control(
    //     DJI_DR16_Data.RC_Value.CH0 / 2,
    //     DJI_DR16_Data.RC_Value.CH1 / -2);

    // switch (DJI_DR16_Data.RC_Value.S1)
    switch (Chassis_Control_Data.Mode)
    {
    case 2:
        Chassis_Set_Mode(CHASSIS_FOLLOW);
        break;

    case 3:
        Chassis_Set_Mode(CHASSIS_PEG_TOP);
        break;

    default:
        break;
    }

    // Shoot_Set_Fire_Add_Speed(DJI_DR16_Data.RC_Value.Wheel * 36 / 800);
    Shoot_Set_Fire_Add_Speed(DJI_DR16_Data.RC_Value.Wheel);

    switch (DJI_DR16_Data.RC_Value.S2)
    {
    case 1:
        Shoot_Set_Mode(SHOOT_BURST_FIRE);
        break;

    case 2:
        Shoot_Set_Mode(SHOOT_SINGE_FIRE);
        break;

    case 3:
        Shoot_Set_Mode(SHOOT_STOP_FIRE);
        break;

    default:
        break;
    }
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
