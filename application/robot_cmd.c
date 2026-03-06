/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-09-04 14:32:58
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-03-22 14:39:51
 * @FilePath: \RM_Template\application\robot_cmd.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_cmd.h"
#include "DJI_DR16.h"
#include "DJI_VT13.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "can_comm.h"
#include "filter.h"
#include "DTOF.h"
#include "mini_pc.h"

#define control 1  //0 DR16  1 VT13

typedef struct __Robot_Cmd_Struct
{
    uint8_t Flag;
    uint16_t Count;

    float Yaw_Angle_Value;
    float Pitch_Angle_Value;
} Robot_Cmd_Struct;

Robot_Cmd_Struct Robot_Cmd;

Chassis_Control_Struct Chassis_Control_Data;

Low_Pass_Filter_Struct Chassis_Xv_Filter;
Low_Pass_Filter_Struct Chassis_Yv_Filter;

void Robot_Cmd_Init(void)
{
    Robot_Cmd.Flag = 0;
    Robot_Cmd.Count = 0;
    Low_Pass_Filter_Init(&Chassis_Xv_Filter, 0.05f, 0.0f);
    Low_Pass_Filter_Init(&Chassis_Yv_Filter, 0.05f, 0.0f);
}

void Robot_Cmd_Timing_Handle(void)
{
    if (Robot_Cmd.Count > 0)
    {
        Robot_Cmd.Count--;
    }
}

#if !control

float Gimbal_Yaw_Source_RC_CH2(void)
{
    return DJI_DR16_Data.RC_Value.CH2 / 410.0f;
}

float Gimbal_Pitch_Source_RC_CH3(void)
{
    return DJI_DR16_Data.RC_Value.CH3 / 280.0f;
}
float Gimbal_Yaw_Source_Mouse_X(void)
{
    return DJI_DR16_Data.Mouse.X / 25964.0f;
}

float Gimbal_Pitch_Source_Mouse_Y(void)
{
    return -DJI_DR16_Data.Mouse.Y / 15000.0f;
}


void Key_Shoot_Set_Fire(void)
{
    static uint8_t Key_Shoot_Fire_Flag = 0;

    switch (Key_Shoot_Fire_Flag)
    {

    case 0:
        Key_Shoot_Fire_Flag = 1;
        break;

    case 1:
        if (DJI_DR16_Data.Keyboard.Key.F == 1)
        {
            Shoot_Set_Mode(SHOOT_SINGE_FIRE);
            Key_Shoot_Fire_Flag = 2;
        }
        break;

    case 2:
        if (DJI_DR16_Data.Keyboard.Key.F == 0)
        {
            Key_Shoot_Fire_Flag = 3;
        }
        break;

    case 3:
        if (DJI_DR16_Data.Keyboard.Key.F == 1)
        {
            Shoot_Set_Mode(SHOOT_STOP_FIRE);
            Key_Shoot_Fire_Flag = 4;
        }
        break;
    case 4:
        if (DJI_DR16_Data.Keyboard.Key.F == 0)
        {
            Key_Shoot_Fire_Flag = 1;
        }
        break;
    }
}
void Mouse_Key_Set(void)
{
    static float Xv = 0.0f;
    static float Yv = 0.0f;
    Yv = ((float)DJI_DR16_Data.Keyboard.Key.W) * 1000.0f - ((float)DJI_DR16_Data.Keyboard.Key.S) * 1000.0f;
    Xv = ((float)DJI_DR16_Data.Keyboard.Key.D) * 1000.0f - ((float)DJI_DR16_Data.Keyboard.Key.A) * 1000.0f;

    Key_Shoot_Set_Fire();

    // 复位达妙电机
    if (DJI_DR16_Data.Keyboard.Key.Q)
    {
        Shoot_Enable_DM();
    }


    //设置底盘功率模式
    if(DJI_DR16_Data.Keyboard.Key.Ctrl)
    {
			Chassis_Control_Data.Power_Mode=1;//设置为超级模式
    }
    else
    {
        Chassis_Control_Data.Power_Mode=0;//设置为普通模式
    }

    Shoot_Set_Fire_Add_Speed(DJI_DR16_Data.Mouse.Key_L - DJI_DR16_Data.Keyboard.Key.Z * 2);

    Chassis_Control_Data.X_Speed = (int16_t)(Xv);
    Chassis_Control_Data.Y_Speed = (int16_t)(Yv);

    Chassis_Control_Data.Mode = DJI_DR16_Data.Keyboard.Key.Shift == 0 ? 3 : 2;
    Chassis_Control_Data.DTOF_Distance = DTOF.Distance;
    Chassis_Control_Data.Fire_Flag = Shoot_Get_Mode(); // 为0则是停止，为2则是单发
    CAN_Comm_TX_Send_Data(&UP_Board, (uint8_t *)&Chassis_Control_Data, 8);
}

void Remote_Control_Set(void)
{

    // Gimbal_Add_Angle(
    //     DJI_DR16_Data.RC_Value.CH2 / 70.0f,
    //     // DJI_DR16_Data.RC_Value.CH3 * 2.0f
    //      DJI_DR16_Data.RC_Value.CH3 / 280.0f
    //     );
    if ((DJI_DR16_Data.RC_Value.S1 == 1) && (DJI_DR16_Data.RC_Value.S2 != 1))
    {
        Chassis_Control_Data.Power_Mode=1;
    }   
		else
		{
			Chassis_Control_Data.Power_Mode=0;
		}
		
		Chassis_Control_Data.X_Speed = (int16_t)(DJI_DR16_Data.RC_Value.CH0 * 1000);
    Chassis_Control_Data.Y_Speed = (int16_t)(DJI_DR16_Data.RC_Value.CH1 * 1000);
    Chassis_Control_Data.Mode = ((DJI_DR16_Data.RC_Value.S1)!=2)?3:2;
    Chassis_Control_Data.DTOF_Distance = DTOF.Distance;
    Chassis_Control_Data.Fire_Flag = Shoot_Get_Mode(); // 为0则是停止，为2则是单发
    CAN_Comm_TX_Send_Data(&UP_Board, (uint8_t *)&Chassis_Control_Data, 8);

    Shoot_Set_Fire_Add_Speed(DJI_DR16_Data.RC_Value.Wheel);

    switch (DJI_DR16_Data.RC_Value.S2)
    {
    case 1:
        // Shoot_Set_Mode(SHOOT_SINGE_FIRE);
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
// 返回1进去键鼠模式，返回0进入遥控器模式
uint8_t Robot_Cmd_Control_Mode(void)
{
    if ((DJI_DR16_Data.RC_Value.S2 == 1) && (DJI_DR16_Data.RC_Value.S1 == 1))
    {
        return 1;
    }
    return 0;
}
#endif


#if control

float Gimbal_PITCH_Source_RC(void)
{
    return DJI_VT13_Data.CH2 / 450.0f;
}

float Gimbal_YAW_Source_RC(void)
{
    return DJI_VT13_Data.CH3  / 550.0f;
}
float Gimbal_Yaw_Source_Mouse_X(void)
{
    return DJI_VT13_Data.Remote.mouse_x / 24964.0f;
}

float Gimbal_Pitch_Source_Mouse_Y(void)
{
    return DJI_VT13_Data.Remote.mouse_y / 15000.0f;
}

void Key_Shoot_Set_Fire(void)
{
    static uint8_t Flag = 0;

    switch (Flag)
    {
    case 0:
        Flag = 1;
        break;

    case 1:
        if (DJI_VT13_Data.Remote.key_board.key.F == 1)
        {
            Shoot_Set_Mode(SHOOT_SINGE_FIRE);
            Flag = 2;
        }
        break;

    case 2:
        if (DJI_VT13_Data.Remote.key_board.key.F == 0)
        {
            Flag = 3;
        }
        break;

    case 3:
        if (DJI_VT13_Data.Remote.key_board.key.F == 1)
        {
            Shoot_Set_Mode(SHOOT_STOP_FIRE);
            Flag = 4;
        }
        break;
    case 4:
        if (DJI_VT13_Data.Remote.key_board.key.F == 0)
        {
            Flag = 1;
        }
        break;
    }
}
void Mouse_Key_Set(void)
{
    static float Xv = 0.0f;
    static float Yv = 0.0f;
    Yv = ((float)DJI_VT13_Data.Remote.key_board.key.W) * 1000.0f - ((float)DJI_VT13_Data.Remote.key_board.key.S) * 1000.0f;
    Xv = ((float)DJI_VT13_Data.Remote.key_board.key.D) * 1000.0f - ((float)DJI_VT13_Data.Remote.key_board.key.A) * 1000.0f;
    Key_Shoot_Set_Fire();

    // 复位达妙电机
    if (DJI_VT13_Data.Remote.key_board.key.Q)
    {
        Shoot_Enable_DM();
    }
    //设置底盘功率模式
    if(DJI_VT13_Data.Remote.key_board.key.Ctrl)
    {
        Chassis_Control_Data.Power_Mode=1;//设置为超级模式
    }
    else
    {
        Chassis_Control_Data.Power_Mode=0;//设置为普通模式
    }

    Shoot_Set_Fire_Add_Speed(DJI_VT13_Data.Remote.mouse_left - DJI_VT13_Data.Remote.key_board.key.Z * 2);

    
    if(DJI_VT13_Data.Remote.mouse_right )
    {
        Gimbal.Yaw.Angle_mini_pc  =PC_Get_Yaw();
        Gimbal.Pitch.Angle_mini_pc=PC_Get_Pitch();
    }
    else
    {
        Gimbal.Yaw.Angle_mini_pc=0;
        Gimbal.Pitch.Angle_mini_pc=0;
    }
    Chassis_Control_Data.X_Speed = (int16_t)(Xv);
    Chassis_Control_Data.Y_Speed = (int16_t)(Yv);

    Chassis_Control_Data.Mode = DJI_VT13_Data.Remote.key_board.key.Shift == 0 ? 3 : 2;
    Chassis_Control_Data.DTOF_Distance = DTOF.Distance;
    Chassis_Control_Data.Fire_Flag = Shoot_Get_Mode(); // 为0则是停止，为2则是单发
    CAN_Comm_TX_Send_Data(&UP_Board, (uint8_t *)&Chassis_Control_Data, 8);
}


void Remote_Power_Mode(void)
{
    static uint8_t Flag = 0;

    switch (Flag)
    {
    case 0:
        Chassis_Control_Data.Power_Mode=0;
        if (DJI_VT13_Data.Remote.stop == 0)
        {
            Flag = 1;
        }
        break;
    case 1:
        if (DJI_VT13_Data.Remote.stop  == 1)
        {
            Chassis_Control_Data.Power_Mode=1;
            Flag = 2;
        }
        break;
    case 2:
        if (DJI_VT13_Data.Remote.stop == 0)
        {
            Flag = 3;
        }
        break;
    case 3:
        if (DJI_VT13_Data.Remote.stop == 1)
        {
            Chassis_Control_Data.Power_Mode=0;
            Flag = 0;
        }
        break;
    }
}

void Remote_Chassis_Mode(void)
{
    static uint8_t Flag = 0;

    switch (Flag)
    {
    case 0:
        Chassis_Control_Data.Mode=3;
        if (DJI_VT13_Data.Remote.fn_r == 0)
        {
            Flag = 1;
        }
        break;
    case 1:
        if (DJI_VT13_Data.Remote.fn_r  == 1)
        {
            Chassis_Control_Data.Mode=2;//小陀螺
            Flag = 2;
        }
        break;
    case 2:
        if (DJI_VT13_Data.Remote.fn_r == 0)
        {
            Flag = 3;
        }
        break;
    case 3:
        if (DJI_VT13_Data.Remote.fn_r == 1)
        {
            Chassis_Control_Data.Mode=3;
            Flag = 0;
        }
        break;
    }
}
void Remote_FIRE_Mode(void)
{
    static uint8_t Flag = 0;

    switch (Flag)
    {
    case 0:
        Shoot_Set_Mode(SHOOT_STOP_FIRE);
        if (DJI_VT13_Data.Remote.fn_l == 0)
        {
            Flag = 1;
        }
        break;
    case 1:
        if (DJI_VT13_Data.Remote.fn_l  == 1)
        {
            Shoot_Set_Mode(SHOOT_SINGE_FIRE);
            Flag = 2;
        }
        break;
    case 2:
        if (DJI_VT13_Data.Remote.fn_l == 0)
        {
            Flag = 3;
        }
        break;
    case 3:
        if (DJI_VT13_Data.Remote.fn_l == 1)
        {
            Shoot_Set_Mode(SHOOT_STOP_FIRE);
            Flag = 0;
        }
        break;
    }
}
void Remote_Mini_pc_Mode(void)
{
    static uint8_t Flag = 0;

    switch (Flag)
    {
    case 0:
        Gimbal.Yaw.Angle_mini_pc=0;
        Gimbal.Pitch.Angle_mini_pc=0;
        if (DJI_VT13_Data.Remote.trigger == 0)
        {
            Flag = 1;
        }
        break;
    case 1:
        Gimbal.Yaw.Angle_mini_pc=0;
        Gimbal.Pitch.Angle_mini_pc=0;
        if (DJI_VT13_Data.Remote.trigger  == 1)
        {
            Flag = 2;
        }
        break;
    case 2:
        if (DJI_VT13_Data.Remote.trigger == 0)
        {
            Flag = 3;
        }
        break;
    case 3:
                Gimbal.Yaw.Angle_mini_pc  =PC_Get_Yaw();
        Gimbal.Pitch.Angle_mini_pc=PC_Get_Pitch();

        if (DJI_VT13_Data.Remote.trigger == 1)
        {
            Flag = 0;
        }
        break;
    }
}
void Remote_Control_Set(void)
{
    Remote_Power_Mode();
    Chassis_Control_Data.X_Speed = (int16_t)(DJI_VT13_Data.CH0 * 1000);
    Chassis_Control_Data.Y_Speed = (int16_t)(DJI_VT13_Data.CH1 * 1000);
    
    Remote_Chassis_Mode();
    Remote_Mini_pc_Mode();
    
    Chassis_Control_Data.DTOF_Distance = DTOF.Distance;
    Chassis_Control_Data.Fire_Flag = Shoot_Get_Mode(); // 为0则是停止，为2则是单发
    CAN_Comm_TX_Send_Data(&UP_Board, (uint8_t *)&Chassis_Control_Data, 8);
    Shoot_Set_Fire_Add_Speed(DJI_VT13_Data.Wheel);

    Remote_FIRE_Mode();
}
// 返回1进去键鼠模式，返回0进入遥控器模式
uint8_t Robot_Cmd_Control_Mode(void)
{
    if ( DJI_VT13_Data.Remote.mode_sw==2)
    {
        return 1;
    }
    return 0;
}
#endif

void Robot_Cmd_Task(void)
{
    if (Robot_Cmd.Count > 0)
    {
        return;
    }

    switch (Robot_Cmd.Flag)
    {
    case 0:
        Robot_Cmd.Flag = 1;

        break;

    case 1:
        if (Robot_Cmd_Control_Mode() == 1)
        {
            Robot_Cmd.Flag = 4;
        }
        else
        {
            Robot_Cmd.Flag = 2;
        }
        break;

    case 2:
        Gimbal_Set_Add_Angel_Fn_Ptr(GIMBAL_MOTOR_YAW, &Gimbal_YAW_Source_RC);     // 设置云台Yaw轴目标值增量
        Gimbal_Set_Add_Angel_Fn_Ptr(GIMBAL_MOTOR_PITCH, &Gimbal_PITCH_Source_RC); // 设置云台Pitch轴目标值增量
        Robot_Cmd.Flag = 3;
        break;
    case 3:

        if (Robot_Cmd_Control_Mode() == 1)
        {
            Robot_Cmd.Flag = 4;
        }

        Remote_Control_Set();
        Robot_Cmd.Count = 5;
        break;

    case 4:
        Gimbal_Set_Add_Angel_Fn_Ptr(GIMBAL_MOTOR_YAW, &Gimbal_Yaw_Source_Mouse_X);     // 设置云台Yaw轴目标值增量
        Gimbal_Set_Add_Angel_Fn_Ptr(GIMBAL_MOTOR_PITCH, &Gimbal_Pitch_Source_Mouse_Y); // 设置云台Pitch轴目标值增量
        Robot_Cmd.Flag = 5;
        break;

    case 5:
        if (Robot_Cmd_Control_Mode() == 0)
        {
            Robot_Cmd.Flag = 2;
        }

        Mouse_Key_Set();
        Robot_Cmd.Count = 5;
        break;
    }
}
