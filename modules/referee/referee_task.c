/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2025-03-14 14:30:13
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-21 02:35:44
 * @FilePath: \RM_Hero_Down_Board\modules\referee\referee_task.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
// #include "robot_def.h"
#include "RM_typedef.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "super_cap.h"
#include "robot_cmd.h"
// #include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
Referee_Interactive_info_t ui_data;                  // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
UI_Task_Struct UI_Instance;
uint8_t referee_timeout;
// @todo 不应该使用全局变量

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID(void)
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_info.referee_id.Robot_Color = referee_info.GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;

    referee_info.referee_id.Robot_ID = referee_info.GameRobotState.robot_id;
    referee_info.referee_id.Cilent_ID = 0x0100 + referee_info.referee_id.Robot_ID; // 计算客户端ID
    referee_info.referee_id.Receiver_Robot_ID = 0;
}

void UITaskInit(void)
{
    referee_recv_info = Referee_Get_Instance();
    // referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = &ui_data; // 获取UI绘制需要的机器人状态数据
    referee_info.init_flag = 1;
}

void UI_Task_Timing(void)
{
    if (UI_Instance.Count > 0)
    {
        UI_Instance.Count--;
    }
    if(referee_timeout--==0)
    {
        referee_timeout=250;
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
    }
}

static uint32_t shoot_line_location[7] = {540, 960, 440, 467, 452,700,0};

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[1];      // 电容能量条
// static String_Data_t UI_State_sta[6];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[6]; // 机器人状态,动态先add才能change
static Graph_Data_t UI_Value_dyn[1]; // 机器人数据,动态先add才能change

//计算刻度线的Y坐标
float Calculate_Scale_y(float Distance) {
    return ((-0.00993f * Distance) + 475.68f);
}
// 刷新刻度线
static void Refresh_Scale(referee_id_t *referee_id, uint16_t DTOF_Distance)
{
    static uint8_t first_draw = 1;
    shoot_line_location[5]=(uint32_t)Calculate_Scale_y((float)DTOF_Distance);
    if (first_draw)
    {
        first_draw = 0;
        // 绘制发射基准线
        UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
        UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
        UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
        UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
        UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Green, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
        UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Cyan, 2, 810, shoot_line_location[5], 1110, shoot_line_location[5]);
    }
    else
    {
        UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_Change, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
        UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_Change, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
        UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_Change, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
        UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_Change, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
        UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_Change, 7, UI_Color_Green, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
        UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_Change, 7, UI_Color_Cyan, 2, 810, shoot_line_location[5], 1110, shoot_line_location[5]);
    }

    UIGraphRefresh(&referee_info.referee_id, 7, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4],UI_shoot_line[5],UI_shoot_line[6]);
}

// 刷新超电电压
static void DrawEnergyBar(referee_id_t *referee_id, uint32_t center_x, uint32_t start_y, uint32_t width, float voltage)
{
    static uint8_t first_draw = 1;
    // 电压范围 7-24V，映射到能量条长度 0-400 像素
    float voltage_normalized = voltage - 7.0f; // 将 7V 作为起点
    if (voltage_normalized < 0.0f)
        voltage_normalized = 0.0f;                                              // 小于 7V 时长度为 0
    uint32_t energy_length = (uint32_t)(voltage_normalized * (400.0f / 17.0f)); // 17V 范围 (24-7) 对应 400 像素
    if (energy_length > 400)
        energy_length = 400; // 上限 400 像素

    // 居中对齐：起点 x = 十字中心 x - 能量条最大长度/2
    uint32_t start_x = center_x - 200; // 400/2 = 200，中心点为 center_x

    // 根据电压值设置颜色
    uint32_t color;
    if (voltage > 20.0f)
    {
        color = UI_Color_Green; // 20V 以上绿色
    }
    else if (voltage >= 12.0f && voltage <= 20.0f)
    {
        color = UI_Color_Yellow; // 12-20V 黄色
    }
    else if (voltage >= 8.0f && voltage < 12.0f)
    {
        color = UI_Color_Pink; // 8-12V 主色（假设红色）
    }
    else
    {
        color = UI_Color_Pink; // 低于 8V 主色
    }

    if (first_draw)
    {
        UILineDraw(&UI_Energy[0], "sc1", UI_Graph_ADD, 9, color, width, start_x, start_y, start_x, start_y);
    }
    else
    {
        UILineDraw(&UI_Energy[0], "sc1", UI_Graph_Change, 9, color, width, start_x, start_y, start_x + energy_length, start_y);
    }
    if (first_draw) // 只执行一次
    {
        first_draw = 0;
    }
    UIGraphRefresh(referee_id, 1, UI_Energy[0]);
}

// 刷新发射机开关
void Refresh_Fire_Flag(referee_id_t *referee_id, uint8_t Fire_Flag)
{
    static uint8_t first_draw = 1;

    if (first_draw)
    {
        first_draw = 0;
        UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 1350, 450, "sh:close");
    }
    else
    {
        if (Fire_Flag == 0)
        {
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 1350, 450, "sh:close");
        }
        else if (Fire_Flag == 2)
        {
            UICharDraw(&UI_State_dyn[0], "sd1", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 1350, 450, "sh: open");
        }
    }
    UICharRefresh(referee_id, UI_State_dyn[0]);
}

void Refresh_Instance(referee_id_t *referee_id, uint16_t DTOF_Distance)
{
    static uint8_t first_draw = 1;
    // void UIIntDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
    // 			   uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer)
    if (first_draw)
    {
        first_draw = 0;
        UIIntDraw(&UI_Value_dyn[0], "sv1", UI_Graph_ADD, 8, UI_Color_Cyan, 18, 2, 1350, 400,(int32_t) DTOF_Distance);
    }
    else
    {

        UIIntDraw(&UI_Value_dyn[0], "sv1", UI_Graph_Change, 8, UI_Color_Cyan, 18, 2, 1350, 400,(int32_t) DTOF_Distance);
    }
    UIGraphRefresh(referee_id,1, UI_Value_dyn[0]);
}
// UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);

void UI_Task(void)
{
    // Chassis_Control_Data.Fire_Flag
    if (UI_Instance.Count > 0)
    {
        return;
    }

    switch (UI_Instance.Flag)
    {
    case 0:
        UI_Instance.Flag = 1;
        break;

    case 1:
        Refresh_Scale(&referee_info.referee_id, Chassis_Control_Data.DTOF_Distance);
        UI_Instance.Count = 100;
        UI_Instance.Flag = 2;
        break;

    case 2:
        DrawEnergyBar(&referee_info.referee_id, 1000, 220, 15, (float)(Super_Cap.Ret_Data.Data.Cap_Voltage) / 100.0f);
        UI_Instance.Count = 100;
        UI_Instance.Flag = 3;
        break;

    case 3:
        Refresh_Fire_Flag(&referee_info.referee_id, Chassis_Control_Data.Fire_Flag);
        UI_Instance.Count = 100;
        UI_Instance.Flag = 4;
        break;

    case 4:
       Refresh_Instance(&referee_info.referee_id,  Chassis_Control_Data.DTOF_Distance);
        UI_Instance.Count = 100;
        UI_Instance.Flag = 1;
        break;


    default:
        break;
    }
}

void MyUIInit(void)
{
    // if (!referee_info.init_flag)
    //     vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务

    //     while (1)
    //     {
    //         if ((referee_info.GameRobotState.robot_id)!= 0x00)
    //				 {
    //					  break;
    //				 }
    //
    //     };
    while (((referee_info.GameRobotState.robot_id) == 0x00))
    {
        HAL_Delay(300);
    }

    DeterminRobotID(); // 确定ui要发送到的目标客户端
    HAL_Delay(300);

    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 0); // 清空UI

    HAL_Delay(500);

    // // 绘制发射基准线
    // UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    // UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    // UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    // UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
    // UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
    // UIGraphRefresh(&referee_info.referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    // HAL_Delay(300);

    //    // 绘制车辆状态标志指示
    //    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 750, "chassis:");
    //    UICharRefresh(&referee_info.referee_id, UI_State_sta[0]);
    //    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 150, 700, "gimbal:");
    //    UICharRefresh(&referee_info.referee_id, UI_State_sta[1]);
    //    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 650, "shoot:");
    //    UICharRefresh(&referee_info.referee_id, UI_State_sta[2]);
    //    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 600, "frict:");
    //    UICharRefresh(&referee_info.referee_id, UI_State_sta[3]);
    //    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 550, "lid:");
    //    UICharRefresh(&referee_info.referee_id, UI_State_sta[4]);

    //    // 绘制车辆状态标志，动态
    //    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    //    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
    //    UICharRefresh(&referee_info.referee_id, UI_State_dyn[0]);
    //    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
    //    UICharRefresh(&referee_info.referee_id, UI_State_dyn[1]);
    //    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
    //    UICharRefresh(&referee_info.referee_id, UI_State_dyn[2]);
    //    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "off");
    //    UICharRefresh(&referee_info.referee_id, UI_State_dyn[3]);
    //    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "open ");
    //    UICharRefresh(&referee_info.referee_id, UI_State_dyn[4]);

    //    // 底盘功率显示，静态
    //    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    //    UICharRefresh(&referee_info.referee_id, UI_State_sta[5]);
    //    // 能量条框
    //    UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    //    UIGraphRefresh(&referee_info.referee_id, 1, UI_Energy[0]);

    //    // 底盘功率显示,动态
    //    UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    //    // 能量条初始状态
    //    UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    //    UIGraphRefresh(&referee_info.referee_id, 2, UI_Energy[1], UI_Energy[2]);
}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    //     count++;
    //     if (count >= 50)
    //     {
    //         count = 0;
    //         count1++;
    //     }
    //     switch (count1 % 4)
    //     {
    //     case 0:
    //     {
    //         _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
    //         _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
    //         _Interactive_data->shoot_mode = SHOOT_ON;
    //         _Interactive_data->friction_mode = FRICTION_ON;
    //         _Interactive_data->lid_mode = LID_OPEN;
    //         _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
    //         if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
    //             _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
    //         break;
    //     }
    //     case 1:
    //     {
    //         _Interactive_data->chassis_mode = CHASSIS_ROTATE;
    //         _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
    //         _Interactive_data->shoot_mode = SHOOT_OFF;
    //         _Interactive_data->friction_mode = FRICTION_OFF;
    //         _Interactive_data->lid_mode = LID_CLOSE;
    //         break;
    //     }
    //     case 2:
    //     {
    //         _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
    //         _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
    //         _Interactive_data->shoot_mode = SHOOT_ON;
    //         _Interactive_data->friction_mode = FRICTION_ON;
    //         _Interactive_data->lid_mode = LID_OPEN;
    //         break;
    //     }
    //     case 3:
    //     {
    //         _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //         _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
    //         _Interactive_data->shoot_mode = SHOOT_OFF;
    //         _Interactive_data->friction_mode = FRICTION_OFF;
    //         _Interactive_data->lid_mode = LID_CLOSE;
    //         break;
    //     }
    //     default:
    //         break;
    //     }
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    static uint8_t first_draw = 1;
    // UIChangeCheck(_Interactive_data);

    // chassis
    // if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    // {
    //     switch (_Interactive_data->chassis_mode)
    //     {
    //     case CHASSIS_ZERO_FORCE:
    //         UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
    //         break;
    //     case CHASSIS_ROTATE:
    //         UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "rotate   ");
    //         // 此处注意字数对齐问题，字数相同才能覆盖掉
    //         break;
    //     case CHASSIS_NO_FOLLOW:
    //         UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "nofollow ");
    //         break;
    //     case CHASSIS_FOLLOW_GIMBAL_YAW:
    //         UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "follow   ");
    //         break;
    //     }
    //     UICharRefresh(&referee_info.referee_id, UI_State_dyn[0]);
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    // }
    // // gimbal
    // if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    // {
    //     switch (_Interactive_data->gimbal_mode)
    //     {
    //     case GIMBAL_ZERO_FORCE:
    //     {
    //         UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "zeroforce");
    //         break;
    //     }
    //     case GIMBAL_FREE_MODE:
    //     {
    //         UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "free     ");
    //         break;
    //     }
    //     case GIMBAL_GYRO_MODE:
    //     {
    //         UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "gyro     ");
    //         break;
    //     }
    //     }
    //     UICharRefresh(&referee_info.referee_id, UI_State_dyn[1]);
    //     _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    // }
    // // shoot
    // if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
    //     UICharRefresh(&referee_info.referee_id, UI_State_dyn[2]);
    //     _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    // }
    // // friction
    // if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
    //     UICharRefresh(&referee_info.referee_id, UI_State_dyn[3]);
    //     _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    // }
    // // lid
    // if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
    //     UICharRefresh(&referee_info.referee_id, UI_State_dyn[4]);
    //     _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    // }
    // // power
    // if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    // {
    //     UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
    //     UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)750 + _Interactive_data->Chassis_Power_Data.chassis_power_mx * 30, 160);
    //     UIGraphRefresh(&referee_info.referee_id, 2, UI_Energy[1], UI_Energy[2]);
    //     _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    // }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
    //     _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    // }

    // if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
    //     _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    // }

    // if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
    //     _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    // }

    // if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
    //     _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    // }

    // if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
    //     _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    // }

    // if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    // {
    //     _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
    //     _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    // }
}
