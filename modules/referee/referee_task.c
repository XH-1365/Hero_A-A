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
#include "chassis.h"
// #include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
Referee_Interactive_info_t ui_data;                  // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
UI_Task_Struct UI_Instance;
uint8_t referee_timeout;
// @todo 不应该使用全局变量

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

static uint32_t shoot_line_location[7] = {420, 960, 440, 467, 452,700,0};

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[2];      // 电容能量条
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
        UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Orange, 2, 860, shoot_line_location[0], 1060, shoot_line_location[0]);
        UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_Orange, 2, shoot_line_location[1], 320, shoot_line_location[1], 520);
//        UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
//        UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
//        UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Green, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
//       UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Cyan, 2, 810, shoot_line_location[5], 1110, shoot_line_location[5]);
          UIGraphRefresh(&referee_info.referee_id,2, UI_shoot_line[0], UI_shoot_line[1]);
    }
    else
    {
//        UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_Change, 7, UI_Color_White, 2, 910, shoot_line_location[0], 1110, shoot_line_location[0]);
//        UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_Change, 7, UI_Color_White, 2, shoot_line_location[1], 340, shoot_line_location[1], 540);
//        UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_Change, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
//        UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_Change, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
//        UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_Change, 7, UI_Color_Green, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
//        UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_Change, 7, UI_Color_Cyan, 2, 810, shoot_line_location[5], 1110, shoot_line_location[5]);
    }

  
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
}
// 刷新缓冲功率
static void DrawBufferEnergyr(referee_id_t *referee_id, uint32_t center_x, uint32_t start_y, uint32_t width, uint16_t value)
{
    static uint8_t first_draw = 1;
    // 范围 0-60J，映射到能量条长度 0-400 像素                                         
    uint32_t energy_length = (uint32_t)(value * (400.0f / 100.0f)); // 范围 (60-0) 对应 400 像素
    if (energy_length > 400)
        energy_length = 400; // 上限 400 像素

    // 居中对齐：起点 x = 十字中心 x - 能量条最大长度/2
    uint32_t start_x = center_x - 200; // 400/2 = 200，中心点为 center_x

    // 根据电压值设置颜色
    uint32_t color;

        color = UI_Color_Green; // 20V 以上绿色


    if (first_draw)
    {
        UILineDraw(&UI_Energy[1], "sc2", UI_Graph_ADD, 9, color, width, start_x, start_y, start_x, start_y);
    }
    else
    {
        UILineDraw(&UI_Energy[1], "sc2", UI_Graph_Change, 9, color, width, start_x, start_y, start_x + energy_length, start_y);
    }
    if (first_draw) // 只执行一次
    {
        first_draw = 0;
    }
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
// 刷新小陀螺开关
void Refresh_Chassis_Flag(referee_id_t *referee_id, uint8_t Chassis_Flag)
{
    static uint8_t first_draw = 1;

    if (first_draw)
    {
        first_draw = 0;
        UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 1350, 350, "ch:close");
    }
    else
    {
        if (Chassis_Flag == 3)
        {
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 1350, 350, "ch: open");
        }
        else
        {
            UICharDraw(&UI_State_dyn[1], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 1350, 350, "ch:close");
        }
        
    }
    UICharRefresh(referee_id, UI_State_dyn[1]);
}
//void Refresh_Instance(referee_id_t *referee_id, uint16_t DTOF_Distance)
//{
//    static uint8_t first_draw = 1;
//    // void UIIntDraw(Graph_Data_t *graph, char graphname[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color,
//    // 			   uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, int32_t Graph_Integer)
//    if (first_draw)
//    {
//        first_draw = 0;
//        UIIntDraw(&UI_Value_dyn[0], "sv1", UI_Graph_ADD, 8, UI_Color_Cyan, 18, 2, 1350, 400,(int32_t) DTOF_Distance);
//    }
//    else
//    {

//        UIIntDraw(&UI_Value_dyn[0], "sv1", UI_Graph_Change, 8, UI_Color_Cyan, 18, 2, 1350, 400,(int32_t) DTOF_Distance);
//    }
//    UIGraphRefresh(referee_id,1, UI_Value_dyn[0]);
//}


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
        DrawEnergyBar     (&referee_info.referee_id, 1000, 220, 15, (float)(Super_Cap.Ret_Data.Data.Cap_Voltage) / 100.0f);
//        DrawBufferEnergyr (&referee_info.referee_id, 1000, 200, 15, referee_info.PowerHeatData.buffer_energy);
        DrawBufferEnergyr (&referee_info.referee_id, 1000, 200, 15, referee_info.GameRobotState.chassis_power_limit);
        UIGraphRefresh(&referee_info.referee_id, 2, UI_Energy[0], UI_Energy[1]);
        UI_Instance.Count = 100;
        UI_Instance.Flag = 3;
        break;

    case 3:
        Refresh_Fire_Flag(&referee_info.referee_id, Chassis_Control_Data.Fire_Flag);
        UI_Instance.Count = 100;
        UI_Instance.Flag = 4;
        break;

    case 4:
       Refresh_Chassis_Flag(&referee_info.referee_id,Chassis.Mode);
        UI_Instance.Count = 100;
        UI_Instance.Flag = 1;
        break;
    case 5:
//      Refresh_Instance(&referee_info.referee_id,  Chassis_Control_Data.DTOF_Distance);
        UI_Instance.Count = 100;
        UI_Instance.Flag = 1;
        break;

    default:
        break;
    }
}

void MyUIInit(void)
{
    while (((referee_info.GameRobotState.robot_id) == 0x00))
    {
        HAL_Delay(300);
    }
    DeterminRobotID(); // 确定ui要发送到的目标客户端
    HAL_Delay(300);

    UIDelete(&referee_info.referee_id, UI_Data_Del_ALL, 0); // 清空UI

    HAL_Delay(500);
}
