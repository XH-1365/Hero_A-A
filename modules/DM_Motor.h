/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-01 14:54:50
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-03 16:04:49
 * @FilePath: \RM_Template\modules\DM_Motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _DM_Motor_H__
#define _DM_Motor_H__
#include "RM_typedef.h"
#include "daemon.h"
// 设置此参数输出最大-25 最小0，所以我在解析后让25+解析后的值
#define P_MIN -3.1415f
#define P_MAX 3.1415f

#define V_MIN -25.0f
#define V_MAX 25.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef enum __DM_Motor_Mode_enum
{
    DM_MIT = 1,     // MIT模式
    DM_ANGLE_SPEED, // 位置速度模式
    DM_SPEED,       // 速度模式
    DM_EMIT,        // 力位混控模式
} DM_Motor_Mode_enum;

typedef enum __DM_Motor_Basic_Cmd_enum
{
    DM_CLEAR_ERROR = 0xFB, // 清除帧错误
    DM_ENABLE,             // 0xFC  电机使能
    DM_DISABLE,            // 0XFD 电机失能
    DM_SAVE_ANGLE_ZERO     // 0XFE  保存位置零点 就是将当前角度设为0点
} DM_Motor_Basic_Cmd_enum;

typedef enum __DM_Motor_Register_enum
{
    DM_REG_MOTOR_ANGLE = 80,     // 电机转子位置
    DM_REG_MOTOR_OUT_ANGLE = 81, // 电机输出轴位置
} DM_Motor_Register_enum;

typedef enum __DM_Motor_Para_Cmd_Tag_enum
{
    DM_PARA_READ = 0x33,
    DM_PARA_WRITE = 0x55,
    DM_PARA_SAVE = 0xAA

} DM_Motor_Para_Cmd_Tag_enum;
#pragma pack(1) // 指定结构体按照1字节对齐
typedef union
{
    // struct
    // {
    //     uint8_t ERR : 4;
    //     uint8_t ID : 4;
    //     int16_t Angle;
    //     int16_t Speed : 12;
    //     int16_t Tprque : 12;
    //     uint8_t Mos_Temp;   // 电调mos的温度
    //     uint8_t Rotor_Temp; // 电机绕组的温度
    // } Data;
    //   uint8_t Buffer[8];
    struct
    {
        uint8_t ERR;
        uint8_t ID;
        int16_t Angle;
        int16_t Speed;
        int16_t Tprque;
        uint8_t Mos_Temp;   // 电调mos的温度
        uint8_t Rotor_Temp; // 电机绕组的温度
    } Data;

    uint8_t Buffer[10];
} DM_Motor_Ret_Data_Union; // 电机的返回数据

typedef union
{
    struct
    {
        uint16_t CAN_ID;
        DM_Motor_Para_Cmd_Tag_enum Cmd_Tag; // 命令标识 读参数是0X33
        DM_Motor_Register_enum Reg_ID;
        uint8_t Data[4];
    } Para;
    uint8_t Buffer[8];
} DM_Motor_Ret_Para_Union; // 电机的返回参数
typedef struct __DM_Motor_Angle_Sum_Struct
{
    float Angle_Sum_Value;
    float Angle_Sum_Data;
    float Angle_Sum_Offset;
    float Angle_Turn;
    float Angle_Now;
    float Angle_Last;
} DM_Motor_Angle_Sum_Struct;
typedef struct __DM_Motor_Ret_Value_Struct
{
    float Angle;
    float Speed;
    float Tprque;
    DM_Motor_Angle_Sum_Struct Angle_Sum_Process;
} DM_Motor_Ret_Value_Struct;

typedef union
{
    struct
    {
        float Angle;     // 位置给定值
        uint16_t Speed;  // 速度限定值
        uint16_t Tprque; // 力矩限定值
    } Data;
    uint8_t Buffer[8];
} DM_Motor_EMIT_Union;

typedef struct __DM_Motor_Struct
{
    CAN_HandleTypeDef *hcan; // 使用的CAN口，hcan1或者hcan2
    int16_t TX_STD_ID;
    int16_t RX_STD_ID;
    DM_Motor_Ret_Data_Union Ret_Data;
    DM_Motor_Ret_Value_Struct Ret_Value;
    DM_Motor_Mode_enum Mode;
    DM_Motor_Ret_Para_Union Ret_Para; // 参数指令反馈报文
    Daemon_Struct *Daemon;
} DM_Motor_Struct;

#pragma pack() // 取消结构体对齐
extern DM_Motor_Struct DM_Motor_4310;

void DM_Motor_Init(void);

void DM_Motor_Clean_Angle_Sum(DM_Motor_Angle_Sum_Struct *Angle_Sum_Process);

char DM_Motor_Get_Data(DM_Motor_Struct *DM_Motor_x, CAN_RxHeaderTypeDef *RxHeader, uint8_t *Motor_RX_Buffer);

void DM_Motor_Read_Data_Cmd(DM_Motor_Struct *DM_Motor_x);

void DM_Motor_Save_Angle_Zero(DM_Motor_Struct *DM_Motor_x);

void DM_Motor_Set_State(DM_Motor_Struct *DM_Motor_x, FunctionalState State);

void DM_Motor_Clear_Error(DM_Motor_Struct *DM_Motor_x);

void DM_Motor_Read_Para_Cmd(DM_Motor_Struct *DM_Motor_x, DM_Motor_Register_enum Reg_ID);

void DM_Motor_Write_Para_Cmd(DM_Motor_Struct *DM_Motor_x, DM_Motor_Register_enum Reg_ID, uint8_t *Array);

void DM_Motor_Save_Para_Cmd(DM_Motor_Struct *DM_Motor_x);

void DM_Motor_EMIT_Control(DM_Motor_Struct *DM_Motor_x, float Angle, float Speed, float Tprque);

#endif
