#include "DJI_Motor.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "bsp_can.h"

DJI_Motor_Data_Struct DJI_MGRP1_RX_Data[DJI_MGRP1_RX_SUM];
DJI_Motor_Data_Struct DJI_MGRP2_RX_Data[DJI_MGRP2_RX_SUM];
DJI_Motor_Group_Struct Motor_Group[DJI_MGRP_SUM] = {
    // 从左到右成员顺序为:Flag--Count--hcan--TX_STD_ID--RX_STD_ID--Data
    {0, 0, &hcan2, {0x200, 0x1FF}, 0x200, DJI_MGRP1_RX_Data, DJI_MGRP1_RX_SUM},
    {0, 0, &hcan1, {0x1FF, 0x2FF}, 0x204, DJI_MGRP2_RX_Data, DJI_MGRP2_RX_SUM},

};
pid_controler Motor_Pid1;
float Speed_Tar = 0.001;
float Speed_err = 0;
float Speed_out = 0;

char Motor_str[100];

uint8_t Error_Data = 0;

void DJI_Motor_Data_Init(void)
{
}

/// @brief 必须在DJI_Motor_Data_Init();之后调用
/// @param
void DJI_Motor_RX_Filter_Set(void)
{

    Bsp_CAN_RX_Filter_Struct Bsp_CAN_RX_Filter_config;
	
    // 云台电机，ID：0x205~0x206
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.STID = 0x204;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.EXID = 0x0000;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.IDE = 0; // 为标准帧
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.RTR = 0; // 为数据帧

    // 掩码 ，掩码位为1时需要进行比较，为0时则可以时任意值，
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.STID = 0x7F0;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.EXID = 0xFFFF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.IDE = 1; // 必须为标准帧
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.RTR = 1; // 必须为数据帧

    Bsp_CAN_RX_Filter_config.FilterBank = CAN1_FILTER_ID_GIMBAL;           // 过滤器编号  CAN过滤器有很多个选择其中一个即可
    Bsp_CAN_RX_Filter_config.SlaveStartFilterBank = 14;// 起始过滤器编号应该为14，这样的话 can1(0-13)和can2(14-27)就能分别得到一半的filter
    Bsp_CAN_RX_Filter_config.hcan = &hcan1;            // 选择CAN1或者CAN2
    Bsp_CAN_RX_Filter_config.fifox = CAN_FilterFIFO0;
    Bsp_CAN_RX_Filter_config.FilterActivation = CAN_FILTER_ENABLE;
    Bsp_CAN_RX_Filter_Set(&Bsp_CAN_RX_Filter_config);
	
	
    // 过滤底盘电机ID：0x201~0x204

    // 过滤器匹配ID设置
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.STID = 0x200;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.EXID = 0x0000;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.IDE = 0; // 为标准帧
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.RTR = 0; // 为数据帧

    // 掩码 ，掩码位为1时需要进行比较，为0时则可以时任意值，
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.STID = 0x7F0;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.EXID = 0xFFFF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.IDE = 1; // 必须为标准帧
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.RTR = 1; // 必须为数据帧

    Bsp_CAN_RX_Filter_config.FilterBank = CAN2_FILTER_ID_CHASSIS;           // 过滤器编号  CAN过滤器有很多个选择其中一个即可
    Bsp_CAN_RX_Filter_config.SlaveStartFilterBank = 14;// 起始过滤器编号应该为14，这样的话 can1(0-13)和can2(14-27)就能分别得到一半的filter
    Bsp_CAN_RX_Filter_config.hcan = &hcan2;            // 选择CAN1或者CAN2
    Bsp_CAN_RX_Filter_config.fifox = CAN_FilterFIFO1;
    Bsp_CAN_RX_Filter_config.FilterActivation = CAN_FILTER_ENABLE;
    Bsp_CAN_RX_Filter_Set(&Bsp_CAN_RX_Filter_config);


		


}

/// @brief 需要Bsp_CAN_Init()前初始化
/// @param
void DJI_Motor_Init(void)
{
    DJI_Motor_Data_Init();
    DJI_Motor_RX_Filter_Set();
    PID_Init(&Motor_Pid1);
}

void DJI_Motor_Set_Current(DJI_Motor_Group_enum DJI_MGRPx, DJI_Motor_TX_ID_enum TX_ID, int motor1, int motor2, int motor3, int motor4)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t CAN_TX_Data[8];
    TxHeader.StdId = Motor_Group[DJI_MGRPx].TX_STD_ID[TX_ID];
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    CAN_TX_Data[0] = (uint8_t)(motor1 >> 8);
    CAN_TX_Data[1] = (uint8_t)(motor1);
    CAN_TX_Data[2] = (uint8_t)(motor2 >> 8);
    CAN_TX_Data[3] = (uint8_t)(motor2);
    CAN_TX_Data[4] = (uint8_t)(motor3 >> 8);
    CAN_TX_Data[5] = (uint8_t)(motor3);
    CAN_TX_Data[6] = (uint8_t)(motor4 >> 8);
    CAN_TX_Data[7] = (uint8_t)(motor4);
    Error_Data = BSP_CAN_TRANSMIT(Motor_Group[DJI_MGRPx].hcan, &TxHeader, CAN_TX_Data, &TxMailbox);
    if (Error_Data != HAL_OK)
    {
        /* Reception Error */
        // sprintf((char *)Motor_str, "Error_Typedef:%d\r\n", Error_Data);
        // while (1)
        // {
        //     HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Motor_str, strlen(Motor_str)); // 串口回传1号电机返回的数据
        // }

        // Error_Handler();
    }
}
void DJI_Motor_Set_Current_Value(DJI_Motor_Group_enum DJI_MGRPx, DJI_Motor_TX_ID_enum TX_ID, float Motor1, float Motor2, float Motor3, float Motor4)
{
    int motor1 = Motor1 * 16384;
    int motor2 = Motor2 * 16384;
    int motor3 = Motor3 * 16384;
    int motor4 = Motor4 * 16384;

    DJI_Motor_Set_Current(DJI_MGRPx, TX_ID, motor1, motor2, motor3, motor4);
}

float DJI_Motor_Get_Angle_Sum(DJI_Motor_Angle_Sum_Struct *Angle_Sum_Process, float Angle_Value, float Max, float Min)
{

    float Jump_Value = Max - Min;

    Angle_Sum_Process->Angle_Last = Angle_Sum_Process->Angle_Now;
    Angle_Sum_Process->Angle_Now = Angle_Value;
    if ((Angle_Sum_Process->Angle_Last - Angle_Sum_Process->Angle_Now) > (Jump_Value - 0.2f))
    {
        (Angle_Sum_Process->Angle_Turn) += Jump_Value;
    }
    else if ((Angle_Sum_Process->Angle_Last - Angle_Sum_Process->Angle_Now) < -(Jump_Value - 0.2f))
    {
        (Angle_Sum_Process->Angle_Turn) -= Jump_Value;
    }
    Angle_Sum_Process->Angle_Sum_Value = (Angle_Sum_Process->Angle_Turn) + (Angle_Sum_Process->Angle_Now);
    return Angle_Sum_Process->Angle_Sum_Value;
}

static void DJI_Motor_Set_Ret(uint8_t *Motor_RX_Buffer, DJI_Motor_Group_enum DJI_MGRPx, uint8_t Motor_Id)
{
    uint16_t angle = 0;
    int16_t speed = 0;
    int16_t current = 0;
    angle = ((uint16_t)Motor_RX_Buffer[0] << 8) | (uint16_t)(Motor_RX_Buffer[1]);
    speed = ((uint16_t)Motor_RX_Buffer[2] << 8) | (uint16_t)(Motor_RX_Buffer[3]);
    current = ((uint16_t)Motor_RX_Buffer[4] << 8) | (uint16_t)(Motor_RX_Buffer[5]);
    Motor_Group[DJI_MGRPx].Data[Motor_Id].Ret.Temperature = Motor_RX_Buffer[6];

    Motor_Group[DJI_MGRPx].Data[Motor_Id].Ret.Angle = (float)angle / 8191;      // 角度归一化
    Motor_Group[DJI_MGRPx].Data[Motor_Id].Ret.Speed = speed;                    // 速度不进行归一化
    Motor_Group[DJI_MGRPx].Data[Motor_Id].Ret.Current = (float)current / 16384; // 电流归一化

    DJI_Motor_Get_Angle_Sum(
        &(Motor_Group[DJI_MGRPx].Data[Motor_Id].Ret.Angle_Sum_Process),
        Motor_Group[DJI_MGRPx].Data[Motor_Id].Ret.Angle,
        1.0,
        0);
}

char DJI_Motor_Get_Data(DJI_Motor_Group_enum DJI_MGRPx, CAN_RxHeaderTypeDef *RxHeader, uint8_t *Motor_RX_Buffer)
{
    uint16_t RX_StdId = 0;
    uint16_t RX_num = 0;
    // uint8_t i = 0;
    uint8_t DJI_Motor_RX_SUM = 0;

    RX_StdId = RxHeader->StdId;

    // switch (DJI_MGRPx)
    // {
    // case DJI_MGRP1:
    //     DJI_Motor_RX_SUM = DJI_MGRP1_RX_SUM;
    //     break;
    // case DJI_MGRP2:
    //     DJI_Motor_RX_SUM = DJI_MGRP2_RX_SUM;
    //     break;
    // case DJI_MGRP3:
    //     DJI_Motor_RX_SUM = DJI_MGRP3_RX_SUM;
    //     break;

    // default:
    //     return ERROR;
    // }

    DJI_Motor_RX_SUM = Motor_Group[DJI_MGRPx].Data_Length;
    // 判断接收到的ID是否大于CAN接收数据帧ID偏移，大于才能通过否则return
    if (RX_StdId <= Motor_Group[DJI_MGRPx].RX_STD_ID)
    {
        return ERROR;
    }

    RX_num = RX_StdId - (Motor_Group[DJI_MGRPx].RX_STD_ID + 1);

    // 判断接收到的ID是否小于当前组的最大电机数，小于才能通过否则return
    if (RX_num + 1 > DJI_Motor_RX_SUM)
    {
        return ERROR;
    }

    Motor_Group[DJI_MGRPx].Data[RX_num].Ret.RX_Flag = 1;
    DJI_Motor_Set_Ret(Motor_RX_Buffer, DJI_MGRPx, RX_num);
    return SUCCESS;
}

void DJI_Motor_Timing_Handle(void)
{
    uint8_t i = 0;
    for (i = DJI_MGRP1; i < DJI_MGRP_SUM; i++)
    {
        if (Motor_Group[i].Count > 0)
        {
            Motor_Group[i].Count = Motor_Group[i].Count - 1;
        } /* code */
    }
}
void DJI_Motor_Handle(void)
{

    if (Motor_Group[DJI_MGRP1].Count != 0)
    {
        return;
    }

    switch (Motor_Group[DJI_MGRP1].Flag)
    {
    case 0:
        // PID_Set(&Motor_Pid1, 0, 0, 0, 0.01, 0.002, 0.8);
        // Motor_Pid1.kp = 0.01;
        // Motor_Pid1.ki = 0;
        // Motor_Pid1.kd = 0;
        // Motor_Group[DJI_MGRP1].Count = 2000;
        Motor_Group[DJI_MGRP1].Flag = 1;
        break;
    case 1:

        // Motor_Pid1.err = Speed_Tar - Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_2].Ret.Speed;
        // Motor_Pid1.output = pid_error_input(&Motor_Pid1, Motor_Pid1.err);
        // DJI_Motor_Set_Current_Value(DJI_MGRP1, DJI_Motor_TX_1_4, 0, Motor_Pid1.output, 0, 0);
        // //        sprintf((char *)Motor_str, "M1{Speed:%.2f,Angle:%.2f,Current:%.2f,Temperature:%u}\r\n", Motor_Group[DJI_MGRP1].Data[DJI_M3508_RX_2].Ret.Speed,
        // //                Motor_Group[DJI_MGRP1].Data[DJI_M3508_RX_2].Ret.Angle,
        // //                Motor_Group[DJI_MGRP1].Data[DJI_M3508_RX_2].Ret.Current,
        // //                Motor_Group[DJI_MGRP1].Data[DJI_M3508_RX_2].Ret.Temperature);

        // sprintf((char *)Motor_str, "Angle:%.2f\r\n", Motor_Group[DJI_MGRP1].Data[DJI_MGRP1_M3508_RX_2].Ret.Angle * 360);
        // HAL_UART_Transmit_DMA(&huart6, (uint8_t *)Motor_str, strlen(Motor_str)); // 串口回传1号电机返回的数据

        Motor_Group[DJI_MGRP1].Count = 10;
        break;
    }
}
