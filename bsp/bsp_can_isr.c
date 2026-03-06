/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-13 13:34:24
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-02 21:15:47
 * @FilePath: \MDK-ARMf:\RM_Program\RM_Code_Program\External_Module\DJI_Motor\bsp\bsp_can_isr.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp_can.h"
#include "DJI_Motor.h"
#include "CH104_IMU_CAN.h"
#include "DM_Motor.h"
#include "can_comm.h"
#include "super_cap.h"
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t Error_Data = 0;
    CAN_RxHeaderTypeDef RxHeader;
    if (hcan == &hcan1)
    {
        Error_Data = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Bsp_CAN1_Fifo0_RX_Data);
        if (Error_Data != HAL_OK)
        {
            Error_Handler();
        }

        DJI_Motor_Get_Data(DJI_MGRP1, &RxHeader, Bsp_CAN1_Fifo0_RX_Data);
        DJI_Motor_Get_Data(DJI_MGRP2, &RxHeader, Bsp_CAN1_Fifo0_RX_Data);
        DM_Motor_Get_Data(&DM_Motor_4310, &RxHeader, Bsp_CAN1_Fifo0_RX_Data);
        CAN_Comm_RX_Callback(&UP_Board, &RxHeader, Bsp_CAN1_Fifo0_RX_Data);
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t Error_Data = 0;
    CAN_RxHeaderTypeDef RxHeader;
    if (hcan == &hcan2)
    {
        Error_Data = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, Bsp_CAN2_Fifo1_RX_Data);
        if (Error_Data != HAL_OK)
        {
            Error_Handler();
        }
        CH104_IMU_CAN_Get_Data(&CH104_IMU_CAN, &RxHeader, Bsp_CAN2_Fifo1_RX_Data);
        DJI_Motor_Get_Data(DJI_MGRP3, &RxHeader, Bsp_CAN2_Fifo1_RX_Data);
        Super_Cap_Get_Data(&Super_Cap, &RxHeader,Bsp_CAN2_Fifo1_RX_Data);
    }
}
