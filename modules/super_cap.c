/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-11-05 21:22:03
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-14 19:26:35
 * @FilePath: \RM_Template\modules\super_cap.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "super_cap.h"
#include "bsp_can.h"
#include <string.h>
Super_Cap_Struct Super_Cap = {0};
void Super_Cap_RX_Filter_Set(void)
{
    Bsp_CAN_RX_Filter_Struct Bsp_CAN_RX_Filter_config;

    // 过滤超电管理，ID：0x211

    // 过滤器匹配ID设置
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.STID = SUPER_CAP_RX_STD_ID;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.EXID = 0x0000;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.IDE = 0; // 为标准帧
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.RTR = 0; // 为数据帧

    // 掩码 ，掩码位为1时需要进行比较，为0时则可以时任意值，
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.STID = 0x7FF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.EXID = 0xFFFF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.IDE = 1; // 必须为标准帧
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.RTR = 1; // 必须为数据帧

    Bsp_CAN_RX_Filter_config.FilterBank = CAN2_FILTER_ID_SUPER_CAP; // 过滤器编号  CAN过滤器有很多个选择其中一个即可
    Bsp_CAN_RX_Filter_config.SlaveStartFilterBank = 14;             // 起始过滤器编号 can1(0-13)和can2(14-27)分别得到一半的filter
    Bsp_CAN_RX_Filter_config.hcan = &hcan2;                         // 选择CAN1或者CAN2
    Bsp_CAN_RX_Filter_config.fifox = CAN_FilterFIFO1;
    Bsp_CAN_RX_Filter_config.FilterActivation = CAN_FILTER_ENABLE;
    Bsp_CAN_RX_Filter_Set(&Bsp_CAN_RX_Filter_config);
}

char Super_Cap_Get_Data(Super_Cap_Struct *Super_Cap_x, CAN_RxHeaderTypeDef *RxHeader, uint8_t *CAN_RX_Buffer)
{
    uint16_t RX_StdId = 0;
    RX_StdId = RxHeader->StdId;

    if (RX_StdId == Super_Cap_x->RX_STD_ID)
    {

        memcpy(Super_Cap_x->Ret_Data.Buffer, CAN_RX_Buffer, sizeof(Super_Cap_Ret_Data_Union));

        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        return SUCCESS;
    }

    return ERROR;
}

void Super_Cap_Timing_Handle()
{
    if(--Super_Cap.Time_out==0)
    {
        Super_Cap.Time_out=800;
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
    }
}

void Super_Cap_init(void)
{
    Super_Cap.hcan = &hcan2;
    Super_Cap.RX_STD_ID = SUPER_CAP_RX_STD_ID;
    Super_Cap.TX_STD_ID = SUPER_CAP_TX_STD_ID;
    Super_Cap_RX_Filter_Set();
    Super_Cap.Time_out=50;
}


//设置超电功率单位 0.01w

void Super_Cap_Set_Power(Super_Cap_Struct *Super_Cap_x, uint16_t Target_Power)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox=CAN_TX_MAILBOX0;
    uint8_t CAN_TX_Data[2];
    uint8_t Error_Data = 0;

    TxHeader.StdId = Super_Cap_x->TX_STD_ID;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x02;
    CAN_TX_Data[0] = (uint8_t)(Target_Power >> 8);
    CAN_TX_Data[1] = (uint8_t)(Target_Power);

    Error_Data = BSP_CAN_TRANSMIT(Super_Cap_x->hcan, &TxHeader, CAN_TX_Data, (uint32_t *)TxMailbox);
    if (Error_Data != HAL_OK)
    {
        Error_Handler();
    }
}
