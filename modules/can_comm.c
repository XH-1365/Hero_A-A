
/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-19 20:52:53
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-17 11:17:24
 * @FilePath: \RM_Hero_UP_Board\modules\can_comm.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "can_comm.h"
#include <string.h>
#include "bsp_can.h"

CAN_Comm_RX_UP_Board_Union UP_Board_RX_Data = {0};

uint8_t CAN_Comm_RX_UP_Board_Buffer[CAN_COMM_UP_BOARD_DLC] = {0};

// CAN_Comm_Struct UP_Board = {0};

CAN_Comm_Struct UP_Board = {0,
                            0,
                            &hcan1,
                            CAN_COMM_UP_BOARD_TX_ID,
                            CAN_COMM_UP_BOARD_RX_ID,
                            UP_Board_RX_Data.Buffer,
                            sizeof(UP_Board_RX_Data)};

void CAN_Comm_RX_Filter_Set(void)
{
    Bsp_CAN_RX_Filter_Struct Bsp_CAN_RX_Filter_config;

    // 过滤下板的数据，ID：0x301

    // 过滤器匹配ID设置
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.STID = 0x301;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.EXID = 0x0000;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.IDE = 0; // 为标准帧
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.RTR = 0; // 为数据帧

    // 掩码 ，掩码位为1时需要进行比较，为0时则可以时任意值，
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.STID = 0x7FF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.EXID = 0xFFFF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.IDE = 1; // 必须为标准帧
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.RTR = 1; // 必须为数据帧

    Bsp_CAN_RX_Filter_config.FilterBank = CAN1_FILTER_ID_CAN_COMM; // 过滤器编号  CAN过滤器有很多个选择其中一个即可
    Bsp_CAN_RX_Filter_config.SlaveStartFilterBank = 14;            // 起始过滤器编号应该为14，这样的话 can1(0-13)和can2(14-27)就能分别得到一半的filter
    Bsp_CAN_RX_Filter_config.hcan = &hcan1;                        // 选择CAN1或者CAN2
    Bsp_CAN_RX_Filter_config.fifox = CAN_FilterFIFO0;
    Bsp_CAN_RX_Filter_config.FilterActivation = CAN_FILTER_ENABLE;
    Bsp_CAN_RX_Filter_Set(&Bsp_CAN_RX_Filter_config);
}
void CAN_Comm_Init(void)
{

    CAN_Comm_RX_Filter_Set();

}

char CAN_Comm_RX_Callback(CAN_Comm_Struct *Instance, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RX_Buffer)
{
    uint16_t RX_StdId = 0;
    RX_StdId = RxHeader->StdId;

    if (RX_StdId == Instance->RX_STD_ID)
    {

        memcpy(Instance->RX_Data, RX_Buffer, RxHeader->DLC);
        return SUCCESS;
    }

    return ERROR;
}
// 以后还是不要用Handle来命名回调函数了以后都用callback
//  char CAN_Comm_Get_Data_Handle(CAN_Comm_Struct *Instance, CAN_RxHeaderTypeDef *RxHeader, uint8_t *Motor_RX_Buffer)
//  {

// }

void CAN_Comm_TX_Send_Data(CAN_Comm_Struct *Instance, uint8_t *Data, uint8_t DLC)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t Error_Data = 0;
    TxHeader.StdId = Instance->TX_STD_ID;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = DLC;

    Error_Data = BSP_CAN_TRANSMIT(Instance->hcan, &TxHeader, Data, &TxMailbox);
    if (Error_Data != HAL_OK)
    {

        //        Error_Handler();
    }
}

void CAN_Comm_Get_RX_Data(CAN_Comm_Struct *Instance, void *Buffer, uint8_t size_t)
{
    memcpy(Buffer, Instance->RX_Data, size_t);
}

void CAN_Comm_Timing_Handle(void)
{
    if (UP_Board.Count > 0)
    {
        UP_Board.Count--;
    }
}

/// @brief 获取当前CAN设备是否正常连接
/// @param Instance
/// @return 返回1说明正常连接中，返回0说明连接断开
char CAN_Comm_Get_State(CAN_Comm_Struct *Instance)
{
    if (Instance->Count > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
