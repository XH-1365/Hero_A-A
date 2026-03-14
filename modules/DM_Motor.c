/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-01 14:54:37
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-15 20:54:21
 * @FilePath: \RM_Template\modules\DM_Motor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "DM_Motor.h"
#include "bsp_can.h"
#include <string.h>

// 发送ID，就是达妙手册所谓的CAN ID
#define DM_TX_STD_ID 0X11
// 接收ID，就是达妙手册所谓的Master ID
#define DM_RX_STD_ID 0X10

DM_Motor_Struct DM_Motor_4310;

void DM_Motor_RX_Filter_Set(void)
{
    Bsp_CAN_RX_Filter_Struct Bsp_CAN_RX_Filter_config;

    // 过滤DM拨弹电机，ID：0x10

    // 过滤器匹配ID设置
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.STID = DM_RX_STD_ID;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.EXID = 0x0000;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.IDE = 0; // 为标准帧
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.RTR = 0; // 为数据帧

    // 掩码 ，掩码位为1时需要进行比较，为0时则可以时任意值，
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.STID = 0x7FF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.EXID = 0xFFFF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.IDE = 1; // 必须为标准帧
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.RTR = 1; // 必须为数据帧

    Bsp_CAN_RX_Filter_config.FilterBank = CAN1_FILTER_ID_DM_MOTOR;           // 过滤器编号  CAN过滤器有很多个选择其中一个即可
    Bsp_CAN_RX_Filter_config.SlaveStartFilterBank = 14;// 起始过滤器编号应该为14，这样的话 can1(0-13)和can2(14-27)就能分别得到一半的filter
    Bsp_CAN_RX_Filter_config.hcan = &hcan1;            // 选择CAN1或者CAN2
    Bsp_CAN_RX_Filter_config.fifox = CAN_FilterFIFO0;
    Bsp_CAN_RX_Filter_config.FilterActivation = CAN_FILTER_ENABLE;
    Bsp_CAN_RX_Filter_Set(&Bsp_CAN_RX_Filter_config);
}

void DM_Motor_Set_Mode(DM_Motor_Mode_enum Mode)
{
}

void DM_Motor_Init(void)
{
    DM_Motor_4310.hcan = &hcan1;
    DM_Motor_4310.RX_STD_ID = DM_RX_STD_ID;
    DM_Motor_4310.TX_STD_ID = DM_TX_STD_ID;
    DM_Motor_RX_Filter_Set();
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (((float)x_int) * span / ((float)((1 << bits) - 1))) + offset;
}
float DM_Motor_Get_Angle_Sum(DM_Motor_Angle_Sum_Struct *Angle_Sum_Process, float Angle_Value, float Max, float Min)
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
static void DM_Motor_Set_Ret(DM_Motor_Struct *DM_Motor_x, uint8_t *Motor_RX_Buffer)
{
    // memcpy(DM_Motor_x->Ret_Data.Buffer, Motor_RX_Buffer, sizeof(DM_Motor_Ret_Data_Union));
    DM_Motor_x->Ret_Data.Data.ERR = (Motor_RX_Buffer[0]) >> 4;
    DM_Motor_x->Ret_Data.Data.ID = (Motor_RX_Buffer[0]) & 0x0F;

    DM_Motor_x->Ret_Data.Data.Angle = (((int16_t)Motor_RX_Buffer[1]) << 8) | (int16_t)Motor_RX_Buffer[2];
    DM_Motor_x->Ret_Data.Data.Speed = (Motor_RX_Buffer[3] << 4) | (Motor_RX_Buffer[4] >> 4);
    DM_Motor_x->Ret_Data.Data.Tprque = ((Motor_RX_Buffer[4] & 0xF) << 8) | Motor_RX_Buffer[5];

    DM_Motor_x->Ret_Value.Angle = 2 * P_MAX + uint_to_float(DM_Motor_x->Ret_Data.Data.Angle, P_MIN, P_MAX, 16);
    DM_Motor_x->Ret_Value.Speed = uint_to_float(DM_Motor_x->Ret_Data.Data.Speed, V_MIN, V_MAX, 12);
    DM_Motor_x->Ret_Value.Tprque = uint_to_float(DM_Motor_x->Ret_Data.Data.Tprque, T_MIN, T_MAX, 12);
    DM_Motor_Get_Angle_Sum(&(DM_Motor_x->Ret_Value.Angle_Sum_Process),  DM_Motor_x->Ret_Value.Angle,PIX2, 0.0f);
}

char DM_Motor_Get_Data(DM_Motor_Struct *DM_Motor_x, CAN_RxHeaderTypeDef *RxHeader, uint8_t *Motor_RX_Buffer)
{
    uint16_t RX_StdId = 0;
    DM_Motor_Ret_Para_Union Ret_Para = {0};

    Ret_Para.Buffer[0] = Motor_RX_Buffer[0];
    Ret_Para.Buffer[1] = Motor_RX_Buffer[1];
    RX_StdId = RxHeader->StdId;

    if (RX_StdId == DM_Motor_x->RX_STD_ID)
    {
        if (Ret_Para.Para.CAN_ID == DM_Motor_x->TX_STD_ID)
        {
            memcpy(DM_Motor_x->Ret_Para.Buffer, Motor_RX_Buffer, RxHeader->DLC);
        }
        else
        {
            DM_Motor_Set_Ret(DM_Motor_x, Motor_RX_Buffer);
        }

        return SUCCESS;
    }

    return ERROR;
}

void DM_Motor_Send_Cmd(DM_Motor_Struct *DM_Motor_x, int16_t TX_STD_ID, uint8_t Cmd[], uint8_t DLC)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    TxHeader.StdId = TX_STD_ID;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = DLC;

    if (BSP_CAN_TRANSMIT(DM_Motor_x->hcan, &TxHeader, Cmd, &TxMailbox) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }
}

void DM_Motor_Read_Data_Cmd(DM_Motor_Struct *DM_Motor_x)
{
    uint8_t Cmd[4] =
        {
            DM_Motor_x->TX_STD_ID & 0x00FF,
            DM_Motor_x->TX_STD_ID >> 8,
            0xCC,
            0X00};

    DM_Motor_Send_Cmd(DM_Motor_x, 0x7FF, Cmd, 4);
}

void DM_Motor_EMIT_Control(DM_Motor_Struct *DM_Motor_x, float Angle, float Speed, float Tprque)
{
    uint16_t Speed_Data = 0;
    uint16_t Tprque_Data = 0;
    Speed_Data = (uint16_t)(Speed * 100);
    Tprque_Data = (uint16_t)(Tprque * 10000 / 10.261f);

    DM_Motor_EMIT_Union EMIT_Data;
    EMIT_Data.Data.Angle = Angle;
    EMIT_Data.Data.Speed = Speed_Data;
    EMIT_Data.Data.Tprque = Tprque_Data;
    DM_Motor_Send_Cmd(DM_Motor_x, 0x300 + DM_Motor_x->TX_STD_ID, EMIT_Data.Buffer, 8);
}

void DM_Motor_Basic_Cmd(DM_Motor_Struct *DM_Motor_x, DM_Motor_Basic_Cmd_enum Basic_Cmd)
{
    uint8_t Cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, Basic_Cmd};
    DM_Motor_Send_Cmd(DM_Motor_x, DM_Motor_x->TX_STD_ID, Cmd, 8);
}

void DM_Motor_Save_Angle_Zero(DM_Motor_Struct *DM_Motor_x)
{
    DM_Motor_Basic_Cmd(DM_Motor_x, DM_SAVE_ANGLE_ZERO);
}

void DM_Motor_Set_State(DM_Motor_Struct *DM_Motor_x, FunctionalState State)
{

    DM_Motor_Basic_Cmd(DM_Motor_x, (State == ENABLE) ? DM_ENABLE : DM_DISABLE);
}

void DM_Motor_Clear_Error(DM_Motor_Struct *DM_Motor_x)
{
    DM_Motor_Basic_Cmd(DM_Motor_x, DM_CLEAR_ERROR);
}

void DM_Motor_Read_Para_Cmd(DM_Motor_Struct *DM_Motor_x, DM_Motor_Register_enum Reg_ID)
{
    uint8_t Cmd[8] = {
        DM_Motor_x->TX_STD_ID & 0x00FF,
        DM_Motor_x->TX_STD_ID >> 8,
        DM_PARA_READ,
        Reg_ID,
        0xFF,
        0xFF,
        0xFF,
        0xFF};
    DM_Motor_Send_Cmd(DM_Motor_x, 0x7FF, Cmd, 8);
}

void DM_Motor_Write_Para_Cmd(DM_Motor_Struct *DM_Motor_x, DM_Motor_Register_enum Reg_ID, uint8_t *Array)
{
    uint8_t Cmd[8] = {
        DM_Motor_x->TX_STD_ID & 0x00FF,
        DM_Motor_x->TX_STD_ID >> 8,
        DM_PARA_WRITE,
        Reg_ID,
        Array[0],
        Array[1],
        Array[2],
        Array[3]};
    DM_Motor_Send_Cmd(DM_Motor_x, 0x7FF, Cmd, 8);
}

void DM_Motor_Save_Para_Cmd(DM_Motor_Struct *DM_Motor_x)
{
    uint8_t Cmd[4] = {
        DM_Motor_x->TX_STD_ID & 0x00FF,
        DM_Motor_x->TX_STD_ID >> 8,
        DM_PARA_SAVE,
        0x01,
    };
    DM_Motor_Send_Cmd(DM_Motor_x, 0x7FF, Cmd, 4);
}
