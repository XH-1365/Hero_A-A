/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-08-24 12:01:25
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-09-29 16:26:11
 * @FilePath: \RM_Template\modules\CH104_IMU_CAN.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "CH104_IMU_CAN.h"
#include "CH104_IMU_CAN_CMD.h"
#include "bsp_can.h"
#include <string.h>

CH104_IMU_CAN_Struct CH104_IMU_CAN;
static void CH104_IMU_CAN_Data_Init(void)
{
    CH104_IMU_CAN.hcan = &hcan2;

    CH104_IMU_CAN.Acceleration.RX_STD_ID = ACC_RX_ID;

    CH104_IMU_CAN.Angular_Velocity.RX_STD_ID = GYRO_RX_ID;

    CH104_IMU_CAN.Euler_Angles.RX_STD_ID = EULER_ANGLES_RX_ID;

    CH104_IMU_CAN.Quaternion.RX_STD_ID = QUAT_RX_ID;

    CH104_IMU_CAN.Pressure.RX_STD_ID = PRESSURE_RX_ID;

    CH104_IMU_CAN.Tilt_Sensor.RX_STD_ID = TILT_SENSOR_RX_ID;
}

static void CH104_IMU_CAN_RX_Filter_Set(void)
{

    Bsp_CAN_RX_Filter_Struct Bsp_CAN_RX_Filter_config;

    // 过滤器匹配ID设置
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.STID = 0x080 + NODE_ID;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.EXID = 0x0000;
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.IDE = 0; // 为标准帧
    Bsp_CAN_RX_Filter_config.Filter_ID.Sub.RTR = 0; // 为数据帧

    // 掩码 ，掩码位为1时需要进行比较，为0时则可以时任意值，
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.STID = 0x0FF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.EXID = 0xFFFF;
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.IDE = 1; // 必须为标准帧
    Bsp_CAN_RX_Filter_config.Filter_Mask_ID.Sub.RTR = 1; // 必须为数据帧

    Bsp_CAN_RX_Filter_config.FilterBank = 14;           // 过滤器组编号  CAN过滤器有很多个选择其中一个即可
    Bsp_CAN_RX_Filter_config.SlaveStartFilterBank = 14; // 起始过滤器编号 can1(0-13)和can2(14-27)分别得到一半的filter
    Bsp_CAN_RX_Filter_config.hcan = &hcan2;             // 选择CAN1或者CAN2
    Bsp_CAN_RX_Filter_config.fifox = CAN_FilterFIFO1;
    Bsp_CAN_RX_Filter_config.FilterActivation = CAN_FILTER_ENABLE;
    Bsp_CAN_RX_Filter_Set(&Bsp_CAN_RX_Filter_config);
}

void CH104_IMU_CAN_Init(void)
{
    CH104_IMU_CAN_Data_Init();
    CH104_IMU_CAN_RX_Filter_Set();
}

char CH104_IMU_CAN_Get_Data(CH104_IMU_CAN_Struct *CH104_IMU_CAN, CAN_RxHeaderTypeDef *RxHeader, uint8_t *CH104_IMU_CAN_RX_Buffer)
{

    switch (RxHeader->StdId)
    {
    case ACC_RX_ID:
        memcpy(&(CH104_IMU_CAN->Acceleration.Data), CH104_IMU_CAN_RX_Buffer, sizeof(CH104_IMU_CAN_Acceleration_Data_Struct));
        break;

    case GYRO_RX_ID:
        memcpy(&(CH104_IMU_CAN->Angular_Velocity.Data), CH104_IMU_CAN_RX_Buffer, sizeof(CH104_IMU_CAN_Angular_Velocity_Data_Struct));
        break;

    case EULER_ANGLES_RX_ID:
        memcpy(&(CH104_IMU_CAN->Euler_Angles.Data), CH104_IMU_CAN_RX_Buffer, sizeof(CH104_IMU_CAN_Euler_Angles_Data_Struct));
        break;

    case QUAT_RX_ID:
        memcpy(&(CH104_IMU_CAN->Quaternion.Data), CH104_IMU_CAN_RX_Buffer, sizeof(CH104_IMU_CAN_Quaternion_Data_Struct));
        break;

    case PRESSURE_RX_ID:
        memcpy(&(CH104_IMU_CAN->Pressure.Data), CH104_IMU_CAN_RX_Buffer, sizeof(CH104_IMU_CAN_Pressure_Data_Struct));
        break;

    case TILT_SENSOR_RX_ID:
        memcpy(&(CH104_IMU_CAN->Tilt_Sensor.Data), CH104_IMU_CAN_RX_Buffer, sizeof(CH104_IMU_CAN_Tilt_Sensor_Data_Struct));
        break;

    default:
        return ERROR;
    }

    return SUCCESS;
}

void CH104_IMU_CAN_Send_Cmd(CH104_IMU_CAN_Struct *CH104_IMU_CAN, uint32_t TX_STD_ID, uint8_t Cmd[], uint8_t DLC)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    TxHeader.StdId = TX_STD_ID;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = DLC;

    if (BSP_CAN_TRANSMIT(CH104_IMU_CAN->hcan, &TxHeader, Cmd, &TxMailbox) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }
}

void CH104_IMU_CAN_Set_Baudrate(CH104_IMU_CAN_Struct *CH104_IMU_CAN, uint32_t CH104_IMU_CAN_Baudrate)
{
    CH104_IMU_CAN_CMD_Union CH104_IMU_CAN_CMD;
    CH104_IMU_CAN_CMD.CMD_Struct.CMD = 0x23;
    CH104_IMU_CAN_CMD.CMD_Struct.Index = 0x2100;
    CH104_IMU_CAN_CMD.CMD_Struct.Sub_Index = 0x00;
    CH104_IMU_CAN_CMD.CMD_Struct.Data = CH104_IMU_CAN_Baudrate;
    CH104_IMU_CAN_Send_Cmd(CH104_IMU_CAN, 0x600 + NODE_ID, CH104_IMU_CAN_CMD.Array, sizeof(CH104_IMU_CAN_CMD_Union));

    //     uint8_t Data[8] = {0x23, 0x00, 0x21, 0x00, 0x40, 0x42, 0x0F, 0x00};
    //    CH104_IMU_CAN_Send_Cmd(CH104_IMU_CAN, 0x600 + NODE_ID, Data, sizeof(Data));
}

void CH104_IMU_CAN_Set_Node_ID(CH104_IMU_CAN_Struct *CH104_IMU_CAN, uint32_t Node_ID)
{
    CH104_IMU_CAN_CMD_Union CH104_IMU_CAN_CMD;
    CH104_IMU_CAN_CMD.CMD_Struct.CMD = 0x23;
    CH104_IMU_CAN_CMD.CMD_Struct.Index = 0x2101;
    CH104_IMU_CAN_CMD.CMD_Struct.Sub_Index = 0x00;
    CH104_IMU_CAN_CMD.CMD_Struct.Data = Node_ID;
    CH104_IMU_CAN_Send_Cmd(CH104_IMU_CAN, 0x600 + NODE_ID, CH104_IMU_CAN_CMD.Array, sizeof(CH104_IMU_CAN_CMD_Union));
}

void CH104_IMU_CAN_Enable(CH104_IMU_CAN_Struct *CH104_IMU_CAN)
{
    uint8_t Data = NODE_ID;
    CH104_IMU_CAN_Send_Cmd(CH104_IMU_CAN, 0x000, &Data, sizeof(Data));
}

void CH104_IMU_CAN_Set_Sensor_Speed(CH104_IMU_CAN_Struct *CH104_IMU_CAN, CH104_IMU_CAN_RX_Sensor_Speed_Index_enum Sensor_Speed_Index, CH104_IMU_CAN_RX_Sensor_Speed_enum Speed)
{
    CH104_IMU_CAN_CMD_Union CH104_IMU_CAN_CMD;
    CH104_IMU_CAN_CMD.CMD_Struct.CMD = 0x2B;
    CH104_IMU_CAN_CMD.CMD_Struct.Index = Sensor_Speed_Index;
    CH104_IMU_CAN_CMD.CMD_Struct.Sub_Index = 0x05;
    CH104_IMU_CAN_CMD.CMD_Struct.Data = Speed;
    CH104_IMU_CAN_Send_Cmd(CH104_IMU_CAN, 0x600 + NODE_ID, CH104_IMU_CAN_CMD.Array, sizeof(CH104_IMU_CAN_CMD_Union));
}

// void CH104_IMU_CAN_Save_Set(CH104_IMU_CAN_Struct *CH104_IMU_CAN)
// {
//     uint8_t Data[8] = {0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//     CH104_IMU_CAN_Send_Cmd(CH104_IMU_CAN, 0x07E5, Data, sizeof(Data));
// }

void CH104_IMU_CAN_Set(void)
{
    //    CH104_IMU_CAN_Set_Baudrate(&CH104_IMU_CAN, 1000000UL);
    //    CH104_IMU_CAN_Enable(&CH104_IMU_CAN);
    CH104_IMU_CAN_Set_Sensor_Speed(&CH104_IMU_CAN, ACC_SPEED_INDEX, SENSOR_CLOSE);
    HAL_Delay(100);
    CH104_IMU_CAN_Set_Sensor_Speed(&CH104_IMU_CAN, GYRO_SPEED_INDEX, SENSOR_CLOSE);
    HAL_Delay(100);
    CH104_IMU_CAN_Set_Sensor_Speed(&CH104_IMU_CAN, EULER_ANGLES_SPEED_INDEX, SENSOR_200HZ);
    HAL_Delay(100);
    CH104_IMU_CAN_Set_Sensor_Speed(&CH104_IMU_CAN, QUAT_SPEED_INDEX, SENSOR_CLOSE);
    HAL_Delay(100);
    // CH104_IMU_CAN_Save_Set(&CH104_IMU_CAN);
}
