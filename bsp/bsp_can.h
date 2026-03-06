/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-12 11:52:01
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-08 20:01:03
 * @FilePath: \CAN_BSP\bsp\bsp_can.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _BSP_CAN_H__
#define _BSP_CAN_H__
#include "can.h"
#pragma pack(1) // 指定结构体按照1字节对齐
typedef enum __BSP_CAN_Filter_ID_enum
{
    // CAN1接收过滤器ID:0~13
    CAN1_FILTER_ID_CHASSIS_GIMBAL = 0,
    CAN1_FILTER_ID_DM_MOTOR,
    CAN1_FILTER_ID_CAN_COMM,
    
    CAN1_FILTER_ID_SUM,

    // CAN2接收过滤器ID:14~27
    CAN2_FILTER_ID_SHOOT = 14,
    CAN2_FILTER_ID_CH104_IMU,
    CAN2_FILTER_ID_SUPER_CAP,
    CAN2_FILTER_ID_SUM
} BSP_CAN_Filter_ID_enum;

typedef union
{
    struct
    {

        uint16_t Low;
        uint16_t High;
    } Value;

    struct
    {
        uint8_t REV : 1;    ///< [0]    ：未使用
        uint8_t RTR : 1;    ///< [1]    : RTR 0：数据帧，1：遥控帧（远程帧）
        uint8_t IDE : 1;    ///< [2]    : IDE 0：标准帧，1：拓展帧
        uint32_t EXID : 18; ///< [21:3] : 存放扩展帧ID
        uint16_t STID : 11; ///< [31:22]: 存放标准帧ID
    } Sub;
} Bsp_CAN_RX_Filter_FilterId_TypeDef;

typedef struct __Bsp_CAN_RX_Filter_Struct
{
    Bsp_CAN_RX_Filter_FilterId_TypeDef Filter_ID;
    Bsp_CAN_RX_Filter_FilterId_TypeDef Filter_Mask_ID; // 掩码 ，掩码位为1时需要进行比较，为0时则可以时任意值，
    BSP_CAN_Filter_ID_enum FilterBank;
    uint8_t SlaveStartFilterBank;
    CAN_HandleTypeDef *hcan;
    uint32_t fifox; // fifo 0~1
    uint32_t FilterActivation;
} Bsp_CAN_RX_Filter_Struct;
void Bsp_CAN_RX_Filter_Set(Bsp_CAN_RX_Filter_Struct *RX_Filter_Para);

#pragma pack() // 取消结构体对齐

extern uint8_t Bsp_CAN1_Fifo0_RX_Data[8]; // CAN1_fifo0的接收缓存
extern uint8_t Bsp_CAN1_Fifo1_RX_Data[8]; // CAN1_fifo1的接收缓存
extern uint8_t Bsp_CAN2_Fifo1_RX_Data[8]; // CAN2_fifo1的接收缓存
#define BSP_CAN_TRANSMIT(hcanx, TxHeaderx, TxDatax, TxMailboxx) Bsp_CAN_Transmit(hcanx, TxHeaderx, TxDatax, TxMailboxx)
#define BSP_CAN_STOP(hcanx) HAL_CAN_Stop(hcanx)
#define BSP_CAN_START(hcanx) HAL_CAN_Start(hcanx)
void Bsp_CAN_Init(void);
#endif
