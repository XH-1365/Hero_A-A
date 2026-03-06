/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2024-10-19 20:52:38
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-15 05:48:26
 * @FilePath: \RM_Hero_UP_Board\modules\can_comm.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CAN_COMM_H__
#define _CAN_COMM_H__
#include "RM_typedef.h"
#define CAN_COMM_UP_BOARD_DLC 4
#define CAN_COMM_UP_BOARD_TX_ID 0x300
#define CAN_COMM_UP_BOARD_RX_ID 0x301
#pragma pack(1) 
typedef union __CAN_Comm_RX_UP_Board_Union
{
    struct
    {
    float Chassis_OMEGA_Speed; // 底盘的旋转角速度单位  0.01 rad/s
    uint16_t Shoot_Heat;          // 发生器热量
    uint8_t  Robot_Level;     //机器人等级 1~10
    } Data;

    uint8_t Buffer[7];

} CAN_Comm_RX_UP_Board_Union;

typedef struct __CAN_Comm_Struct
{
    uint16_t Flag;
    uint16_t Count;
    CAN_HandleTypeDef *hcan;   // 使用的CAN口，hcan1或者hcan2
    int16_t TX_STD_ID;         // CAN发送数据帧ID
    int16_t RX_STD_ID;         // CAN接收数据帧ID
    uint8_t *RX_Data;          // CAN接收到的数据
     uint8_t RX_Data_Length; // CAN接收数据长度
} CAN_Comm_Struct;

#pragma pack() 
extern CAN_Comm_RX_UP_Board_Union UP_Board_RX_Data;
extern CAN_Comm_Struct UP_Board;
void CAN_Comm_Init(void);
char CAN_Comm_RX_Callback(CAN_Comm_Struct *Instance, CAN_RxHeaderTypeDef *RxHeader, uint8_t * RX_Buffer);
void CAN_Comm_TX_Send_Data(CAN_Comm_Struct *Instance, uint8_t *Data, uint8_t DLC);
void CAN_Comm_Get_RX_Data(CAN_Comm_Struct *Instance,void * Buffer,uint8_t size_t );

#endif


