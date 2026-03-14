/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-20 16:56:52
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-22 13:24:55
 * @FilePath: \RM_Hero_Down_Board\modules\can_comm.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CAN_COMM_H__
#define _CAN_COMM_H__
#include "RM_typedef.h"
#define CAN_COMM_DOWN_BOARD_DLC 8
//#define CAN_COMM_DOWN_BOARD_TX_ID 0x300
//#define CAN_COMM_DOWN_BOARD_RX_ID 0x301

#define CAN_COMM_DOWN_BOARD_TX_ID 0x301
#define CAN_COMM_DOWN_BOARD_RX_ID 0x300
#pragma pack(1) 
typedef struct __CAN_Comm_Struct
{
    uint16_t Flag;
    uint16_t Count;
    CAN_HandleTypeDef *hcan;   // 使用的CAN口，hcan1或者hcan2
    int16_t TX_STD_ID;         // CAN发送数据帧ID
    int16_t RX_STD_ID;         // CAN接收数据帧ID
    uint8_t *RX_Data;          // CAN接收到的数据
    const uint8_t RX_Data_Length; // CAN接收数据长度
} CAN_Comm_Struct;
#pragma pack() 
extern CAN_Comm_Struct Down_Board;
void CAN_Comm_Init(void);
char CAN_Comm_RX_Callback(CAN_Comm_Struct *Instance, CAN_RxHeaderTypeDef *RxHeader, uint8_t * RX_Buffer);
void CAN_Comm_TX_Send_Data(CAN_Comm_Struct *Instance, uint8_t *Data, uint8_t DLC);
void *CAN_Comm_Get_RX_Data(CAN_Comm_Struct *Instance);

void CAN_Comm_Timing_Handle(void);

char CAN_Comm_Get_State(CAN_Comm_Struct *Instance);

#endif


