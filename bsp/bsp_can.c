/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-12 11:50:54
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-09-11 19:36:46
 * @FilePath: \CAN_BSP\bsp\bsp_can.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp_can.h"
#include "can.h"
// #include "usart.h"
#include <stdio.h>
#include <string.h>
uint8_t Bsp_CAN1_Fifo0_RX_Data[8]; // CAN1_fifo0的接收缓存
uint8_t Bsp_CAN1_Fifo1_RX_Data[8]; // CAN1_fifo1的接收缓存
uint8_t Bsp_CAN2_Fifo1_RX_Data[8]; // CAN2_fifo1的接收缓存
void Bsp_CAN_RX_Filter_Set(Bsp_CAN_RX_Filter_Struct *RX_Filter_Para)
{
    CAN_FilterTypeDef can_filter_conf;

    can_filter_conf.FilterBank = RX_Filter_Para->FilterBank;             // 过滤器组编号  CAN过滤器有很多个选择其中一个即可
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;                  // id屏蔽模式    屏蔽模式就是指定一个ID 并设置相关掩码允许ID在某个范围内通过过滤器
                                                                         // 这里过滤的ID为0X200~0X20F
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;                 // 32bit 滤波，此处一般用于拓展帧的滤波
    can_filter_conf.FilterIdHigh = RX_Filter_Para->Filter_ID.Value.High; // high 16 bit  用于匹配的ID 因为ID为11位，这个滤波寄存器为16位，且ID要向左对齐
                                                                         // 所以要向左移5位，因为ID的最高位为第11位向左对齐就直接向左移动5位即11+5=16，正好是16位最高位

    can_filter_conf.FilterIdLow = RX_Filter_Para->Filter_ID.Value.Low;            // low 16bit
    can_filter_conf.FilterMaskIdHigh = RX_Filter_Para->Filter_Mask_ID.Value.High; // 掩码 ，掩码位为1时需要进行比较，为0时则可以时任意值，
                                                                                  // 这里过滤的ID为0X200~0X20F所以12位数据中高8位需要比较，低4位为任意值
                                                                                  // 所以掩码为0x7F0
                                                                                  // 至于为何要左移5位原因和上面一样
    can_filter_conf.FilterMaskIdLow = RX_Filter_Para->Filter_Mask_ID.Value.Low;
    can_filter_conf.FilterFIFOAssignment = RX_Filter_Para->fifox; // 选择CAN接收FIFO0
    can_filter_conf.SlaveStartFilterBank = RX_Filter_Para->SlaveStartFilterBank; // can1(0-13)和can2(14-27)分别得到一半的filter
    can_filter_conf.FilterActivation = RX_Filter_Para->FilterActivation;
    HAL_CAN_ConfigFilter(RX_Filter_Para->hcan, &can_filter_conf);
}

void Bsp_CAN_Init(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}
