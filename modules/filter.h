/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2025-01-17 15:44:47
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-10 23:26:22
 * @FilePath: \RM_Hero_UP_Board\modules\filter.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _FILTER_H__
#define _FILTER_H__
#include "RM_typedef.h"

typedef struct __Low_Pass_Filter_Struct
{
    float Alpha;          // 权重因子
    float Filtered_Value; // 上一次的滤波值
    float Output;//输出值
} Low_Pass_Filter_Struct;

/// @brief 应用低通滤波器进行滤波
/// @param Filter 指向低通滤波器结构体的指针
/// @param Alpha 滤波器的权重因子 越小滤波效果越好但是响应速度越慢
/// @param Filter_Input 需要滤波的输入值
/// @return 滤波后的输出值
float Low_Pass_Filter(Low_Pass_Filter_Struct *Filter, float Filter_Input);

/// @brief 初始化低通滤波器
/// @param Filter 指向低通滤波器结构体的指针
/// @param Alpha 滤波器的权重因子 越小滤波效果越好但是响应速度越慢
/// @param Filtered_Value 初始的滤波值
void Low_Pass_Filter_Init(Low_Pass_Filter_Struct *Filter, float Alpha, float Filtered_Value);

#endif
