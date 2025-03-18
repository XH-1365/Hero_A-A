#include "filter.h"

/// @brief 初始化低通滤波器
/// @param Filter 指向低通滤波器结构体的指针
/// @param Alpha 滤波器的权重因子 越小滤波效果越好但是响应速度越慢
/// @param Filtered_Value 初始的滤波值
void Low_Pass_Filter_Init(Low_Pass_Filter_Struct *Filter, float Alpha, float Filtered_Value)
{
    Filter->Alpha = Alpha;
    Filter->Filtered_Value = Filtered_Value;
}

/// @brief 应用低通滤波器进行滤波
/// @param Filter 指向低通滤波器结构体的指针
/// @param Alpha 滤波器的权重因子 越小滤波效果越好但是响应速度越慢
/// @param Filter_Input 需要滤波的输入值
/// @return 滤波后的输出值
float Low_Pass_Filter(Low_Pass_Filter_Struct *Filter, float Filter_Input)
{
    // 进行低通滤波
    Filter->Filtered_Value = Filter->Alpha * Filter_Input + (1 - Filter->Alpha) * Filter->Filtered_Value;
    Filter->Output = Filter->Filtered_Value;
    return  Filter->Output;
}
