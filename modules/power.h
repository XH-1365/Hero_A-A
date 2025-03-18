#ifndef _POWER_H__
#define _POWER_H__

#include "RM_typedef.h"


/// @brief 计算3508电机的功率
/// @param Motor_Speed 电机转速 单位：rad/s
/// @param Current 电流 单位：A
/// @return 功率 单位：w
float Motor_3508_Calculat_Power(float Motor_Speed, float Current);

/**
 * @brief 计算电机的总功率
 * 
 * 该函数接收一个功率值数组和数组的长度，计算并返回数组中每个功率值乘以其索引的加权和。
 * 
 * @param Power_Value 功率值数组
 * @param Power_Value_Num 功率值数组的长度
 * @return float 返回功率值数组的加权和
 */
float Calculat_Power_Sum(float Power_Value[], uint8_t Power_Value_Num);

/// @brief 计算衰减系数0~1
/// @param Power_Max_Limt 最大功率上限 单位：w
/// @param Power_Sum 底盘功率总和 单位：w
/// @return 衰减系数
float Power_Calculat_Damping_Coefficient(float Power_Max_Limt, float Power_Sum);


/**
 * @brief 计算电机的目标电流
 * 
 * 该函数根据当前电机功率、电机速度和电机电流，计算每个电机的目标电流。
 * 
 * @param Zeta 电流衰减系数
 * @param Motor_Power_Now 当前电机功率数组
 * @param Motor_Speed_Now 当前电机速度数组
 * @param Motor_Current_Now 当前电机电流数组
 * @param Motor_Current_Tar 目标电机电流数组
 * @param Motor_Num 电机数量
 * 
 * @note 该函数使用了一个二次方程来计算目标电流，并根据计算结果更新目标电流数组。
 */
void Power_Calculat_Current_Tar( float Zeta, float Motor_Power_Now[], float Motor_Speed_Now[], float Motor_Current_Now[], float Motor_Current_Tar[], uint8_t Motor_Num);

#endif

