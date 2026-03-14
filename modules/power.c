#include "power.h"
#include <math.h>

/// @brief 计算一元二次方程的解
/// @param a 二次项系数
/// @param b 一次项系数
/// @param c 常数项
/// @param x1 指向第一个解的指针 当只有一个解时，输出值为x1
/// @param x2 指向第二个解的指针
/// @return -1表示无解，0表示有一个解，1表示有两个解
static int8_t Quadratic_formula(float a, float b, float c, float *x1, float *x2)
{
    float Delta = b * b - 4 * a * c;
    if (Delta < 0)
    {
        return -1; // 无解
    }
    else if (Delta == 0)
    {
        *x1 = -b / (2 * a);
        return 0; // 一个解
    }
    else
    {
        *x1 = (-b + sqrtf(Delta)) / (2 * a);
        *x2 = (-b - sqrtf(Delta)) / (2 * a);
        return 1; // 两个解
    }
}

/// @brief 根据功率值返回索引
/// @param Power 功率值
/// @return 索引值
static uint8_t Index(float Power)
{
    return Power > 0.0f ? 1.0f : 0.0f;
}

/// @brief 计算3508电机的功率
/// @param Motor_Speed 电机转速 单位：rad/s
/// @param Current 电流 单位：A
/// @return 功率 单位：w
float Motor_3508_Calculat_Power(float Motor_Speed, float Current)
{
    float Power = 0.0f; // 功率 单位：w
    const float K0 = 0.6641993427076276f;
    const float K1 = 0.006444303471551991f;
    const float K2 = 0.00014238543496646608f;
    const float K3 = 0.01764443020460121f;
    const float K4 = 0.16501438506607852f;
    const float K5 = 3.0967217257287935e-05f;

    // 以下为电机功率预测公式，可通过电机反馈回来的电流和转速求出单个电机的功率
    // 公式来源链接：https://www.bilibili.com/video/BV1tyfdY4ERb?spm_id_from=333.788.videopod.sections&vd_source=a7f7bb1f8bac687ec0949d8cde7f1ab9
    Power = K0 +
            K1 * Current +
            K2 * Motor_Speed +
            K3 * Current * Motor_Speed +
            K4 * Current * Current +
            K5 * Motor_Speed * Motor_Speed;

    return Power;
}

/**
 * @brief 计算电机的总功率
 *
 * 该函数接收一个功率值数组和数组的长度，计算并返回数组中每个功率值乘以其索引的加权和。
 *
 * @param Power_Value 功率值数组
 * @param Power_Value_Num 功率值数组的长度
 * @return float 返回功率值数组的加权和
 */
float Calculat_Power_Sum(float Power_Value[], uint8_t Power_Value_Num)
{
    uint8_t i = 0;
    float Power_Sum = 0.0f;
    for (i = 0; i < Power_Value_Num; i++)
    {
        Power_Sum += Power_Value[i] * Index(Power_Value[i]);
    }
    //以下为测试比完赛后记得按需改正
    return Power_Sum*0.8f;
}

///也许可以试试这个把衰减系数同时乘上电流和速度
/// @brief 计算功率衰减系数0~1
/// @param Power_Max_Limt 最大功率上限 单位：w
/// @param Power_Sum 底盘功率总和 单位：w
/// @return 衰减系数
float Power_Calculat_Damping_Coefficient(float Power_Max_Limt, float Power_Sum)
{
    float Zeta = 0.0f;

    // 求出衰减系数
    Zeta = Power_Max_Limt / Power_Sum;
    if (Zeta > 1.0f)
    {
        Zeta = 1.0f;
    }
    return Zeta;
}

/**
 * 
 * @brief 计算电机的目标电流  好像没啥用？即使计算出目标电流，也没有输入的控制参数，有点不明所以
 *
 * 该函数根据当前电机功率、电机速度和电机电流，计算每个电机的目标电流。
 *
 * @param Zeta 电流衰减系数
 * @param Motor_Power_Now 当前电机功率数组 单位： w
 * @param Motor_Speed_Now 当前电机速度数组 单位： rad/s
 * @param Motor_Current_Now 当前电机电流数组  单位： A
 * @param Motor_Current_Tar 目标电机电流数组  单位： A
 * @param Motor_Num 电机数量
 *
 * @note 该函数使用了一个二次方程来计算目标电流，并根据计算结果更新目标电流数组。
 */
void Power_Calculat_Current_Tar(float Zeta, float Motor_Power_Now[], float Motor_Speed_Now[], float Motor_Current_Now[], float Motor_Current_Tar[], uint8_t Motor_Num)
{
    const float K0 = 0.6641993427076276f;
    const float K1 = 0.006444303471551991f;
    const float K2 = 0.00014238543496646608f;
    const float K3 = 0.01764443020460121f;
    const float K4 = 0.16501438506607852f;
    const float K5 = 3.0967217257287935e-05f;

    uint8_t i = 0;
    int8_t Delta = 0;
    float a, b, c, x1, x2;

    for (i = 0; i < Motor_Num; i++)
    {
        a = Zeta * K4;
        b = K1 * Zeta + Zeta * K3 * Motor_Speed_Now[i];
        c = Zeta * K0 + Zeta * K2 * Motor_Speed_Now[i] + Zeta * K5 * Motor_Speed_Now[i] * Motor_Speed_Now[i] - Motor_Power_Now[i];
        Delta = Quadratic_formula(a, b, c, &x1, &x2);
        if (Delta == 1)
        {
            if ((Motor_Current_Now[i] - x1) < (Motor_Current_Now[i] - x2))
            {
                Motor_Current_Tar[i] = x1;
            }
            else
            {
                Motor_Current_Tar[i] = x2;
            }
        }
        else if (Delta == 0)
        {
            Motor_Current_Tar[i] = x1;
        }
        else if (Delta == -1)
        {
            Motor_Current_Tar[i] = 0;
        }
    }
}
