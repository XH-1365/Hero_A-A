#include "gimbal_solver.h"
#include <math.h>

/// @brief 计算一元二次方程的解
/// @param a 二次项系数
/// @param b 一次项系数
/// @param c 常数项
/// @param x1 指向第一个解的指针
/// @param x2 指向第二个解的指针
static void Quadratic_formula(float a, float b, float c, float *x1, float *x2)
{
    float Delta = 0;
    Delta = b * b - 4 * a * c;
    (*x1) = (-b + sqrtf(Delta)) / (2 * a);
    (*x2) = (-b - sqrtf(Delta)) / (2 * a);
}


/**
 * @brief 生成对称三角波，范围 [-amp, amp]
 * @param freq 三角波频率 (Hz)
 * @param amp  三角波幅值 (与速度指令单位一致)
 * @param dt   控制周期 (秒)，即两次调用之间的时间间隔
 * @return     当前时刻的三角波值
 */
float triangle_wave(float freq, float amp, float dt) {
    static float phase = 0.0f;          // 相位 [0, 1)
    
    // 更新相位
    phase += freq * dt;
    if (phase >= 1.0f) {
        phase -= 1.0f;
    }
    
    // 计算三角波：y = amp * (1 - 4 * |t - 0.5|)
    float t = phase;
    return amp * (1.0f - 4.0f * fabsf(t - 0.5f));
}


/// @brief 输入角度得到丝杆的位置
/// @param screw_position 单位：弧度
/// @return 丝杆的位置 单位：mm
float Angle_To_Screw_Pitch_Length_Calculate(float Pitch_Angle)
{
    float d = 0;
    float d_square = 0;
    float a1 = 31.42f;
    float c2 = 64.1515f;
    float D = Pitch_Angle;
    float A = 0.9253;
    float A1 = 0;
    float A2 = 0;
    float a2 = 40.27f;
    float a2_square = 0;
    float c1 = 0;
    float x1 = 0;
    float x2 = 0;

    d_square = a1 * a1 + c2 * c2 - 2 * a1 * c2 * cosf(D);
    d = sqrtf(d_square);
    A1 = asinf((a1 / d) * sinf(D));
    A2 = A - A1;
    a2_square = a2 * a2;
    Quadratic_formula(1.0f, -2 * d * cosf(A2), d_square - a2_square, &x1, &x2);
    if (x1 <= 45 && x1 > 0)
    {
        c1 = x1;
    }
    else if (x2 <= 45 && x2 > 0)
    {
        c1 = x2;
    }
    return c1;
}

/// @brief 输入电机的角度得到丝杆的位置
/// @param M2006_Angle 电机角度 单位：弧度（rad）
/// @return 丝杆位置单位：mm
static float M2006_Angle_To_Screw_Length(float M2006_Angle)
{
    return (M2006_Angle / (PIX2 * 36.0f)) * 12.0f;
}

/// @brief 输入丝杆位置得到角度
/// @param Screw_Length 单位：mm
/// @return 云台的角度 单位：弧度(rad)
static float Screw_Pitch_Length_To_Angle_Calculate(float Screw_Length)
{
    const float b1 = 64.1515f;
    const float b2 = 31.43f;
    const float d1 = 40.27f;
    const float COSA = 0.601594979f;
    const float SINA = 0.7988012776f;

    float d2 = Screw_Length;
    float a_square = (b1 * b1) + (d2 * d2) - (2 * b1 * d2 * COSA);
    float a = sqrtf(a_square);

    float COSD1 = ((b2 * b2) + a_square - (d1 * d1)) / (2 * b2 * a);
    float D1 = acosf(COSD1);
    float D2 = asinf((d2 * SINA) / a);

    // return (D1 + D2 - 0.5731f - 0.6455f);
    return -(D1 + D2 - 0.5731f - 0.6455f);//这里的负号是因为电机角度的方向是反向的
}

/// @brief 输入电机的角度得到丝杆的位置
/// @param M2006_Angle 电机角度 单位：弧度（rad）
/// @return 云台的角度 单位：弧度(rad)
float Gimbal_Screw_Pitch_Get_Angle(float M2006_Angle)
{
    float screw_length = M2006_Angle_To_Screw_Length(M2006_Angle);
    return Screw_Pitch_Length_To_Angle_Calculate(screw_length);
}

