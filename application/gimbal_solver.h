#ifndef _GIMBAL_SOLVER_H__
#define _GIMBAL_SOLVER_H__
#include "RM_typedef.h"

/// @brief 输入电机的角度得到丝杆的位置
/// @param M2006_Angle 电机角度 单位：弧度（rad）
/// @return 云台的角度 单位：弧度(rad)
float Gimbal_Screw_Pitch_Get_Angle(float M2006_Angle);
float triangle_wave(float freq, float amp, float dt);
#endif
