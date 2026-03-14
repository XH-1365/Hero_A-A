#ifndef __SPEED_RAMP__
#define __SPEED_RAMP__

#include "RM_typedef.h"
typedef struct
{
    float current_speed; // 当前实际速度
    float max_acceleration; // 最大加速度（用于加速）
    float max_deceleration; // 最大减速度（用于减速）
    float max_speed;        // 最大速度限制
    float output_speed;//输出速度值
} SpeedRampController;

float Apply_Speed_Ramp(SpeedRampController *ramp, float target_speed, float dt);
void Speed_Ramp_Init(SpeedRampController *ramp, float max_acceleration,float max_deceleration,float max_speed);
#endif
