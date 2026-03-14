/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2025-03-19 02:59:15
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-20 00:08:01
 * @FilePath: \RM_Hero_Down_Board\modules\Speed_Ramp.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "Speed_Ramp.h"
#include <math.h>
#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

void Speed_Ramp_Init(SpeedRampController *ramp, float max_acceleration, float max_deceleration, float max_speed)
{
    ramp->max_acceleration = max_acceleration;
    ramp->max_deceleration = max_deceleration;
    ramp->max_speed = max_speed;
}
/**
 * @brief 应用速度渐变控制
 * @param ramp 速度渐变控制器
 * @param target_speed 目标速度
 * @param dt 时间间隔（秒）
 * @return 更新后的当前速度
 */
float Apply_Speed_Ramp(SpeedRampController *ramp, float target_speed, float dt)
{
    float direction = 0.0f;
    float max_accel = 0.0f;
    // 计算速度差值
    float speed_diff = target_speed - ramp->current_speed;

    // 判断是加速还是减速
    // float max_accel = (speed_diff > 0) ? ramp->max_acceleration : ramp->max_deceleration;
    if (target_speed > ramp->current_speed)
    {
        max_accel = (ramp->current_speed >= 0.0f) ? ramp->max_acceleration : ramp->max_deceleration;
        direction = 1.0f;
    }
    else if (target_speed < ramp->current_speed)
    {
        max_accel = (ramp->current_speed > 0.0f) ? ramp->max_deceleration : ramp->max_acceleration;
        direction = -1.0f;
    }

    // 应用加速度限制
    float allowed_diff = copysignf(
        fminf(fabsf(speed_diff), max_accel * dt),
        direction);

    // 更新当前速度
    ramp->current_speed += allowed_diff;

    // 应用速度限制
    ramp->current_speed = CLAMP(ramp->current_speed, -ramp->max_speed, ramp->max_speed);
    ramp->output_speed = ramp->current_speed;
    return ramp->output_speed;
}
