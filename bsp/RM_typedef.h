/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-14 14:25:52
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-20 16:20:10
 * @FilePath: \RM_Hero_UP_Board\bsp\RM_typedef.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _RM_TYPEDEF__
#define _RM_TYPEDEF__
#include "stm32f4xx_hal.h"

#define PI 3.14159265359f
#define PIX2 6.2831852f
//转速单位转换 RPM->rad/s 用法：把单位为RPM的转速乘以这个数值就可以得到单位为rad/s的转速
#define RPM_TO_RADIAN 0.10471975511965977f
//归一化函数，将数值归一化到-1~1范围
#define NORMALIZE(x, min, max) (((2.0f * ((x) - (min))) / ((max) - (min))) - 1.0f)


typedef FunctionalState Fn_State_t;
#endif
