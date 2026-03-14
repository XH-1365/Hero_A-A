/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-14 14:25:52
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-05-19 22:15:47
 * @FilePath: \RM_Hero_UP_Board\bsp\RM_typedef.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _RM_TYPEDEF__
#define _RM_TYPEDEF__
#include "stm32f4xx_hal.h"
#include "robot_config.h"
#define PI 3.1415926f
#define PIX2 6.2831852f
//转速单位转换 RPM->rad/s 用法：把单位为RPM的转速乘以这个数值就可以得到单位为rad/s的转速
#define RPM_TO_RADIAN 0.10471975511965977f
#define RADIAN_TO_RPM 9.549296586f

//归一化函数，将数值归一化到-1~1范围
#define NORMALIZE(x, min, max) (((2.0 * ((x) - (min))) / ((max) - (min))) - 1.0)

#define DEGREE_TO_RAD 0.01745329252f // pi/180角度转弧度

#endif
