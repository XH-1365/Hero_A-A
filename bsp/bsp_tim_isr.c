/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-14 15:48:39
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-09-30 19:13:53
 * @FilePath: \RM_Template\bsp\bsp_tim_isr.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp_tim.h"
#include "daemon.h"
#include "DJI_Motor.h"
#include "chassis.h"
#include "gimbal.h"
#include "vofa.h"
#include "mini_pc.h"
#include "DJI_DR16.h"
#include "DJI_VT13.h"
#include "shoot.h"
#include "robot_cmd.h"
#include "CH104_IMU_USART.H"
#include "daemon.h"
#include "DTOF.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // uint8_t i = 0;
  if (htim == &htim7)
  {
    Robot_Cmd_Timing_Handle();
    DR16_Timing_Handle();
    DJI_Motor_Timing_Handle();
    Chassis_Timing_Handle();
    Gimbal_Timing_Handle();
    Shoot_Timing_Handle();
      
    #if huart8_control
    Vofa_Timing_Handle();
    Gimbal_Vofa_Tx_Handle();
    #else
    MINI_PC__Timing_Handle();
    #endif
   
    CH104_IMU_USART_Timing_Handle();
    Daemon_Timing_Handle();
    DTOF_IIC_Timing_Handle();
    VT13_Timing_Handle();
  }
}
