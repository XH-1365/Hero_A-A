/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-14 14:25:52
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-17 15:58:35
 * @FilePath: \RM_Hero_UP_Board\application\app.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

/*--------应用层--------*/
#include "app.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"

/*--------模块层--------*/
#include "DJI_Motor.h"
#include "DJI_DR16.h"
#include "DJI_VT13.h"
#include "DM_Motor.h"
#include "vofa.h"
#include "CH104_IMU_CAN.h"
#include "CH104_IMU_USART.H"
#include "can_comm.h"
#include "referee_task.h"
#include "rm_referee.h"
#include "daemon.h"
#include "DTOF.h"
#include "super_cap.h"
#include "mini_pc.h"
/*--------bsp层--------*/
#include "RM_typedef.h"
#include "bsp_tim.h"
#include "bsp_usart.h"
#include "bsp_can.h"

/*--------驱动层--------*/
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

uint16_t Test = 0;
float DM_Angle_Test = 0;

void App_Init(void)
{

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

    #if huart8_control
        Vofa_Init();
    #else
        MINI_PC_Init();
    #endif
	CH104_IMU_USART_Init();
	// CH104_IMU_CAN_Init();
	CAN_Comm_Init();
	DM_Motor_Init();
	DJI_Motor_Init();
	Super_Cap_init();
	Bsp_CAN_Init();
	// CH104_IMU_CAN_Set();


	DJI_DR16_Init();
    DJI_VT13_Init();
	DTOF_Init();
	BSP_TIM_Init();
	BSP_USART_Init();

	Robot_Cmd_Init();
	Gimbal_Init();
	Shoot_Init();
    
}

void App_Run(void)
{

//	if (DR16_Get_Count() == 0)
//	{
//		return;
//	}
    if (VT13_Get_Count() == 0)
	{
		return;
	}
	Test++;

	Vofa_Receive_Handle();

	//	// HAL_Delay(500);
	// Vofa_Set_TX_Value(VOFA_TX_CURRENT, (float)(DTOF.Distance) / 10);
	// Vofa_Transmit();

	CH104_IMU_USART_Handle();
	Robot_Cmd_Task();
	Gimbal_Task();
	Shoot_Task();
	Daemon_Callback_Handle();
	DTOF_IIC_Handle();

}
