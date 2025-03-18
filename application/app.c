/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-10-14 14:25:52
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-15 01:02:42
 * @FilePath: \RM_Hero_Down_Board\application\app.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: 励磁器狂人 9300491+exciter-maniac@user.noreply.gitee.com
 * @Date: 2024-08-14 15:33:11
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-10-14 21:48:22
 * @FilePath: \MDK-ARMf:\RM_Program\RM_Code_Program\Template\RM_Template\RM_Template\application\app.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

/*--------应用层--------*/
#include "app.h"
#include "robot_cmd.h"
// #include "gimbal.h"
// #include "shoot.h"
#include "chassis.h"

/*--------模块层--------*/
#include "DJI_Motor.h"
#include "DJI_DR16.h"
#include "DM_Motor.h"
#include "vofa.h"
#include "CH104_IMU_CAN.h"
#include "CH104_IMU_USART.H"
#include "can_comm.h"
#include "super_cap.h"
#include "referee_task.h"
#include "rm_referee.h"
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

	//	CH104_IMU_USART_Init();

	Bsp_CAN_Init();
	DJI_Motor_Init();
	CAN_Comm_Init();
	DM_Motor_Init();
	Super_Cap_init();

	Vofa_Init();
	DJI_DR16_Init();

	BSP_USART_Init();
	BSP_TIM_Init();
	Robot_Cmd_Init();
	Chassis_Init();

	RefereeInit();
	UITaskInit();
	MyUIInit();

	Super_Cap_Set_Power(&Super_Cap, 4500); // 开启超电45w
	// Gimbal_Init();
	// Shoot_Init();
}

void App_Run(void)
{

	//	if (DR16_Get_Count() == 0)
	//	{
	//		return;
	//	}

	if (CAN_Comm_Get_State(&Down_Board) == 0)
	{
		return;
	}
	Test++;

	Vofa_Receive_Handle();

	//	Vofa_print(Down_Board_RX_Data);
	// CH104_IMU_USART_Handle();
	Robot_Cmd_Task();
	Chassis_Task();
	// HAL_Delay(1000);
	// 	Super_Cap_Set_Power(&Super_Cap,10000);
	// Gimbal_Task();
	// Shoot_Task();
		UI_Task();
	// DJI_Motor_Handle();
	// 	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);
	//	HAL_Delay(10);
}
