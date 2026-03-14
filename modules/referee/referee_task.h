/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2025-02-26 20:45:46
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-02 16:47:36
 * @FilePath: \RM_Hero_UP_Board\modules\referee\referee_task.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _REFEREE_H__
#define _REFEREE_H__

#include "rm_referee.h"
// #include "robot_def.h"
#include "RM_typedef.h"

typedef struct __UI_Task_Struct
{
  uint8_t Flag;
  uint16_t Count;


}UI_Task_Struct;


/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
// referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);
void UITaskInit(void);
/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */

void MyUIInit(void);


//UI_Task周期计时函数 应放1ms中断里
void UI_Task_Timing(void);

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */

void UI_Task(void);

#endif // REFEREE_H
