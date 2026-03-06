/*
 * @Author: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @Date: 2025-02-28 11:07:58
 * @LastEditors: liciqikuanren 104132901+liciqikuanren@users.noreply.github.com
 * @LastEditTime: 2025-03-03 16:22:00
 * @FilePath: \RM_Hero_UP_Board\modules\daemon.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "daemon.h"
Daemon_Control_Struct Daemon_Control = {0, 0};
Daemon_Struct Daemon[DAEMON_SUM] = {0};

void Daemon_Config(Daemon_Struct *Instance, uint16_t Overload_Count, void (*Offline_CallBack)(void), Fn_State_t Enable)
{
    if (Instance != NULL)
    {
        Instance->Overload_Count = Overload_Count;
        Instance->Enable = Enable;
        Instance->Offline_CallBack = Offline_CallBack;
    }
}

Daemon_Struct *Daemon_Register(Daemon_ID_enum Daemon_ID, uint16_t Overload_Count, void (*Offline_CallBack)(void), Fn_State_t Enable)
{
//	Daemon[Daemon_ID].Online_Flag=1;
    Daemon_Config(&Daemon[Daemon_ID], Overload_Count, Offline_CallBack, Enable);
    return &(Daemon[Daemon_ID]);
}

void Daemon_Set_Overload_Count(Daemon_Struct *Instance, uint16_t Overload_Count)
{
    Instance->Overload_Count = Overload_Count;
}

/// @brief 设置掉线回调函数
/// @param Instance 掉线检测实例
/// @param Offline_CallBack   回调函数指针
void Daemon_Set_Offline_CallBack(Daemon_Struct *Instance, void (*Offline_CallBack)(void))
{
    Instance->Offline_CallBack = Offline_CallBack;
}

void Daemon_Set_Enable(Daemon_Struct *Instance, Fn_State_t Enable)
{
    Instance->Enable = Enable;
}

uint8_t Daemon_Get_Online(Daemon_Struct *Instance)
{
    return Instance->Online_Flag;
}

// 暂时不用这个初始化
void Daemon_Init(void)
{
}

/**
 * @brief Daemon回调处理函数（放入APP循环里）
 *
 * 该函数用于处理守护进程的回调事件。当Daemon_Control.Count大于0时，函数直接返回。
 * 否则，将Daemon_Control.Count设置为1（表示1ms周期）。
 *
 * 函数遍历所有守护进程（DAEMON_SUM），如果某个守护进程的Offline_CallBack不为空且Offline_Event为1，
 * 则将Offline_Event置为0，并调用Offline_CallBack函数。
 *
 * @note 该函数假设Daemon_Control和Daemon数组已经在其他地方定义和初始化。
 */
void Daemon_Callback_Handle(void)
{
    uint16_t i;

    switch (Daemon_Control.Flag)
    {

    case 0:
        Daemon_Control.Flag = 1;
        Daemon_Control.Count = 100;
        break;

    case 1:
        if (Daemon_Control.Count > 0)
        {
            return;
        }
        Daemon_Control.Count = 1; // 1ms周期

        for (i = 0; i < DAEMON_SUM; i++)
        {
            if (Daemon[i].Enable == DISABLE)
            {
                continue;
            }

            if (Daemon[i].Offline_Event == 1)
            {
                Daemon[i].Offline_Event = 0; // 重置掉线事件不管回调函数有没有注册只要掉线事件置1就重置
                if (Daemon[i].Offline_CallBack != NULL)
                {
                    Daemon[i].Offline_CallBack(); // 当掉线回调事件置1时，调用回调函数
                }
            }
        }
        break;

    default:
        break;
    }
}

/// @brief 放到1ms中断里
/// @param
void Daemon_Timing_Handle(void)
{
    uint16_t i;
    for (i = 0; i < DAEMON_SUM; i++)
    {
        if (Daemon[i].Enable == DISABLE)
        {
            continue;
        }

        if (Daemon[i].Count > 0)
        {
            Daemon[i].Count--;
        }

        if ((Daemon[i].Count == 0) && (Daemon[i].Online_Flag == 1))
        {
            Daemon[i].Online_Flag = 0;
            Daemon[i].Offline_Event = 1;
        }
    }

    if (Daemon_Control.Count > 0)
    {
        Daemon_Control.Count--;
    }
}

/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载temp_count,相当于"喂狗"
 *
 * @param instance daemon实例指针
 */
void Daemon_Reload(Daemon_Struct *Instance)
{
    if (Instance != NULL)
    {
        Instance->Count = Instance->Overload_Count;
        Instance->Online_Flag = 1;
    }
}
