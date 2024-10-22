/**
  ******************************************************************************
  * @file    pid.h
  * @brief    PID控制器。
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
	
#ifndef __PID_H
#define __PID_H
/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

typedef enum
{
	PID_DISABLE,
	PID_ENABLE
}PID_STATE;

typedef struct pid_controler
{
	float err;	//err值相关计算值
	float err_dz;//误差死区
	float err_limit;
	
	float d_err;
	float i_err;
	float i_err_limit;
	
	float kp;	//PID参数
	float ki;
	float kd;

	float output;			//输出相关
	float output_limit;		//输出限幅
	float p_output;
	float i_output;
	float d_output;
	
	PID_STATE startfalg;
	
}pid_controler;

//PID初始化
void PID_Init(pid_controler *PID);

//PID改参数
void PID_Set(pid_controler *PID, float kp, float ki, float kd, float i_err_limit, float err_dz, float output_limit);

//PID缓存清空
void PID_Clear(pid_controler *PID);

//输入参数，
//PID 模块结构体
//err 该轮PID调控中得到的err值
float pid_error_input(pid_controler *PID, float err);

#endif

