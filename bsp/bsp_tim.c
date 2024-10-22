#include "bsp_tim.h"

void BSP_TIM_Init(void) 
{
  HAL_TIM_Base_Start_IT(&htim7);
}
