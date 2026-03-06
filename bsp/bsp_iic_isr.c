#include "bsp_iic.h"
#include "DTOF.h"
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2)
    {
        DTOF_IIC_RX_Handle();
        // DTOF_Read_Reg(DTOF_DISTANCE_8BIT_H, 2);
        // DTOF_Data_Process();
    }
}
