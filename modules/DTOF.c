#include "DTOF.h"
#include "i2c.h"
DTOF_Struct DTOF = {0};

void DTOF_Write_Reg(DTOF_REG_enum reg, uint8_t data)
{
    DTOF.TX_Buffer[0] = data;
    if (HAL_I2C_Mem_Write_DMA(&hi2c2, DTOF_IIC_WRITE_ADDRESS, reg, 1, DTOF.TX_Buffer, 1) != HAL_OK)
    {
        // Error_Handler();
    }
    //    if (HAL_I2C_Mem_Write(&hi2c2, DTOF_IIC_WRITE_ADDRESS, reg, 1, DTOF.TX_Buffer, 1,100) != HAL_OK)
    //    {
    //        Error_Handler();
    //    }
}

void DTOF_Read_Reg(DTOF_REG_enum reg, uint8_t size_t)
{
    if (HAL_I2C_Mem_Read_DMA(&hi2c2, DTOF_IIC_WRITE_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, DTOF.RX_Buffer, size_t) != HAL_OK)
    {
        // Error_Handler();
    }
    //    if (HAL_I2C_Mem_Read(&hi2c2, DTOF_IIC_WRITE_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, DTOF.RX_Buffer, size_t,100) != HAL_OK)
    //    {
    //        Error_Handler();
    //    }
}

void DTOF_Data_Process(void)
{
    DTOF.Distance = DTOF.RX_Buffer[0];
    DTOF.Distance = ((DTOF.Distance) << 8) + DTOF.RX_Buffer[1];
}

void DTOF_Enable(void)
{
    DTOF_Write_Reg(DTOF_ENABLE_FLAG, 1);
    DTOF.Enable_Flag = 1;
}

void DTOF_Disable(void)
{
    DTOF_Write_Reg(DTOF_ENABLE_FLAG, 0);
    DTOF.Enable_Flag = 0;
}
void DTOF_IIC_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
        Error_Handler();
    }
    //    DTOF_Enable();
    //     DTOF_Read_Reg(DTOF_DISTANCE_8BIT_H, 2);
}
void DTOF_Daemon_Init(void)
{
    DTOF.Daemon = Daemon_Register(DAE_DTOF, 10, NULL, ENABLE);
}

void DTOF_Init(void)
{
    DTOF_Daemon_Init();
    DTOF_Enable();
    DTOF_Read_Reg(DTOF_DISTANCE_8BIT_H, 2);
    DTOF_Data_Process();
}

/// @brief //放IIC中断回调函数，当接收到IIC数据后调用
/// @param
void DTOF_IIC_RX_Handle(void)
{
    DTOF_Read_Reg(DTOF_DISTANCE_8BIT_H, 2);
    DTOF_Data_Process();
    Daemon_Reload(DTOF.Daemon);
}

void DTOF_Test_Handle(void)
{
    //	DTOF_Init();
    DTOF_Read_Reg(DTOF_DISTANCE_8BIT_H, 2);
    DTOF_Data_Process();
}

/// @brief 放1ms定时器中断里
/// @param
void DTOF_IIC_Timing_Handle(void)
{
    if (DTOF.Count > 0)
    {
        DTOF.Count = DTOF.Count - 1;
    }
}

/// @brief DTOF IIC自动复位程序 放app循环里
/// @param
void DTOF_IIC_Handle(void)
{
    if (DTOF.Count > 0)
    {
        return;
    }
    switch (DTOF.Flag)
    {

    case 0:
        DTOF.Flag = 1;
        break;

    case 1:

        if ((DTOF.Daemon->Online_Flag == 0) && (DTOF.Daemon->Offline_Event == 0))
        {
            DTOF_Disable();
            DTOF.Flag = 2;
            DTOF.Count = 300;
        }
        else if ((DTOF.Distance >= 65535)&&(DTOF.Daemon->Online_Flag == 1))
        {
            // DTOF_Disable();
            DTOF.Flag = 4;
            DTOF.Count = 300;
        }
        else
        {
            DTOF.Count = 100;
        }
        break;

    case 2:

        if ((DTOF.Daemon->Online_Flag == 0) && (DTOF.Daemon->Offline_Event == 0))
        {

            DTOF.Flag = 3;
            DTOF.Count = 10;
        }
        else
        {
            DTOF.Flag = 1;
            DTOF.Count = 100;
        }

        break;

    case 3:
        DTOF_IIC_Init();
        DTOF.Flag = 4;
        DTOF.Count = 300;
        break;
    case 4:
        DTOF_Enable();
        DTOF.Flag = 5;
        DTOF.Count = 300;
        break;

    case 5:
        DTOF_Read_Reg(DTOF_DISTANCE_8BIT_H, 2);
        DTOF.Flag = 1;
        DTOF.Count = 300;
        break;
    default:
        break;
    }
}

// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
// {
//     if (hi2c == &hi2c2)
//     {
//         DTOF_Read_Reg(DTOF_DISTANCE_8BIT_H, 2);
//         DTOF_Data_Process();
//     }
// }
