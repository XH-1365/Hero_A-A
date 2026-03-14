#include "mini_pc.h"

uint8_t Mini_PC_RX_BUFF[MINI_RX_SIZE];
uint8_t Mini_PC_TX_BUFF[MINI_TX_SIZE]={TX_TAIL ,MINI_TX_SIZE};

Mini_pc_Struct Mini_pc_Data={0};

float mini_yaw;
float mini_pitch;
void Check_Sum(uint8_t *Mini_PC_TX_BUFF )
{
     uint8_t SUM=0;
    if(Mini_PC_TX_BUFF[0] == TX_TAIL && Mini_PC_TX_BUFF[1] == TX_LENTH)
    {
        for(int o=0; o<10 ;o++)
        {
            SUM +=Mini_PC_TX_BUFF[o];
        }
        Mini_PC_TX_BUFF[10]=SUM;
    }
}
void Data_Processing(uint8_t *Mini_PC_RX_BUFF , uint8_t size)
{
    uint8_t SUM=0;
    uint16_t yaw;
    uint16_t pitch;
    if(Mini_PC_RX_BUFF[0]==RX_TAIL && size == MINI_RX_SIZE)
    {
        for(int o=0;o<(size-1);o++)
        {
            SUM +=Mini_PC_RX_BUFF[o];
        }
        if( SUM == Mini_PC_RX_BUFF[5])
        {
            
            yaw  =(Mini_PC_RX_BUFF[2]<<8) + Mini_PC_RX_BUFF[1];
            pitch=(Mini_PC_RX_BUFF[4]<<8) + Mini_PC_RX_BUFF[3];
            mini_yaw  = (yaw-9000)/(-100.f);
            mini_pitch= (pitch-9000)/(-100.f);

            Mini_pc_Data.yaw  = mini_yaw/5000.f;
            Mini_pc_Data.pitch=mini_pitch/5000.f;
        }
    }
}



void MINI_PC_Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8,Mini_PC_RX_BUFF,MINI_RX_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx,DMA_IT_HT);
}

uint8_t *MINI_PC_Get_Buffer(void)
{
    return Mini_PC_RX_BUFF;
}
void MINI_PC_RX_Handle(uint8_t *Mini_PC_RX_BUFF , uint8_t size)
{
    Data_Processing(Mini_PC_RX_BUFF,size);
    MINI_PC_Init();
}




void MINI_PC_send()
{
    int16_t roll = (int16_t) ((-CH104_IMU_USART.Ret_Data.Data.Euler_Angles.Roll) *100);
    int16_t yaw  = (int16_t) ((-CH104_IMU_USART.Ret_Data.Data.Euler_Angles.Yaw) *100);
    int16_t pitch= (int16_t) ((-CH104_IMU_USART.Ret_Data.Data.Euler_Angles.Pitch) *100);
    

    Mini_PC_TX_BUFF[2]= (uint16_t)roll>>8;
    Mini_PC_TX_BUFF[3]= (uint16_t)roll&0x00ff;
    Mini_PC_TX_BUFF[4]= (uint16_t)yaw>>8;
    Mini_PC_TX_BUFF[5]= (uint16_t)yaw&0x00ff;
    Mini_PC_TX_BUFF[6]= (uint16_t)pitch>>8;
    Mini_PC_TX_BUFF[7]= (uint16_t)pitch&0x00ff;

    Mini_PC_TX_BUFF[8]= Mini_pc_Data.Count>8;
    Mini_PC_TX_BUFF[9]= Mini_pc_Data.Count&0x00ff;
    Check_Sum(Mini_PC_TX_BUFF);
    
    HAL_UART_Transmit_DMA(&huart8,Mini_PC_TX_BUFF , TX_LENTH);

}

void MINI_PC__Timing_Handle(void)
{
    Mini_pc_Data.Count++;
}
float PC_Get_Yaw()
{
    return Mini_pc_Data.yaw;
}
float PC_Get_Pitch()
{
    return Mini_pc_Data.pitch;
}
