#ifndef MINI_PC_H
#define MINI_PC_H

#include "usart.h"
#include "bsp_usart.h"

#include "CH104_IMU_USART.H"

#define MINI_RX_SIZE 6 
#define MINI_TX_SIZE 11 

#define RX_TAIL  0X51
#define TX_TAIL  0X52
#define TX_LENTH 0X0B

#pragma pack(1)

typedef struct
{
    uint8_t Flag;   
    uint16_t Count; 
    float   yaw;
    float   pitch;
}Mini_pc_Struct;



#pragma pack()


extern Mini_pc_Struct Mini_pc_Data;
void MINI_PC_Init(void);

void MINI_PC_RX_Handle(uint8_t *RX_BUFF , uint8_t size);
uint8_t *MINI_PC_Get_Buffer(void);

void MINI_PC__Timing_Handle(void);


void MINI_PC_send(void);
float PC_Get_Yaw(void);
float PC_Get_Pitch(void);
#endif 
