#ifndef _DJI_VT13_H__
#define _DJI_VT13_H__

#include "RM_typedef.h"
#include "usart.h"

#define DJI_VT13_DATA_LENGHT 21


#pragma pack(1)

typedef  struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2; // C0 H1 S2
    uint64_t stop:1;
    uint64_t fn_l:1;
    uint64_t fn_r:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;

    union 
    {
        uint16_t Data;
        struct  keyboard_Struct
        {
            uint8_t W : 1;
            uint8_t S : 1;
            uint8_t A : 1;
            uint8_t D : 1;
            uint8_t Shift : 1;
            uint8_t Ctrl : 1;
            uint8_t Q : 1;
            uint8_t E : 1;
            uint8_t R : 1;
            uint8_t F : 1;
            uint8_t G : 1;
            uint8_t Z : 1;
            uint8_t X : 1;
            uint8_t V : 1;
            uint8_t B : 1;
        } key;
        
    } key_board;
    
    uint16_t crc16;
}remote_data_t;

typedef struct
{
    uint8_t Count;
    remote_data_t Remote;
    float CH0;
    float CH1;
    float CH2;
    float CH3;
    float Wheel;
}DJI_VT13_Struct;
#pragma pack()

extern DJI_VT13_Struct DJI_VT13_Data;

void DJI_VT13_Init(void);
uint8_t *DJI_VT13_Get_Buffer(void);
void VT13_RX_Handle(uint8_t *Buffer, uint8_t size);
void VT13_Timing_Handle(void);
uint8_t VT13_Get_Count(void);





#endif
