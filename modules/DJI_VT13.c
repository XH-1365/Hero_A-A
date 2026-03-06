#include "DJI_VT13.h"

static uint16_t crc16_init = 0xffff;

static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
uint8_t verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return 0;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}


//////////////////////////////////////////////////////////

uint8_t DJI_VT13_Buffer[DJI_VT13_DATA_LENGHT];
DJI_VT13_Struct DJI_VT13_Data={0};

void DJI_VT13_Init()
{
     HAL_UARTEx_ReceiveToIdle_DMA(&huart7, DJI_VT13_Buffer, sizeof(DJI_VT13_Buffer)); // 串口空闲中断接收DMA
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);                                // 关闭DMA接收一半中断
}

uint8_t *DJI_VT13_Get_Buffer(void)
{
    return DJI_VT13_Buffer;
}

void DJI_VT13_Data_Process(uint8_t *Buffer, uint8_t size)
{
    if(Buffer==NULL)
    {
        return;
    }
    if(size!=DJI_VT13_DATA_LENGHT)
    {
        return;        
    }
    if(verify_crc16_check_sum(Buffer,size)==0)
    {
        return;
    }
    if(Buffer[0]==0xA9 && Buffer[1]==0x53)
    {
         uint64_t bits = (uint64_t)Buffer[2]         |
                         ((uint64_t)Buffer[3] << 8)  |
                         ((uint64_t)Buffer[4] << 16) |
                         ((uint64_t)Buffer[5] << 24) |
                         ((uint64_t)Buffer[6] << 32) |
                         ((uint64_t)Buffer[7] << 40) |
                         ((uint64_t)Buffer[8] << 48) |
                         ((uint64_t)Buffer[9] << 56);

        DJI_VT13_Data.Remote.ch_0     = (bits >> 0)  & 0x07FF;
        DJI_VT13_Data.Remote.ch_1     = (bits >> 11) & 0x07FF;
        DJI_VT13_Data.Remote.ch_2     = (bits >> 22) & 0x07FF;
        DJI_VT13_Data.Remote.ch_3     = (bits >> 33) & 0x07FF;
        DJI_VT13_Data.Remote.mode_sw  = (bits >> 44) & 0x03;
        DJI_VT13_Data.Remote.stop     = (bits >> 46) & 0x01;
        DJI_VT13_Data.Remote.fn_l     = (bits >> 47) & 0x01;
        DJI_VT13_Data.Remote.fn_r     = (bits >> 48) & 0x01;
        DJI_VT13_Data.Remote.wheel    = (bits >> 49) & 0x07FF;
        DJI_VT13_Data.Remote.trigger  = (bits >> 60) & 0x01;
        
        DJI_VT13_Data.Remote.mouse_x = (int16_t)(Buffer[10] | (Buffer[11] << 8));
        DJI_VT13_Data.Remote.mouse_y = (int16_t)(Buffer[12] | (Buffer[13] << 8));
        DJI_VT13_Data.Remote.mouse_z = (int16_t)(Buffer[14] | (Buffer[15] << 8));
        
        uint8_t mouse_byte = Buffer[16];
        DJI_VT13_Data.Remote.mouse_left   = (mouse_byte >> 0) & 0x03;
        DJI_VT13_Data.Remote.mouse_right  = (mouse_byte >> 2) & 0x03;
        DJI_VT13_Data.Remote.mouse_middle = (mouse_byte >> 4) & 0x03;

        DJI_VT13_Data.Remote.key_board.Data = (uint16_t)(Buffer[17] | (Buffer[18] << 8));
        
        DJI_VT13_Data.CH0= NORMALIZE(DJI_VT13_Data.Remote.ch_0,364,1684);
        DJI_VT13_Data.CH1= NORMALIZE(DJI_VT13_Data.Remote.ch_1,364,1684);
        DJI_VT13_Data.CH2= NORMALIZE(DJI_VT13_Data.Remote.ch_2,364,1684);
        DJI_VT13_Data.CH3= NORMALIZE(DJI_VT13_Data.Remote.ch_3,364,1684);
        DJI_VT13_Data.Wheel=NORMALIZE(DJI_VT13_Data.Remote.wheel,364,1684);
    }
}

void VT13_RX_Handle(uint8_t *Buffer, uint8_t size)
{
    DJI_VT13_Data_Process(Buffer,size);
    DJI_VT13_Init();
    DJI_VT13_Data.Count=50;
}
void VT13_Timing_Handle(void)
{
    if(DJI_VT13_Data.Count> 0)
    {
        DJI_VT13_Data.Count--;
    }
}
uint8_t VT13_Get_Count(void)
{
    return DJI_VT13_Data.Count;
}
