#ifndef _CH104_IMU_USART_H__
#define _CH104_IMU_USART_H__
#include "RM_typedef.h"
#define CH104_IMU_USART_BUFFER_LENGTH 82
typedef enum __CH104_IMU_USART_State_enum
{
    CH104_IMU_USART_DICONNECT = 0, // 断连状态
    CH104_IMU_USART_CONNECT,       // 连接状态
} CH104_IMU_USART_State_enum;

#pragma pack(1) // 指定结构体按照1字节对齐
/// @brief 三轴加速度，单位为:G
typedef struct __CH104_IMU_USART_Acceleration_Data_Struct
{
    float Acc_X; // 加速度X轴
    float Acc_Y; // 加速度Y轴
    float Acc_Z; // 加速度Z轴

} CH104_IMU_USART_Acceleration_Data_Struct;

/// @brief 三轴角速度，单位为deg/s(°/s)
typedef struct __CH104_IMU_USART_Angular_Velocity_Data_Struct
{
    float Gyro_X; // 角速度X轴
    float Gyro_Y; // 角速度Y轴
    float Gyro_Z; // 角速度Z轴

} CH104_IMU_USART_Angular_Velocity_Data_Struct;

/// @brief 三轴磁场强度，单位为:uT
typedef struct __CH104_IMU_USART_Magnetic_Intensity_Data_Struct
{
    float MagInt_X; // 磁场强度X轴
    float MagInt_Y; // 磁场强度Y轴
    float MagInt_Z; // 磁场强度Z轴

} CH104_IMU_USART_Magnetic_Intensity_Data_Struct;

/// @brief 欧拉角，单位为:deg(°)
typedef struct __CH104_IMU_USART_Euler_Angles_Data_Struct
{
    float Roll;  // 横滚角
    float Pitch; // 俯仰角
    float Yaw;   // 航向角

} CH104_IMU_USART_Euler_Angles_Data_Struct;

/*
@brief 四元数

*/
typedef struct __CH104_IMU_USART_Quaternion_Data_Struct
{
    float Quat_W; //  实部（Scalar part）
    float Quat_X; // 第一虚部（First imaginary part）
    float Quat_Y; // 第二虚部（Second imaginary part）
    float Quat_Z; // 第三虚部（Third imaginary part）

} CH104_IMU_USART_Quaternion_Data_Struct;

typedef union __CH104_IMU_USART_Data_Union
{
    struct
    {
        uint16_t Head;
        uint16_t Length;
        uint16_t CRC_Value;
        uint8_t Tag;
        uint8_t ID;
        uint8_t Reserve;
        int8_t Temperature;
        float Pressure;
        uint32_t Time;                                                     // 时间戳单位：ms
        CH104_IMU_USART_Acceleration_Data_Struct Acceleration;             // 三轴加速度
        CH104_IMU_USART_Angular_Velocity_Data_Struct Angular_Velocity;     // 三轴角速度
        CH104_IMU_USART_Magnetic_Intensity_Data_Struct Magnetic_Intensity; // 三轴磁场强度
        CH104_IMU_USART_Euler_Angles_Data_Struct Euler_Angles;             // 欧拉角，单位为0.01°
        CH104_IMU_USART_Quaternion_Data_Struct Quaternion;
    } Data;
    uint8_t Buffer[CH104_IMU_USART_BUFFER_LENGTH];
} CH104_IMU_USART_Data_Union;

typedef struct __CH104_IMU_USART_Struct
{
    uint8_t Flag;
    uint16_t Count;

    uint16_t RX_Count;
    CH104_IMU_USART_State_enum State;

    CH104_IMU_USART_Data_Union Ret_Data;
} CH104_IMU_USART_Struct;
#pragma pack() // 取消结构体对齐

void CH104_IMU_USART_Init(void);
void CH104_IMU_USART_Timing_Handle(void);
uint8_t *CH104_IMU_USART_Get_Buffer(void);
void CH104_IMU_USART_RX_Handle(uint8_t *Buffer, uint8_t size);
void CH104_IMU_USART_Handle(void);


#endif
