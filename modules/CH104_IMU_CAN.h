#ifndef _CH104_IMU_CAN_H__
#define _CH104_IMU_CAN_H__
#include "RM_typedef.h"
#include "CH104_IMU_CAN_CMD.h"
#define NODE_ID 8
typedef enum __CH104_IMU_CAN_RX_ID_enum
{
    ACC_RX_ID = 0x180 + NODE_ID,          // 加速度传感器的接收 ID
    GYRO_RX_ID = 0x280 + NODE_ID,         // 陀螺仪的接收 ID
    EULER_ANGLES_RX_ID = 0x380 + NODE_ID, // 欧拉角的接收 ID
    QUAT_RX_ID = 0x480 + NODE_ID,         // 四元数的接收 ID
    PRESSURE_RX_ID = 0x680 + NODE_ID,     // 气压传感器的接收 ID
    TILT_SENSOR_RX_ID = 0x780 + NODE_ID   // 倾角传感器的接收 ID
} CH104_IMU_CAN_RX_ID_enum;



#pragma pack(1) // 指定结构体按照1字节对齐
/// @brief 三轴加速度，单位为mG(0.001G)
typedef struct __CH104_IMU_CAN_Acceleration_Data_Struct
{
    int16_t Acc_X; // 加速度X轴
    int16_t Acc_Y; // 加速度Y轴
    int16_t Acc_Z; // 加速度Z轴

} CH104_IMU_CAN_Acceleration_Data_Struct;

typedef struct __CH104_IMU_CAN_Acceleration_Struct
{
    CH104_IMU_CAN_RX_ID_enum RX_STD_ID;
    CH104_IMU_CAN_Acceleration_Data_Struct Data;
} CH104_IMU_CAN_Acceleration_Struct;
/*----------------------------------------------------------*/

/// @brief 三轴角速度，单位为0.1dps(°/s)
typedef struct __CH104_IMU_CAN_Angular_Velocity_Data_Struct
{
    int16_t Gyro_X; // 角速度X轴
    int16_t Gyro_Y; // 角速度Y轴
    int16_t Gyro_Z; // 角速度Z轴

} CH104_IMU_CAN_Angular_Velocity_Data_Struct;

typedef struct __CH104_IMU_CAN_Angular_Velocity_Struct
{
    CH104_IMU_CAN_RX_ID_enum RX_STD_ID;
    CH104_IMU_CAN_Angular_Velocity_Data_Struct Data;
} CH104_IMU_CAN_Angular_Velocity_Struct;
/*----------------------------------------------------------*/

/// @brief 欧拉角，单位为0.01°
typedef struct __CH104_IMU_CAN_Euler_Angles_Data_Struct
{
    int16_t Roll;  // 横滚角
    int16_t Pitch; // 俯仰角
    int16_t Yaw;   // 航向角

} CH104_IMU_CAN_Euler_Angles_Data_Struct;

typedef struct __CH104_IMU_CAN_Euler_Angles_Struct
{
    CH104_IMU_CAN_RX_ID_enum RX_STD_ID;
    CH104_IMU_CAN_Euler_Angles_Data_Struct Data;
} CH104_IMU_CAN_Euler_Angles_Struct;
/*----------------------------------------------------------*/

/*
@brief 四元数，单位为四元数扩大10000倍后结果。
如四元数为1,0,0,0 时, 输出10000,0,0,0。
*/
typedef struct __CH104_IMU_CAN_Quaternion_Data_Struct
{
    int16_t Quat_W; //  实部（Scalar part）
    int16_t Quat_X; // 第一虚部（First imaginary part）
    int16_t Quat_Y; // 第二虚部（Second imaginary part）
    int16_t Quat_Z; // 第三虚部（Third imaginary part）

} CH104_IMU_CAN_Quaternion_Data_Struct;

typedef struct __CH104_IMU_CAN_Quaternion_Struct
{
    CH104_IMU_CAN_RX_ID_enum RX_STD_ID;
    CH104_IMU_CAN_Quaternion_Data_Struct Data;
} CH104_IMU_CAN_Quaternion_Struct;
/*----------------------------------------------------------*/
/// @brief 大气压，单位为Pa
typedef struct __CH104_IMU_CAN_Pressure_Data_Struct
{
    int32_t Pressure_Value;

} CH104_IMU_CAN_Pressure_Data_Struct;

/// @brief 大气压，单位为Pa
typedef struct __CH104_IMU_CAN_Pressure_Struct
{
    CH104_IMU_CAN_RX_ID_enum RX_STD_ID;
    CH104_IMU_CAN_Pressure_Data_Struct Data; // 大气压值
} CH104_IMU_CAN_Pressure_Struct;
/*----------------------------------------------------------*/

/// @brief 倾角仪，单位为0.01°
typedef struct __CH104_IMU_CAN_Tilt_Sensor_Data_Struct
{
    int32_t Angle_X; // X轴角度值
    int32_t Angle_Y; // Y轴角度值
} CH104_IMU_CAN_Tilt_Sensor_Data_Struct;

typedef struct __CH104_IMU_CAN_Tilt_Sensor_Struct
{
    CH104_IMU_CAN_RX_ID_enum RX_STD_ID;
    CH104_IMU_CAN_Tilt_Sensor_Data_Struct Data;
} CH104_IMU_CAN_Tilt_Sensor_Struct;
/*----------------------------------------------------------*/

typedef struct __CH104_IMU_CAN_Struct
{
    CAN_HandleTypeDef *hcan; // 使用的CAN口，hcan1或者hcan2
    CH104_IMU_CAN_Acceleration_Struct Acceleration;
    CH104_IMU_CAN_Angular_Velocity_Struct Angular_Velocity;
    CH104_IMU_CAN_Euler_Angles_Struct Euler_Angles;
    CH104_IMU_CAN_Quaternion_Struct Quaternion;
    CH104_IMU_CAN_Pressure_Struct Pressure;
    CH104_IMU_CAN_Tilt_Sensor_Struct Tilt_Sensor;
} CH104_IMU_CAN_Struct;
/*----------------------------------------------------------*/
#pragma pack() // 取消结构体对齐

extern CH104_IMU_CAN_Struct CH104_IMU_CAN;

void CH104_IMU_CAN_Init(void);

char CH104_IMU_CAN_Get_Data(CH104_IMU_CAN_Struct *CH104_IMU_CAN, CAN_RxHeaderTypeDef *RxHeader, uint8_t *CH104_IMU_CAN_RX_Buffer);


void CH104_IMU_CAN_Set_Baudrate(CH104_IMU_CAN_Struct *CH104_IMU_CAN, uint32_t CH104_IMU_CAN_Baudrate);
void CH104_IMU_CAN_Set(void);
// void CH104_IMU_CAN_Save_Set(CH104_IMU_CAN_Struct *CH104_IMU_CAN);
#endif
