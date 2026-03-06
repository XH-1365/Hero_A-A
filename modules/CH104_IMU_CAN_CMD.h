/*
 * @Author: liciqikuanren 1072047735@qq.com
 * @Date: 2024-08-25 16:58:56
 * @LastEditors: liciqikuanren 1072047735@qq.com
 * @LastEditTime: 2024-08-26 17:00:07
 * @FilePath: \RM_Template\modules\CH104_IMU_CAN_CMD.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CH104_IMU_CAN_CMD_H__
#define _CH104_IMU_CAN_CMD_H__
#include "RM_typedef.h"

/// @brief 波特率挡位，单位：bit/s
typedef enum __CH104_IMU_CAN_Baudrate_enum
{
  CH104_IMU_CAN_Baudrate_1000K,
  CH104_IMU_CAN_Baudrate_800K,
  CH104_IMU_CAN_Baudrate_500K,
  CH104_IMU_CAN_Baudrate_250K,
  CH104_IMU_CAN_Baudrate_125K,
  CH104_IMU_CAN_Baudrate_100K,
  CH104_IMU_CAN_Baudrate_50K,
  CH104_IMU_CAN_Baudrate_20K,
  CH104_IMU_CAN_Baudrate_10K
} CH104_IMU_CAN_Baudrate_enum;
typedef enum __CH104_IMU_CAN_RX_Sensor_Speed_enum
{
  SENSOR_CLOSE = 0x00,
  SENSOR_200HZ = 0x05,
  SENSOR_100HZ = 0x0A,
  SENSOR_50HZ = 0X14,
  SENSOR_20HZ = 0X32,
  SENSOR_10HZ = 0X64

} CH104_IMU_CAN_RX_Sensor_Speed_enum;

typedef enum __CH104_IMU_CAN_RX_Sensor_Speed_Index_enum
{
  ACC_SPEED_INDEX = 0x1800,
  GYRO_SPEED_INDEX = 0x1801,
  EULER_ANGLES_SPEED_INDEX = 0x1802,
  QUAT_SPEED_INDEX = 0x1803,
  PRESSURE_SPEED_INDEX = 0x1804
} CH104_IMU_CAN_RX_Sensor_Speed_Index_enum;

#pragma pack(1) // 指定结构体按照1字节对齐
typedef struct __CH104_IMU_CAN_CMD_Struct
{
  uint8_t CMD;
  uint16_t Index;
  uint8_t Sub_Index;
  uint32_t Data;
} CH104_IMU_CAN_CMD_Struct;

typedef union __CH104_IMU_CAN_CMD_Union
{
  CH104_IMU_CAN_CMD_Struct CMD_Struct;
  uint8_t Array[8];
} CH104_IMU_CAN_CMD_Union;
#pragma pack() // 取消结构体对齐

#endif
