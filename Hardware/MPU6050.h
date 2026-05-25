/**
 * @file    MPU6050.h
 * @brief   MPU6050六轴传感器驱动模块头文件
 * @note    提供加速度计和陀螺仪数据的读取接口
 *         使用I2C通信协议，从机地址为0xD0
 */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup MPU6050_API MPU6050 API函数
 * @{
 */

/**
 * @brief   MPU6050写寄存器
 * @param   RegAddress   寄存器地址（参考MPU6050手册）
 * @param   Data         要写入寄存器的数据（0x00~0xFF）
 * @retval  无
 * 
 * @par     使用示例：
 * @code
 *   // 设置电源管理寄存器1，取消睡眠模式
 *   MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
 * @endcode
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);

/**
 * @brief   MPU6050读寄存器
 * @param   RegAddress   寄存器地址（参考MPU6050手册）
 * @retval  读取到的寄存器数据（0x00~0xFF）
 * 
 * @par     使用示例：
 * @code
 *   uint8_t whoami = MPU6050_ReadReg(MPU6050_WHO_AM_I);
 * @endcode
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

/**
 * @brief   MPU6050初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    使用前必须调用此函数初始化MPU6050
 *         配置内容包括：
 *         - 取消睡眠模式，时钟源选择X轴陀螺仪
 *         - 设置采样率
 *         - 陀螺仪满量程：±2000°/s
 *         - 加速度计满量程：±16g
 * 
 * @par     使用示例：
 * @code
 *   // 在系统初始化时调用
 *   MPU6050_Init();
 * @endcode
 */
void MPU6050_Init(void);

/**
 * @brief   获取MPU6050的ID号
 * @param   无
 * @retval  MPU6050的WHO_AM_I寄存器值（通常为0x68）
 * 
 * @note    可用于验证MPU6050是否正常工作
 * 
 * @par     使用示例：
 * @code
 *   uint8_t id = MPU6050_GetID();
 *   if (id == 0x68) {
 *       // MPU6050正常工作
 *   }
 * @endcode
 */
uint8_t MPU6050_GetID(void);

/**
 * @brief   获取MPU6050传感器数据
 * @param   AccX    加速度计X轴数据输出指针（范围：-32768~32767）
 * @param   AccY    加速度计Y轴数据输出指针（范围：-32768~32767）
 * @param   AccZ    加速度计Z轴数据输出指针（范围：-32768~32767）
 * @param   GyroX   陀螺仪X轴数据输出指针（范围：-32768~32767）
 * @param   GyroY   陀螺仪Y轴数据输出指针（范围：-32768~32767）
 * @param   GyroZ   陀螺仪Z轴数据输出指针（范围：-32768~32767）
 * @retval  无
 * 
 * @note    原始数据需要根据满量程进行换算：
 *         - 加速度：满量程±16g，灵敏度16384 LSB/g
 *           实际加速度(g) = 原始值 / 16384
 *         - 陀螺仪：满量程±2000°/s，灵敏度16.4 LSB/(°/s)
 *           实际角速度(°/s) = 原始值 / 16.4
 * 
 * @par     使用示例：
 * @code
 *   int16_t ax, ay, az, gx, gy, gz;
 *   MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
 *   
 *   // 转换为实际物理量
 *   float acc_x = ax / 16384.0f;      // 单位：g
 *   float gyro_z = gz / 16.4f;        // 单位：°/s
 * @endcode
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/** @} */

#endif /* __MPU6050_H */
