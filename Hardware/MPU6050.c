/**
 * @file    MPU6050.c
 * @brief   MPU6050六轴传感器驱动模块实现
 * @note    提供加速度计和陀螺仪数据的读取功能
 *         使用软件I2C进行通信
 */

#include "stm32f10x.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"

/*============================================================================
 * 常量定义
 *============================================================================*/

/**
 * @brief   MPU6050的I2C从机地址
 * @note    0xD0 = 0x68 << 1（7位地址0x68左移1位）
 *         最低位为读写位：0表示写，1表示读
 */
#define MPU6050_ADDRESS     0xD0

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   MPU6050写寄存器
 * @param   RegAddress   寄存器地址（参考MPU6050手册）
 * @param   Data         要写入寄存器的数据（0x00~0xFF）
 * @retval  无
 * 
 * @note    I2C通信时序：
 *          - 起始条件
 *          - 发送从机地址（写操作）
 *          - 发送寄存器地址
 *          - 发送要写入的数据
 *          - 停止条件
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();                          /* I2C起始条件 */
    MyI2C_SendByte(MPU6050_ADDRESS);        /* 发送从机地址，读写位为0（写操作） */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    MyI2C_SendByte(RegAddress);             /* 发送寄存器地址 */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    MyI2C_SendByte(Data);                   /* 发送要写入寄存器的数据 */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    MyI2C_Stop();                           /* I2C停止条件 */
}

/**
 * @brief   MPU6050读寄存器
 * @param   RegAddress   寄存器地址（参考MPU6050手册）
 * @retval  读取到的寄存器数据（0x00~0xFF）
 * 
 * @note    I2C通信时序：
 *          - 起始条件
 *          - 发送从机地址（写操作）
 *          - 发送寄存器地址
 *          - 重复起始条件
 *          - 发送从机地址（读操作）
 *          - 读取数据
 *          - 发送非应答信号
 *          - 停止条件
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    
    MyI2C_Start();                          /* I2C起始条件 */
    MyI2C_SendByte(MPU6050_ADDRESS);        /* 发送从机地址，读写位为0（写操作） */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    MyI2C_SendByte(RegAddress);             /* 发送寄存器地址 */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    
    MyI2C_Start();                          /* I2C重复起始条件 */
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01); /* 发送从机地址，读写位为1（读操作） */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    Data = MyI2C_ReceiveByte();             /* 读取指定寄存器的数据 */
    MyI2C_SendAck(1);                       /* 发送非应答，终止从机数据输出 */
    MyI2C_Stop();                           /* I2C停止条件 */
    
    return Data;
}

/**
 * @brief   MPU6050连续读多个寄存器
 * @param   RegAddress   起始寄存器地址
 * @param   DataArray    输出参数，用于存储多个寄存器值的数组
 * @param   Count        要读取的寄存器数量
 * @retval  无
 * 
 * @note    MPU6050支持地址自动递增，连续读取效率更高
 *         除最后一个字节外，都需要发送应答信号
 */
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    uint8_t i;
    
    MyI2C_Start();                          /* I2C起始条件 */
    MyI2C_SendByte(MPU6050_ADDRESS);        /* 发送从机地址，读写位为0（写操作） */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    MyI2C_SendByte(RegAddress);             /* 发送起始寄存器地址 */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    
    MyI2C_Start();                          /* I2C重复起始条件 */
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01); /* 发送从机地址，读写位为1（读操作） */
    MyI2C_ReceiveAck();                     /* 接收从机应答 */
    
    /* 循环读取Count个字节，MPU6050内部地址指针自动递增 */
    for (i = 0; i < Count; i++)
    {
        DataArray[i] = MyI2C_ReceiveByte(); /* 读取数据存入数组 */
        
        if (i < Count - 1)                  /* 未读取到最后一个字节 */
        {
            MyI2C_SendAck(0);               /* 发送应答，通知从机继续发送 */
        }
        else                                /* 读取到最后一个字节 */
        {
            MyI2C_SendAck(1);               /* 发送非应答，通知从机停止发送 */
        }
    }
    MyI2C_Stop();                           /* I2C停止条件 */
}

/**
 * @brief   MPU6050初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    寄存器配置说明：
 *          - PWR_MGMT_1 (0x6B):   取消睡眠，时钟源选择X轴陀螺仪
 *          - PWR_MGMT_2 (0x6C):   所有轴均不待机
 *          - SMPLRT_DIV (0x19):   采样率分频，输出速率 = 1kHz / (1 + 7) = 125Hz
 *          - CONFIG (0x1A):       不使能DLPF，带宽为8kHz
 *          - GYRO_CONFIG (0x1B):  满量程±2000°/s
 *          - ACCEL_CONFIG (0x1C): 满量程±16g
 */
void MPU6050_Init(void)
{
    MyI2C_Init();                               /* 先初始化底层的I2C */
    
    /* 电源管理寄存器1：取消睡眠模式，选择时钟源为X轴陀螺仪 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    
    /* 电源管理寄存器2：所有轴均不待机 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    
    /* 采样率分频寄存器：输出速率 = 1kHz / (1 + 7) = 125Hz */
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);
    
    /* 配置寄存器：禁用DLPF，带宽8kHz */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x00);
    
    /* 陀螺仪配置寄存器：满量程±2000°/s，灵敏度16.4 LSB/(°/s) */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    
    /* 加速度计配置寄存器：满量程±16g，灵敏度16384 LSB/g */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

/**
 * @brief   获取MPU6050的ID号
 * @param   无
 * @retval  MPU6050的WHO_AM_I寄存器值（正常应为0x68）
 * 
 * @note    可用于检测MPU6050是否正常工作
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief   获取MPU6050传感器数据
 * @param   AccX    加速度计X轴数据输出指针
 * @param   AccY    加速度计Y轴数据输出指针
 * @param   AccZ    加速度计Z轴数据输出指针
 * @param   GyroX   陀螺仪X轴数据输出指针
 * @param   GyroY   陀螺仪Y轴数据输出指针
 * @param   GyroZ   陀螺仪Z轴数据输出指针
 * @retval  无
 * 
 * @note    数据寄存器映射（连续14字节）：
 *          - [0-1]:  加速度计X轴 (ACCEL_XOUT_H/L)
 *          - [2-3]:  加速度计Y轴 (ACCEL_YOUT_H/L)
 *          - [4-5]:  加速度计Z轴 (ACCEL_ZOUT_H/L)
 *          - [6-7]:  温度数据 (TEMP_OUT_H/L)
 *          - [8-9]:  陀螺仪X轴 (GYRO_XOUT_H/L)
 *          - [10-11]: 陀螺仪Y轴 (GYRO_YOUT_H/L)
 *          - [12-13]: 陀螺仪Z轴 (GYRO_ZOUT_H/L)
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];
    
    /* 从ACCEL_XOUT_H寄存器开始连续读取14字节，效率更高 */
    MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);
    
    /* 拼接高8位和低8位，得到16位原始数据 */
    *AccX = (Data[0] << 8) | Data[1];   /* 加速度计X轴数据 */
    *AccY = (Data[2] << 8) | Data[3];   /* 加速度计Y轴数据 */
    *AccZ = (Data[4] << 8) | Data[5];   /* 加速度计Z轴数据 */
    
    /* Data[6]和Data[7]为温度数据，此处暂不使用 */
    
    *GyroX = (Data[8] << 8) | Data[9];  /* 陀螺仪X轴数据 */
    *GyroY = (Data[10] << 8) | Data[11]; /* 陀螺仪Y轴数据 */
    *GyroZ = (Data[12] << 8) | Data[13]; /* 陀螺仪Z轴数据 */
}
