/**
 * @file    MyI2C.h
 * @brief   软件I2C驱动模块头文件
 * @note    使用GPIO模拟I2C时序，支持主模式下的读写操作
 *         引脚定义：SCL - PB10, SDA - PB11
 */

#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h"

/*============================================================================
 * 引脚配置宏定义
 *============================================================================*/

/**
 * @defgroup MyI2C_PinConfig I2C引脚配置
 * @{
 */

/**
 * @brief   I2C使用的GPIO端口时钟使能宏
 */
#define RCC_GPIOx       RCC_APB2Periph_GPIOB

/**
 * @brief   I2C时钟线引脚（SCL）
 * @note    连接到PB10
 */
#define SCL_Pin         GPIO_Pin_10

/**
 * @brief   I2C数据线引脚（SDA）
 * @note    连接到PB11
 */
#define SDA_Pin         GPIO_Pin_11

/**
 * @brief   I2C使用的GPIO端口
 */
#define GPIOPort        GPIOB

/** @} */

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup MyI2C_API 软件I2C API函数
 * @{
 */

/**
 * @brief   I2C引脚初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    配置SCL和SDA引脚为开漏输出模式
 *         初始状态：SCL和SDA均为高电平（总线空闲）
 * 
 * @par     使用示例：
 * @code
 *   // 在使用I2C前调用
 *   MyI2C_Init();
 * @endcode
 */
void MyI2C_Init(void);

/**
 * @brief   写入SCL引脚电平
 * @param   Bitval   要写入的电平值（0或1）
 * @retval  无
 * @note    静态函数，仅供本模块内部调用
 */
static void MyI2C_W_SCL(uint8_t Bitval);

/**
 * @brief   写入SDA引脚电平
 * @param   Bitval   要写入的电平值（0或1）
 * @retval  无
 * @note    静态函数，仅供本模块内部调用
 */
static void MyI2C_W_SDA(uint8_t Bitval);

/**
 * @brief   读取SCL引脚电平
 * @param   无
 * @retval  SCL引脚电平（0或1）
 * @note    静态函数，仅供本模块内部调用
 */
static uint8_t MyI2C_R_SCL(void);

/**
 * @brief   读取SDA引脚电平
 * @param   无
 * @retval  SDA引脚电平（0或1）
 * @note    静态函数，仅供本模块内部调用
 */
static uint8_t MyI2C_R_SDA(void);

/**
 * @brief   I2C起始条件
 * @param   无
 * @retval  无
 * 
 * @note    起始条件时序：
 *          - SDA先由高变低（SCL为高）
 *          - 然后SCL拉低
 */
void MyI2C_Start(void);

/**
 * @brief   I2C停止条件
 * @param   无
 * @retval  无
 * 
 * @note    停止条件时序：
 *          - SCL先拉高
 *          - 然后SDA由低变高
 */
void MyI2C_Stop(void);

/**
 * @brief   I2C发送一个字节
 * @param   Byte   要发送的字节数据（0x00~0xFF）
 * @retval  无
 * 
 * @note    时序说明：
 *          - 高位先行（MSB First）
 *          - SCL低电平时改变SDA数据
 *          - SCL高电平时从机读取SDA
 */
void MyI2C_SendByte(uint8_t Byte);

/**
 * @brief   I2C接收一个字节
 * @param   无
 * @retval  接收到的字节数据（0x00~0xFF）
 * 
 * @note    时序说明：
 *          - 高位先行（MSB First）
 *          - SCL高电平时读取SDA数据
 *          - 接收完成后释放SDA总线
 */
uint8_t MyI2C_ReceiveByte(void);

/**
 * @brief   I2C接收应答信号
 * @param   无
 * @retval  应答值（0：应答，1：非应答）
 * 
 * @note    主机在第9个时钟周期释放SDA
 *         从机在此时拉低SDA表示应答（ACK=0）
 *         保持高电平表示非应答（NACK=1）
 */
uint8_t MyI2C_ReceiveAck(void);

/**
 * @brief   I2C发送应答信号
 * @param   Ack   应答值（0：应答，1：非应答）
 * @retval  无
 * 
 * @note    主机在第9个时钟周期控制SDA
 *         拉低SDA表示应答（ACK），保持高电平表示非应答（NACK）
 */
void MyI2C_SendAck(uint8_t Ack);

/** @} */

#endif /* __MYI2C_H */
