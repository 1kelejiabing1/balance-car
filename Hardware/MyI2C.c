/**
 * @file    MyI2C.c
 * @brief   软件I2C驱动模块实现
 * @note    使用GPIO模拟I2C时序，支持标准I2C通信协议
 *         通信速率取决于代码执行速度，约为数十kHz
 */

#include "MyI2C.h"

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   I2C引脚初始化函数
 * @param   无
 * @retval  无
 * 
 * @note    此函数完成以下配置：
 *          - 使能GPIOB时钟
 *          - 配置SCL和SDA引脚为开漏输出模式
 *          - 初始状态将SCL和SDA拉高（总线空闲）
 * 
 * @par     开漏输出的优点：
 *          - 可实现线与功能
 *          - 多设备共享总线时不会产生冲突
 *          - 方便切换输入/输出方向
 */
void MyI2C_Init(void)
{
    /* 使能GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    /* 配置SCL和SDA引脚为开漏输出模式 */
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;   /* 开漏输出模式 */
    GPIO_InitStruct.GPIO_Pin = SDA_Pin | SCL_Pin;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOPort, &GPIO_InitStruct);
    
    /* 总线空闲时，SCL和SDA均为高电平 */
    GPIO_SetBits(GPIOPort, SDA_Pin | SCL_Pin);
}

/**
 * @brief   写入SCL引脚电平
 * @param   Bitval   要写入的电平值（0或1）
 * @retval  无
 * @note    静态函数，仅供本模块内部调用
 */
static void MyI2C_W_SCL(uint8_t Bitval)
{
    GPIO_WriteBit(GPIOPort, SCL_Pin, (BitAction)Bitval);
}

/**
 * @brief   写入SDA引脚电平
 * @param   Bitval   要写入的电平值（0或1）
 * @retval  无
 * @note    静态函数，仅供本模块内部调用
 */
void MyI2C_W_SDA(uint8_t Bitval)
{
    GPIO_WriteBit(GPIOPort, SDA_Pin, (BitAction)Bitval);
}

/**
 * @brief   读取SCL引脚电平
 * @param   无
 * @retval  SCL引脚电平（0或1）
 * @note    静态函数，仅供本模块内部调用
 */
uint8_t MyI2C_R_SCL(void)
{
    return GPIO_ReadInputDataBit(GPIOPort, SCL_Pin);
}

/**
 * @brief   读取SDA引脚电平
 * @param   无
 * @retval  SDA引脚电平（0或1）
 * @note    静态函数，仅供本模块内部调用
 */
uint8_t MyI2C_R_SDA(void)
{
    return GPIO_ReadInputDataBit(GPIOPort, SDA_Pin);
}

/**
 * @brief   I2C起始条件
 * @param   无
 * @retval  无
 * 
 * @note    起始条件时序：
 *          1. SDA = 1, SCL = 1（确保总线空闲）
 *          2. SDA = 0（SCL为高时SDA下降沿）
 *          3. SCL = 0（占用总线）
 * 
 * @par     使用场景：
 *          每次通信开始前调用，标识通信开始
 */
void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);     /* SDA先置高 */
    MyI2C_W_SCL(1);     /* SCL置高 */
    MyI2C_W_SDA(0);     /* SCL高时SDA下降沿产生起始条件 */
    MyI2C_W_SCL(0);     /* SCL拉低，准备发送数据 */
}

/**
 * @brief   I2C停止条件
 * @param   无
 * @retval  无
 * 
 * @note    停止条件时序：
 *          1. SDA = 0, SCL = 1
 *          2. SDA = 1（SCL为高时SDA上升沿）
 * 
 * @par     使用场景：
 *          每次通信结束后调用，释放总线
 */
void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);     /* SDA先置低 */
    MyI2C_W_SCL(1);     /* SCL置高 */
    MyI2C_W_SDA(1);     /* SCL高时SDA上升沿产生停止条件 */
}

/**
 * @brief   I2C发送一个字节
 * @param   Byte   要发送的字节数据（0x00~0xFF）
 * @retval  无
 * 
 * @note    时序说明：
 *          - 高位先行（MSB First），依次发送bit7~bit0
 *          - SCL低电平时改变SDA数据
 *          - SCL高电平时从机读取SDA
 * 
 * @par     示例：
 *          发送0xA5（二进制10100101）
 *          依次发送：1,0,1,0,0,1,0,1
 */
void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    
    for(i = 0; i < 8; i++)
    {
        /* 发送当前位（高位先行） */
        MyI2C_W_SDA(Byte & (0x80 >> i));    /* 提取第i位（从高位开始） */
        MyI2C_W_SCL(1);                     /* SCL高电平，从机读取数据 */
        MyI2C_W_SCL(0);                     /* SCL低电平，准备发送下一位 */
    }
}

/**
 * @brief   I2C接收一个字节
 * @param   无
 * @retval  接收到的字节数据（0x00~0xFF）
 * 
 * @note    时序说明：
 *          - 高位先行（MSB First），依次接收bit7~bit0
 *          - SCL高电平时读取SDA数据
 *          - 每读取一位后拉低SCL，等待下一位
 */
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t Byte = 0x00;
    
    MyI2C_W_SDA(1);                         /* 释放SDA总线，让从机控制 */
    
    for(i = 0; i < 8; i++)
    {
        MyI2C_W_SCL(1);                     /* SCL高电平，从机输出数据 */
        
        if(MyI2C_R_SDA())                   /* 读取SDA引脚 */
        {
            Byte |= (0x80 >> i);            /* 读取到1，将对应位置1 */
        }
        
        MyI2C_W_SCL(0);                     /* SCL低电平，准备读取下一位 */
    }
    
    return Byte;
}

/**
 * @brief   I2C接收应答信号
 * @param   无
 * @retval  应答值
 *          - 0：应答（ACK），从机表示可以继续接收
 *          - 1：非应答（NACK），从机表示无法继续或数据已结束
 * 
 * @note    主机在第9个时钟周期释放SDA总线
 *         从机在此时拉低SDA表示应答（ACK）
 *         保持高电平表示非应答（NACK）
 */
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t Ack;
    
    MyI2C_W_SDA(1);                         /* 主机释放SDA总线 */
    MyI2C_W_SCL(1);                         /* SCL高电平，从机输出应答 */
    Ack = MyI2C_R_SDA();                    /* 读取应答信号 */
    MyI2C_W_SCL(0);                         /* SCL低电平，结束应答阶段 */
    
    return Ack;
}

/**
 * @brief   I2C发送应答信号
 * @param   Ack   应答值
 *          - 0：应答（ACK），表示主机希望从机继续发送数据
 *          - 1：非应答（NACK），表示主机希望从机停止发送
 * @retval  无
 * 
 * @note    主机在第9个时钟周期控制SDA总线
 *         拉低SDA表示应答（ACK），保持高电平表示非应答（NACK）
 * 
 * @par     使用场景：
 *          - 连续读取数据时，前几个字节发送ACK，最后一个字节发送NACK
 *          - 写操作不需要主机发送应答
 */
void MyI2C_SendAck(uint8_t Ack)
{
    MyI2C_W_SDA(Ack);                       /* 设置应答信号 */
    MyI2C_W_SCL(1);                         /* SCL高电平，从机读取应答 */
    MyI2C_W_SCL(0);                         /* SCL低电平，结束应答阶段 */
}
