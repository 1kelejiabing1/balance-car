#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"                  // 设备头文件
#include <stdio.h>
#include <stdarg.h>


/**
 * @brief 串口配置宏定义
 * @{
 */
#define SERIAL_BufLength				320
// 串口波特率（统一使用9600）
#define SERIAL_BaudRate       	9600
/** @} */

/**
 * @brief 串口设备结构体
 * @note  用于封装多串口硬件信息与接收状态，实现双串口复用驱动
 *        包含：硬件配置 + 接收缓存 + 状态机
 */
typedef struct
{
    /*--------------------- 硬件配置参数 ---------------------*/
    USART_TypeDef*        	USARTx;            // 串口外设（USART1/USART3）
    GPIO_TypeDef*         	GPIOx;             // 串口对应GPIO端口
    uint32_t              	RCC_USART;         // 串口时钟
    uint32_t              	RCC_GPIO;          // GPIO时钟
    uint16_t              	Pin_Tx;            // TX发送引脚
    uint16_t              	Pin_Rx;            // RX接收引脚
    uint8_t               	IRQn;               // 中断号
    uint8_t               	PreemptPriority;    // 抢占优先级
    uint8_t              	SubPriority;        // 子优先级

    /*--------------------- 接收状态与缓存 ---------------------*/
    uint8_t               	RxBuf[SERIAL_BufLength];  						// 接收数据缓冲区
	uint8_t					RxBufcnt;
    uint8_t               	RxFlag;                     // 一帧数据接收完成标志
	uint8_t 				RxState;
} Serial_Device_t;

/**
 * @brief 外部声明全局串口设备实例
 * @{
 */
// 蓝牙模块串口（USART2）
extern Serial_Device_t Serial_Bluetooth;
// 模块串口（USART）
extern Serial_Device_t Serial;
/** @} */

/*--------------------- 外部接口函数 ---------------------*/

/**
 * @brief  串口初始化函数
 * @param  serial_dev: 串口设备结构体指针
 * @retval 无
 */
void Serial_Init(Serial_Device_t* serial_dev);

/**
 * @brief  串口发送单字节
 * @param  serial_dev: 串口设备指针
 * @param  Byte: 待发送字节
 * @retval 无
 */
void Serial_SendByte(Serial_Device_t* serial_dev, uint8_t Byte);

void Serial_Printf(Serial_Device_t* serial_dev, char *format, ...);

uint8_t Serial_GetRxFlag(Serial_Device_t* serial_dev);

/**
 * @brief  获取接收到的一帧数据
 * @param  serial_dev: 串口设备指针
 * @retval 数据缓冲区指针
 */
uint8_t* Serial_GetData(Serial_Device_t* serial_dev);

void Serial_RxIrqHandler(Serial_Device_t* serial_dev, uint8_t ReceiveData);
#endif