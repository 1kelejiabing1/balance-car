/**
 * @file    Serial.h
 * @brief   串口通信驱动模块头文件
 * 
 * @note    本模块提供双串口（USART1/USART2）的驱动接口
 *         支持调试串口和蓝牙串口，采用帧格式接收数据：[数据内容]
 */

#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"                  /* 设备头文件 */
#include <stdio.h>                      /* printf函数支持 */
#include <stdarg.h>                     /* 可变参数支持 */

/*============================================================================
 * 配置宏定义
 *============================================================================*/

/**
 * @defgroup Serial_Config 串口配置参数
 * @{
 */

/**
 * @brief   串口接收缓冲区大小（字节）
 * @note    最大支持320字节的帧数据，超过此长度的数据将被截断
 */
#define SERIAL_BufLength    320

/**
 * @brief   串口通信波特率
 * @note    统一使用9600bps，如需修改请同步修改上位机配置
 */
#define SERIAL_BaudRate     9600

/** @} */

/*============================================================================
 * 数据结构定义
 *============================================================================*/

/**
 * @brief   串口设备结构体
 * @note    封装多串口硬件信息与接收状态，实现双串口复用驱动
 *         包含硬件配置参数和接收状态机
 * 
 * @par     帧接收格式说明：
 *          - 帧起始标志：'['  (ASCII 0x5B)
 *          - 帧结束标志：']'  (ASCII 0x5D)
 *          - 示例：`[data1,data2,data3]`
 * 
 * @par     状态机说明：
 *          - RxState = 0：等待帧起始符'['
 *          - RxState = 1：正在接收数据，直到遇到']'结束
 */
typedef struct
{
    /*--------------------- 硬件配置参数 ---------------------*/
    USART_TypeDef*  USARTx;         /**< 串口外设基地址（USART1/USART2/USART3等） */
    GPIO_TypeDef*   GPIOx;          /**< 串口引脚对应的GPIO端口 */
    uint32_t        RCC_USART;      /**< 串口外设时钟使能宏（RCC_APBxPeriph_USARTx） */
    uint32_t        RCC_GPIO;       /**< GPIO端口时钟使能宏（RCC_APB2Periph_GPIOx） */
    uint16_t        Pin_Tx;         /**< TX发送引脚（如GPIO_Pin_9） */
    uint16_t        Pin_Rx;         /**< RX接收引脚（如GPIO_Pin_10） */
    uint8_t         IRQn;           /**< 串口中断号（如USART1_IRQn） */
    uint8_t         PreemptPriority;/**< NVIC抢占优先级（数值越小优先级越高） */
    uint8_t         SubPriority;    /**< NVIC子优先级（数值越小优先级越高） */

    /*--------------------- 接收状态与缓存 ---------------------*/
    uint8_t         RxBuf[SERIAL_BufLength];    /**< 接收数据缓冲区，存储一帧完整数据 */
    uint8_t         RxBufcnt;                   /**< 当前已接收字节数计数器 */
    uint8_t         RxFlag;                     /**< 一帧数据接收完成标志（1：收到完整帧） */
    uint8_t         RxState;                    /**< 接收状态机状态（0：等待帧头，1：接收数据） */
} Serial_Device_t;

/*============================================================================
 * 外部全局变量声明
 *============================================================================*/

/**
 * @defgroup Serial_GlobalInstances 全局串口实例
 * @{
 */

/**
 * @brief   调试串口设备实例（USART1）
 * @note    用于printf输出和程序调试信息打印
 *         引脚：TX=PA9, RX=PA10
 *         中断优先级：抢占优先级1，子优先级1
 */
extern Serial_Device_t Serial;

/**
 * @brief   蓝牙串口设备实例（USART2）
 * @note    用于无线遥控指令接收和参数调试
 *         引脚：TX=PA2, RX=PA3
 *         中断优先级：抢占优先级2，子优先级2
 */
extern Serial_Device_t Serial_Bluetooth;

/** @} */

/*============================================================================
 * API函数声明
 *============================================================================*/

/**
 * @defgroup Serial_API 串口API函数
 * @{
 */

/**
 * @brief   串口初始化函数
 * @param   serial_dev   串口设备结构体指针
 * @retval  无
 * 
 * @note    此函数会完成以下初始化：
 *          - 使能串口和GPIO时钟
 *          - 配置TX/RX引脚模式
 *          - 配置串口参数（波特率9600/8N1）
 *          - 使能接收中断并配置NVIC
 *          - 开启串口
 * 
 * @warning 传入的serial_dev指针不能为NULL
 * 
 * @par     使用示例：
 * @code
 *   // 初始化调试串口
 *   Serial_Init(&Serial);
 *   // 初始化蓝牙串口
 *   Serial_Init(&Serial_Bluetooth);
 * @endcode
 */
void Serial_Init(Serial_Device_t* serial_dev);

/**
 * @brief   串口发送单字节数据
 * @param   serial_dev   串口设备结构体指针
 * @param   Byte         待发送的单字节数据（0x00~0xFF）
 * @retval  无
 * 
 * @note    该函数为阻塞发送，会等待发送完成后再返回
 *         发送前会自动等待上一次发送完成
 * 
 * @par     使用示例：
 * @code
 *   Serial_SendByte(&Serial, 'A');
 *   Serial_SendByte(&Serial, 0x0D);  // 发送回车符
 * @endcode
 */
void Serial_SendByte(Serial_Device_t* serial_dev, uint8_t Byte);

/**
 * @brief   串口格式化输出（增强型printf）
 * @param   serial_dev   串口设备结构体指针
 * @param   format       格式化字符串（同printf语法）
 * @param   ...          可变参数列表
 * @retval  无
 * 
 * @note    内部缓冲区大小为100字节，格式化后总长度不应超过缓冲区
 *         支持标准printf的格式化控制符：%d、%f、%s、%x等
 * 
 * @par     使用示例：
 * @code
 *   uint8_t speed = 100;
 *   float angle = 45.5f;
 *   Serial_Printf(&Serial_Bluetooth, "[plot,%d,%.1f]", speed, angle);
 * @endcode
 */
void Serial_Printf(Serial_Device_t* serial_dev, char *format, ...);

/**
 * @brief   获取串口接收完成标志并自动清除
 * @param   serial_dev   串口设备结构体指针
 * @retval  1 - 收到一帧完整数据（RxFlag为1）
 * @retval  0 - 未收到完整数据
 * 
 * @note    此函数为边缘触发模式：读取RxFlag后会自动清零
 *         防止重复处理同一帧数据，建议在主循环中轮询调用
 * 
 * @par     使用示例：
 * @code
 *   if (Serial_GetRxFlag(&Serial_Bluetooth))
 *   {
 *       uint8_t* pData = Serial_GetData(&Serial_Bluetooth);
 *       // 处理接收到的数据...
 *   }
 * @endcode
 */
uint8_t Serial_GetRxFlag(Serial_Device_t* serial_dev);

/**
 * @brief   获取接收到的数据缓冲区指针
 * @param   serial_dev   串口设备结构体指针
 * @retval  数据缓冲区指针（指向RxBuf），失败返回NULL
 * 
 * @note    使用此函数前应先调用Serial_GetRxFlag确认收到数据
 *         返回的缓冲区是以'\0'结尾的字符串
 * 
 * @par     使用示例：
 * @code
 *   if (Serial_GetRxFlag(&Serial_Bluetooth))
 *   {
 *       uint8_t* data = Serial_GetData(&Serial_Bluetooth);
 *       // data的内容如："key,K1,press"
 *   }
 * @endcode
 */
uint8_t* Serial_GetData(Serial_Device_t* serial_dev);

/**
 * @brief   串口接收中断处理核心函数（状态机）
 * @param   serial_dev    串口设备结构体指针
 * @param   ReceiveData   接收到的单字节数据
 * @retval  无
 * 
 * @note    此函数由中断服务函数调用，实现帧数据的解析
 *         状态机说明：
 *         - 状态0（RxState=0）：等待帧起始符'['
 *         - 状态1（RxState=1）：接收数据直到遇到']'结束符
 *         收到完整帧后自动添加'\0'并置位RxFlag
 * 
 * @warning 此函数不应在用户代码中直接调用，由中断服务函数自动调用
 * 
 * @par     帧格式示例：
 * @code
 *   [key,K1,press]      // 按键数据包
 *   [slider,SpeedKp,2.5] // 滑杆数据包
 *   [joystick,0,100,50,0] // 摇杆数据包
 * @endcode
 */
void Serial_RxIrqHandler(Serial_Device_t* serial_dev, uint8_t ReceiveData);

/** @} */

#endif /* __SERIAL_H */
