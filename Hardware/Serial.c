/**
 * @file    Serial.c
 * @brief   串口通信驱动模块实现
 * 
 * @note    本模块实现双串口的初始化、发送、接收及中断处理功能
 *         支持USART1（调试串口）和USART2（蓝牙串口）
 *         接收数据使用帧格式：[数据内容]
 */

#include "Serial.h"

/*============================================================================
 * 全局串口设备实例定义
 *============================================================================*/

/**
 * @brief   调试串口设备实例（USART1）
 * @note    用于程序调试和printf输出
 *          TX: PA9, RX: PA10
 *          中断优先级：抢占优先级1，子优先级1
 */
Serial_Device_t Serial = {
    .USARTx = USART1,
    .GPIOx = GPIOA,
    .RCC_USART = RCC_APB2Periph_USART1,
    .RCC_GPIO = RCC_APB2Periph_GPIOA,
    .Pin_Tx = GPIO_Pin_9,
    .Pin_Rx = GPIO_Pin_10,
    .IRQn = USART1_IRQn,
    .PreemptPriority = 1,
    .SubPriority = 1,
};

/**
 * @brief   蓝牙串口设备实例（USART2）
 * @note    用于无线调试和遥控指令接收
 *          TX: PA2, RX: PA3
 *          中断优先级：抢占优先级2，子优先级2
 */
Serial_Device_t Serial_Bluetooth = {
    .USARTx = USART2,
    .GPIOx = GPIOA,
    .RCC_USART = RCC_APB1Periph_USART2,
    .RCC_GPIO = RCC_APB2Periph_GPIOA,
    .Pin_Tx = GPIO_Pin_2,
    .Pin_Rx = GPIO_Pin_3,
    .IRQn = USART2_IRQn,
    .PreemptPriority = 2,    // 中断优先级
    .SubPriority = 2,
};

/*============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief   串口初始化函数
 * @param   serial_dev   串口设备结构体指针
 * @retval  无
 * 
 * @note    此函数完成以下配置：
 *          1. 使能串口和GPIO时钟（自动区分APB1/APB2）
 *          2. 配置TX引脚为复用推挽输出模式
 *          3. 配置RX引脚为上拉输入模式
 *          4. 配置串口为9600波特率、8N1格式
 *          5. 使能接收中断并配置NVIC优先级
 *          6. 开启串口
 */
void Serial_Init(Serial_Device_t* serial_dev)
{
    // 空指针校验
    if (serial_dev == NULL)
    {
        return;
    }
    
    /*-------------------------- 时钟使能 --------------------------*/
    // 区分 APB1 / APB2 总线时钟使能
    if(serial_dev->RCC_USART == RCC_APB2Periph_USART1)
    {
        RCC_APB2PeriphClockCmd(serial_dev->RCC_USART, ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(serial_dev->RCC_USART, ENABLE);
    }
    RCC_APB2PeriphClockCmd(serial_dev->RCC_GPIO, ENABLE);
    
    /*-------------------------- 引脚配置 --------------------------*/
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 配置 TX 引脚：复用推挽输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = serial_dev->Pin_Tx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(serial_dev->GPIOx, &GPIO_InitStruct);
    
    // 配置 RX 引脚：上拉输入
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin = serial_dev->Pin_Rx;
    GPIO_Init(serial_dev->GPIOx, &GPIO_InitStruct);
    
    /*-------------------------- 串口基本配置 --------------------------*/
    // 波特率9600，8数据位，1停止位，无校验，无硬件流控
    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = SERIAL_BaudRate;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(serial_dev->USARTx, &USART_InitStruct);
    
    /*-------------------------- 中断配置 --------------------------*/
    // 使能接收数据寄存器非空中断
    USART_ITConfig(serial_dev->USARTx, USART_IT_RXNE, ENABLE);
    
    // 配置NVIC中断优先级
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = serial_dev->IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = serial_dev->PreemptPriority;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = serial_dev->SubPriority;
    NVIC_Init(&NVIC_InitStruct);
	
    // 开启串口
    USART_Cmd(serial_dev->USARTx, ENABLE);
}

/**
 * @brief   串口发送单字节数据
 * @param   serial_dev   串口设备结构体指针
 * @param   Byte         待发送的单字节数据
 * @retval  无
 * 
 * @note    该函数为阻塞发送，会等待发送完成后再返回
 */
void Serial_SendByte(Serial_Device_t* serial_dev, uint8_t Byte)
{
    if (serial_dev == NULL) return;
    
    USART_SendData(serial_dev->USARTx, Byte);
    // 等待发送数据寄存器为空
    while(USART_GetFlagStatus(serial_dev->USARTx, USART_FLAG_TXE) == RESET);
}

/**
 * @brief   串口发送字符串
 * @param   serial_dev   串口设备结构体指针
 * @param   String       待发送的字符串指针（以'\0'结尾）
 * @retval  无
 * 
 * @note    循环调用Serial_SendByte发送每个字符直到字符串结束符
 */
void Serial_SendString(Serial_Device_t* serial_dev, char *String)
{
	if (serial_dev == NULL) return;
	for(int i = 0; String[i] != '\0'; i++)
	{
		Serial_SendByte(serial_dev, String[i]);
	}
}

/**
 * @brief   串口格式化打印（类似printf）
 * @param   serial_dev   串口设备结构体指针
 * @param   format       格式化字符串
 * @param   ...          可变参数列表
 * @retval  无
 * 
 * @note    内部使用vsprintf将格式化字符串输出到缓冲区
 *         缓冲区大小为100字节，注意避免溢出
 */
void Serial_Printf(Serial_Device_t* serial_dev, char *format, ...)
{
	char String[100];               // 定义字符数组缓冲区
	va_list arg;                    // 定义可变参数列表变量
	va_start(arg, format);          // 从format开始接收参数列表
	vsprintf(String, format, arg);  // 格式化字符串到缓冲区
	va_end(arg);                    // 结束可变参数
	Serial_SendString(serial_dev, String);  // 串口发送格式化后的字符串
}

/**
 * @brief   获取串口接收完成标志并清除
 * @param   serial_dev   串口设备结构体指针
 * @retval  1 - 收到一帧完整数据（RxFlag为1）
 * @retval  0 - 未收到完整数据
 * 
 * @note    读取RxFlag后会自动清零，避免重复处理同一帧数据
 */
uint8_t Serial_GetRxFlag(Serial_Device_t* serial_dev)
{
	if (serial_dev == NULL) return 0;
	if(serial_dev->RxFlag)
	{
		serial_dev->RxFlag = 0;     // 清除接收完成标志
		return 1;
	}
	return 0;
}

/**
 * @brief   获取接收到的数据缓冲区指针
 * @param   serial_dev   串口设备结构体指针
 * @retval  数据缓冲区指针（RxBuf），失败返回NULL
 * 
 * @note    使用此函数前应先调用Serial_GetRxFlag确认收到数据
 */
uint8_t* Serial_GetData(Serial_Device_t* serial_dev)
{
    if (serial_dev == NULL) return NULL;
    return serial_dev->RxBuf;
}

/**
 * @brief   串口接收中断处理核心函数（状态机）
 * @param   serial_dev    串口设备结构体指针
 * @param   ReceiveData   接收到的单字节数据
 * @retval  无
 * 
 * @note    帧格式说明：
 *          - 帧起始符：'['
 *          - 帧结束符：']'
 *          - 数据内容为起始符和结束符之间的字符
 *          - 收到结束符后自动添加字符串结束符'\0'并置位RxFlag
 * 
 * @par     状态机说明：
 *          状态0：等待帧起始符'['
 *          状态1：接收数据，直到收到结束符']'
 */
void Serial_RxIrqHandler(Serial_Device_t* serial_dev, uint8_t ReceiveData)
{
	if (serial_dev == NULL) return;
	
	// 状态0：等待帧起始符
	if(ReceiveData == '[' && serial_dev->RxState == 0)
	{
		serial_dev->RxBufcnt = 0;      // 重置缓冲区计数
		serial_dev->RxState = 1;       // 切换到接收数据状态
	}
	// 状态1：接收数据
	else if(serial_dev->RxState == 1 && serial_dev->RxBufcnt < SERIAL_BufLength)
	{
		if(ReceiveData == ']')          // 检测到帧结束符
		{
			serial_dev->RxBuf[serial_dev->RxBufcnt++] = '\0';  // 添加字符串结束符
			serial_dev->RxState = 0;     // 回到等待起始符状态
			serial_dev->RxFlag = 1;      // 置位接收完成标志
		}
		else                            // 普通数据，存入缓冲区
		{
			serial_dev->RxBuf[serial_dev->RxBufcnt++] = ReceiveData;
		}
	}
}

/*============================================================================
 * 中断服务函数
 *============================================================================*/

/**
 * @brief   USART1 中断服务函数
 * @retval  无
 * @note    处理USART1（调试串口）的接收中断
 *          - 读取接收数据
 *          - 调用状态机进行处理
 *          - 清除中断标志位
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(Serial.USARTx, USART_IT_RXNE) == SET)
    {
        uint8_t ReceiveData = USART_ReceiveData(Serial.USARTx);
        Serial_RxIrqHandler(&Serial, ReceiveData);
        USART_ClearITPendingBit(Serial.USARTx, USART_IT_RXNE);
    }
}

/**
 * @brief   USART2 中断服务函数
 * @retval  无
 * @note    处理USART2（蓝牙串口）的接收中断
 *          - 读取接收数据
 *          - 调用状态机进行处理
 *          - 清除中断标志位
 */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(Serial_Bluetooth.USARTx, USART_IT_RXNE) == SET)
    {
        uint8_t ReceiveData = USART_ReceiveData(Serial_Bluetooth.USARTx);
		Serial_RxIrqHandler(&Serial_Bluetooth, ReceiveData);
        USART_ClearITPendingBit(Serial_Bluetooth.USARTx, USART_IT_RXNE);
    }
}

/*============================================================================
 * printf重定向
 *============================================================================*/

/**
 * @brief   printf函数重定向
 * @param   ch   要输出的字符
 * @param   f    文件指针（未使用）
 * @retval  输出的字符
 * 
 * @note    将标准库printf的输出重定向到USART1调试串口
 *         需在工程设置中勾选"Use MicroLIB"或实现相应的底层函数
 */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(&Serial, ch);
	return ch;
}
